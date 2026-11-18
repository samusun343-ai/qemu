/*
 * SHARP Brain GPIO matrix keyboard.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Hardware (PW-SH6 platform, from linux-brain imx28-pwsh6.dts):
 *
 *   rows (outputs, guest-driven):  GPIO4_0 .. GPIO4_7
 *   columns (inputs, pulled up):   GPIO2_16..GPIO2_21, GPIO4_8
 *
 * The guest scans the matrix by driving the rows low (all of them
 * for interrupt detection, one at a time while scanning); a pressed
 * key pulls its column low.  This model keeps a bitmask of closed
 * contacts and re-evaluates the column input levels whenever the
 * guest re-drives the rows (observed through the PINCTRL output
 * hook) or a host key event changes the contact mask.
 *
 * Host key -> guest mapping policy (keys arrive as linux keycodes):
 *
 *   - PC keys that exist on the Brain (letters, space, return,
 *     backspace, arrows, shift, minus) close the corresponding
 *     matrix contact 1:1.  Host Esc closes the 戻る contact (the key
 *     SHARP's manuals call the "Escape key"), host Home closes the
 *     Brain Home contact, and host Delete is an alias for the Brain
 *     削除 (Backspace) contact.
 *   - Digits and punctuation only exist as the blue "symbols" legend
 *     of the QWERTY keys (記号 modifier + key); pressing them on the
 *     host holds the symbols contact plus the base key contact, which
 *     is exactly what typing them on the real device does.  Glyphs
 *     on the Shift+symbols layer (the US-shifted forms of the blue
 *     legends: ! @ # $ % ^ & * ( ) ~ + | : " { } < > ?) are reached
 *     by holding the symbols and Shift contacts together with the
 *     base key.
 *   - Host keycodes are positional: the same keycode is printed
 *     with a different glyph on a US and on a JIS keyboard (e.g.
 *     KEY_LEFTBRACE is "[" on US, "@" on JIS).  The "host-layout"
 *     property ("jis" by default -- the Brain is a Japanese-market
 *     device -- or "us") selects the chord table so that the glyph
 *     printed on the host key is the glyph the chord produces.
 *     Keys that only exist on JIS keyboards (Yen, Ro, Henkan,
 *     Muhenkan, Zenkakuhankaku, KatakanaHiragana) and the ISO
 *     102nd key have explicit assignments in both layouts.
 *   - Brain-specific dictionary keys that have no PC equivalent
 *     (History, My辞書, 一覧, 国語, 音声, 英和, 記号,
 *     marker, search) are reached through Ctrl+letter combinations.
 *     戻る also has a combo (Ctrl+B) for keyboards without Esc.
 *   - The mapping a key-down applies is latched per host key, so a
 *     key-up always releases the same contacts even if Ctrl was
 *     released first; contacts are refcounted because several host
 *     keys can hold one contact (both Shifts, the symbols layer).
 *     Ctrl/Shift state is tracked per key so frontend autorepeat
 *     cannot inflate it.
 *   - Chord timing: the WinCE keyboard driver folds the symbols
 *     layer only when the 記号 contact has been latched as a
 *     modifier well before the base key's press edge -- which is
 *     what a human typist produces, since the modifier finger
 *     lands first.  A chord whose contacts all close in the same
 *     scan is reported as two independent key presses (the base
 *     key plus a bare 記号 press, which e.g. the dictionary
 *     search field turns into its "~" wildcard: "<char>~" /
 *     "~<char>" pairs).  So while the guest scans in the WinCE
 *     direction, a symbols chord's base contact is queued until
 *     the guest has swept the matrix with the modifiers down and
 *     a minimum stagger has elapsed; keys pressed meanwhile queue
 *     behind it in host order, and a release that arrives early
 *     takes effect just after the base lands, so the chord always
 *     completes whole.  The Brainux driver folds only same-scan
 *     chords, so while the guest drives the columns (its scan
 *     direction) chords stay atomic.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "standard-headers/linux/input-event-codes.h"
#include "ui/console.h"
#include "ui/input.h"
#include "hw/core/qdev-properties.h"
#include "brain.h"

#define SYM_ROW 2
#define SYM_COL 4

/* ------------------------------------------------------------------ */
/* PW-SH6 / PW-AJ2 layout                                              */
/* ------------------------------------------------------------------ */

/*
 * Physical matrix (row, col), from linux-brain imx28-pwsh6.dts.
 * The (row, col) key table is the same, but the SCAN DIRECTION is
 * the opposite of the first guess: the guest drives the ROWS and
 * reads the COLUMNS.  Proven by the guest's own GPIO programming
 * (boottest16 trace): the KBD driver configures GPIO4_0..4_7 as
 * outputs (DOE set, DOUT low) and arms PINCTRL interrupts on the
 * columns GPIO2_16..2_21 + GPIO4_8 (PIN2IRQ/IRQLEV/IRQPOL/IRQEN),
 * i.e. a column going low (key pressed, row driven low) is the
 * interrupt source.  The columns are pulled up on the board, so
 * they idle high; a pressed key pulls its column low.
 *
 *        col0     col1     col2      col3     col4     col5    col6
 *  row0  V        H        Y         戻る     C        D       -
 *  row1  M        K        History   Down     PageUp   F       E
 *  row2  B        J        U         Left     記号     X       -
 *  row3  N        I        My辞書    Up       PageDown Q       Home
 *  row4  Backspace G      T         Right    Shift    A       -
 *  row5  P        O        一覧      Enter    Z        W       国語
 *  row6  Minus    L        R         Space    音声     S       英和
 *  row7  -        -        -         -        -        -       -
 *
 * rows = guest-driven outputs.  Logical row -> GPIO4 bit (from
 * brainux brain-kbd-gpio.c: GPIO4_5 is unused; row 5 = GPIO4_6,
 * row 6 = GPIO4_7): row0=GPIO4_0, row1=GPIO4_1, row2=GPIO4_2,
 * row3=GPIO4_3, row4=GPIO4_4, row5=GPIO4_6, row6=GPIO4_7.
 * col0..col5 = GPIO2_16..GPIO2_21, col6 = GPIO4_8 (pulled-up inputs).
 */

#define D BRAIN_KEY_DIRECT
#define S BRAIN_KEY_SYMBOL
#define H BRAIN_KEY_SHIFT
#define N BRAIN_KEY_NOSHIFT

static const BrainKeyDef pwsh6_direct[KEY_CNT] = {
    /* letters (direct contacts) */
    [KEY_A]                    = { 4, 5, D },   /* A */
    [KEY_B]                    = { 2, 0, D },   /* B */
    [KEY_C]                    = { 0, 4, D },   /* C */
    [KEY_D]                    = { 0, 5, D },   /* D */
    [KEY_E]                    = { 1, 6, D },   /* E */
    [KEY_F]                    = { 1, 5, D },   /* F */
    [KEY_G]                    = { 4, 1, D },   /* G */
    [KEY_H]                    = { 0, 1, D },   /* H */
    [KEY_I]                    = { 3, 1, D },   /* I */
    [KEY_J]                    = { 2, 1, D },   /* J */
    [KEY_K]                    = { 1, 1, D },   /* K */
    [KEY_L]                    = { 6, 1, D },   /* L */
    [KEY_M]                    = { 1, 0, D },   /* M */
    [KEY_N]                    = { 3, 0, D },   /* N */
    [KEY_O]                    = { 5, 1, D },   /* O */
    [KEY_P]                    = { 5, 0, D },   /* P */
    [KEY_Q]                    = { 3, 5, D },   /* Q */
    [KEY_R]                    = { 6, 2, D },   /* R */
    [KEY_S]                    = { 6, 5, D },   /* S */
    [KEY_T]                    = { 4, 2, D },   /* T */
    [KEY_U]                    = { 2, 2, D },   /* U */
    [KEY_V]                    = { 0, 0, D },   /* V */
    [KEY_W]                    = { 5, 5, D },   /* W */
    [KEY_X]                    = { 2, 5, D },   /* X */
    [KEY_Y]                    = { 0, 2, D },   /* Y */
    [KEY_Z]                    = { 5, 4, D },   /* Z */

    /* direct keys that exist on a PC keyboard too */
    [KEY_MINUS]                = { 6, 0, D },   /* - */
    [KEY_SPACE]                = { 6, 3, D },   /* Space */
    [KEY_ENTER]                = { 5, 3, D },   /* Enter */
    [KEY_KPENTER]              = { 5, 3, D },
    [KEY_BACKSPACE]            = { 4, 0, D },   /* Backspace (Brain 削除) */
    [KEY_DELETE]               = { 4, 0, D },   /* alias: same 削除 contact */
    [KEY_UP]                   = { 3, 3, D },   /* Up */
    [KEY_DOWN]                 = { 1, 3, D },   /* Down */
    [KEY_LEFT]                 = { 2, 3, D },   /* Left */
    [KEY_RIGHT]                = { 4, 3, D },   /* Right */
    /* (the host Shift keys themselves are handled in the event path:
     * they drive the Shift contact through brain_kbd_sync_shift()) */
    [KEY_ESC]                  = { 0, 3, D },   /* 戻る: SHARP's own manuals
                                                 * call this key the
                                                 * "Escape key" (returns to
                                                 * the previous screen) */
    [KEY_HOME]                 = { 3, 6, D },   /* Home */
    [KEY_PAGEUP]               = { 1, 4, D },   /* Page Up */
    [KEY_PAGEDOWN]             = { 3, 4, D },   /* Page Down */
    [KEY_INSERT]               = { 1, 2, D },   /* History (alias) */

    /*
     * Digits and punctuation: the blue "symbols" legends.  記号 + base
     * key, exactly like typing them on the real device.
     */
    [KEY_1]                    = { 3, 5, S },   /* 記号+Q = 1 */
    [KEY_2]                    = { 5, 5, S },   /* 記号+W = 2 */
    [KEY_3]                    = { 1, 6, S },   /* 記号+E = 3 */
    [KEY_4]                    = { 6, 2, S },   /* 記号+R = 4 */
    [KEY_5]                    = { 4, 2, S },   /* 記号+T = 5 */
    [KEY_6]                    = { 0, 2, S },   /* 記号+Y = 6 */
    [KEY_7]                    = { 2, 2, S },   /* 記号+U = 7 */
    [KEY_8]                    = { 3, 1, S },   /* 記号+I = 8 */
    [KEY_9]                    = { 5, 1, S },   /* 記号+O = 9 */
    [KEY_0]                    = { 5, 0, S },   /* 記号+P = 0 */
    [KEY_GRAVE]                = { 0, 5, S },   /* 記号+D */
    [KEY_EQUAL]                = { 1, 5, S },   /* 記号+F */
    [KEY_BACKSLASH]            = { 4, 1, S },   /* 記号+G */
    [KEY_SEMICOLON]            = { 0, 1, S },   /* 記号+H */
    [KEY_APOSTROPHE]           = { 2, 1, S },   /* 記号+J */
    [KEY_LEFTBRACE]            = { 1, 1, S },   /* 記号+K */
    [KEY_RIGHTBRACE]           = { 6, 1, S },   /* 記号+L */
    [KEY_COMMA]                = { 3, 0, S },   /* 記号+N = , */
    [KEY_DOT]                  = { 1, 0, S },   /* 記号+M = . */
    [KEY_SLASH]                = { 6, 0, S },   /* 記号+minus = / */
    [KEY_KP1]                  = { 3, 5, S },
    [KEY_KP2]                  = { 5, 5, S },
    [KEY_KP3]                  = { 1, 6, S },
    [KEY_KP4]                  = { 6, 2, S },
    [KEY_KP5]                  = { 4, 2, S },
    [KEY_KP6]                  = { 0, 2, S },
    [KEY_KP7]                  = { 2, 2, S },
    [KEY_KP8]                  = { 3, 1, S },
    [KEY_KP9]                  = { 5, 1, S },
    [KEY_KP0]                  = { 5, 0, S },
    [KEY_KPDOT]                = { 1, 0, S },
    [KEY_KPSLASH]              = { 6, 0, S },
    [KEY_KPASTERISK]           = { 1, 5, S },
    [KEY_KPMINUS]              = { 6, 0, D },
    [KEY_KPPLUS]               = { 1, 5, S },
    [KEY_KPEQUAL]              = { 1, 5, S },
    [KEY_KPCOMMA]              = { 3, 0, S },   /* 記号+N = , */

    /*
     * Keys that exist only on JIS (or ISO) host keyboards.  These
     * keycodes never arrive from a US keyboard, so the assignments
     * are the same in both host layouts.
     */
    [KEY_YEN]                  = { 4, 1, S },   /* ¥: 記号+G = \ (the
                                                 * Brain has no ¥
                                                 * legend; \ is its
                                                 * glyph.  With Shift:
                                                 * 記号+Shift+G = |,
                                                 * the JIS key's own
                                                 * shifted legend) */
    [KEY_RO]                   = { 6, 0, D | H }, /* _: Shift+Minus
                                                 * (the Brain's shift
                                                 * legend for Minus) */
    [KEY_102ND]                = { 4, 1, S },   /* ISO extra key: \ */
    [KEY_HENKAN]               = { 6, 3, D },   /* 変換 = Space (the
                                                 * Brain Space key is
                                                 * スペース・変換) */
    [KEY_MUHENKAN]             = { 6, 3, D },   /* 無変換: Space alias */
    [KEY_ZENKAKUHANKAKU]       = { SYM_ROW, SYM_COL, D }, /* the 記号
                                                 * key itself: the
                                                 * Brain's input-mode
                                                 * key */
    [KEY_KATAKANAHIRAGANA]     = { SYM_ROW, SYM_COL, D }, /* ditto */
};

/*
 * JIS host layout: overrides over pwsh6_direct for the positional
 * keycodes whose printed glyph differs from US.  The chord for each
 * key produces the glyph printed on the JIS key:
 *   KEY_EQUAL      "^"  = 記号+Shift+Y   (US: "=" = 記号+F)
 *   KEY_LEFTBRACE  "@"  = 記号+Shift+W   (US: "[" = 記号+K)
 *   KEY_RIGHTBRACE "["  = 記号+K         (US: "]" = 記号+L)
 *   KEY_BACKSLASH  "]"  = 記号+L         (US: "\" = 記号+G)
 *   KEY_APOSTROPHE ":"  = 記号+Shift+H   (US: "'" = 記号+J)
 * (KEY_SEMICOLON is ";" in both layouts and needs no override; its
 * shifted form differs and lives in the shifted table.)
 */
static const BrainKeyDef pwsh6_jis_direct[KEY_CNT] = {
    [KEY_EQUAL]                = { 0, 2, S | H },   /* ^ */
    [KEY_LEFTBRACE]            = { 5, 5, S | H },   /* @ */
    [KEY_RIGHTBRACE]           = { 1, 1, S },       /* [ */
    [KEY_BACKSLASH]            = { 6, 1, S },       /* ] */
    [KEY_APOSTROPHE]           = { 0, 1, S | H },   /* : */
};

/*
 * JIS host layout, consulted when a host Shift key is held at
 * key-down time.  JIS shifted glyphs that equal the US-shifted form
 * of a symbols-layer chord (e.g. Shift+";" is "+" = 記号+Shift+F)
 * just name the base key: the host Shift contact is already closed,
 * so it supplies the Shift part of the chord.  Glyphs that live on
 * the *plain* symbols layer although the JIS key needs Shift for
 * them (Shift+7 is "'", Shift+- is "=") carry BRAIN_KEY_NOSHIFT so
 * the host Shift contribution is suppressed while they are held.
 */
static const BrainKeyDef pwsh6_jis_shifted[KEY_CNT] = {
    [KEY_2]                    = { 2, 1, S },       /* " = 記号+Shift+J */
    [KEY_6]                    = { 2, 2, S },       /* & = 記号+Shift+U */
    [KEY_7]                    = { 2, 1, S | N },   /* ' = 記号+J */
    [KEY_8]                    = { 5, 1, S },       /* ( = 記号+Shift+O */
    [KEY_9]                    = { 5, 0, S },       /* ) = 記号+Shift+P */
    [KEY_0]                    = { 5, 0, S | N },   /* (no JIS glyph:
                                                     * plain 0) */
    [KEY_MINUS]                = { 1, 5, S | N },   /* = = 記号+F */
    [KEY_EQUAL]                = { 0, 5, S },       /* ~ = 記号+Shift+D */
    [KEY_LEFTBRACE]            = { 0, 5, S | N },   /* ` = 記号+D */
    [KEY_SEMICOLON]            = { 1, 5, S },       /* + = 記号+Shift+F */
    [KEY_APOSTROPHE]           = { 3, 1, S },       /* * = 記号+Shift+I */
};

/*
 * Ctrl+letter combinations for the Brain-specific dictionary keys.
 * (Ctrl itself makes no contact; it only selects this table.)
 */
static const BrainKeyDef pwsh6_combo[KEY_CNT] = {
    [KEY_B]                    = { 0, 3, D },   /* Ctrl+B  -> 戻る (back) */
    [KEY_H]                    = { 1, 2, D },   /* Ctrl+H  -> 履歴 History */
    [KEY_D]                    = { 3, 2, D },   /* Ctrl+D  -> マイ辞書 */
    [KEY_L]                    = { 5, 2, D },   /* Ctrl+L  -> 一覧 (list) */
    [KEY_K]                    = { 5, 6, D },   /* Ctrl+K  -> 国語 */
    [KEY_J]                    = { 6, 6, D },   /* Ctrl+J  -> 英和 */
    [KEY_V]                    = { 6, 4, D },   /* Ctrl+V  -> 音声 (voice) */
    [KEY_S]                    = { SYM_ROW, SYM_COL, D }, /* Ctrl+S -> 記号 */
    [KEY_M]                    = { 5, 2, S },   /* Ctrl+M  -> マーカー */
    [KEY_F]                    = { 3, 6, S },   /* Ctrl+F  -> 検索 */
};

static const BrainKbdLayout pwsh6_layout = {
    .name        = "pw-sh6",
    .symbol_row  = SYM_ROW,
    .symbol_col  = SYM_COL,
    .shift_row   = 4,
    .shift_col   = 4,
    .direct      = pwsh6_direct,
    .combo       = pwsh6_combo,
    .jis_direct  = pwsh6_jis_direct,
    .jis_shifted = pwsh6_jis_shifted,
    .us_shifted  = NULL,    /* every US shifted glyph follows from the
                             * plain entry plus the passed-through
                             * host Shift contact */
};

static const BrainKbdLayout *brain_kbd_layouts[] = {
    &pwsh6_layout,
    NULL,
};

const BrainKbdLayout *brain_kbd_find_layout(const char *name)
{
    for (int i = 0; brain_kbd_layouts[i]; i++) {
        if (!strcmp(brain_kbd_layouts[i]->name, name)) {
            return brain_kbd_layouts[i];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* matrix model                                                        */
/* ------------------------------------------------------------------ */

#define CONTACT_BIT(r, c)   (1ULL << ((r) * BRAIN_KBD_COLS + (c)))

/* Logical row -> physical GPIO4 bit.
 * From brainux (brain-hackers/linux-brain, drivers/input/keyboard/
 * brain-kbd-gpio.c, `bk_gpio_read_keys`): the raw GPIO4 read is
 * decoded so that raw bits 6 and 7 map to rows 5 and 6, and raw
 * bit 5 (GPIO4_5) is UNUSED.  The board wires logical row 5 to
 * GPIO4_6 and logical row 6 to GPIO4_7; GPIO4_5 is not a row.
 * The WinCE driver drives the rows, so it must drive the same
 * physical pins to scan a given logical row.  Row 7 has no keys
 * and no pin. */
static const int brain_kbd_row_gpio_bit[BRAIN_KBD_ROWS] = {
    0, 1, 2, 3, 4, 6, 7, -1
};

/* row r is driven low by the guest (output enabled, DOUT clear) */
static bool brain_kbd_row_low(BrainKbdState *s, int r)
{
    int bit = brain_kbd_row_gpio_bit[r];

    if (bit < 0) {
        return false;
    }
    return (s->row_doe & (1u << bit)) && !(s->row_out & (1u << bit));
}

/* is column c currently driven low by the guest (transposed scan)? */
static bool brain_kbd_col_low(BrainKbdState *s, int c)
{
    if (c < 6) {
        uint32_t m = 1u << (16 + c);

        return (s->col2_doe & m) && !(s->col2_out & m);
    }
    return (s->col4_doe & 0x100u) && !(s->col4_out & 0x100u);
}

/*
 * Recompute the matrix levels from the pressed contacts, for both
 * scan directions guests use on this board:
 *
 * - WinCE drives the rows on GPIO4_0..7 and reads the columns
 *   (GPIO2_16..21 for col0..5, GPIO4_8 for col6): a pressed key
 *   pulls its column low while its row is driven low; otherwise
 *   the column stays pulled up (high).  Columns are the inputs
 *   that driver arms interrupts on.
 *
 * - Brainux/Linux drives the columns and reads the rows on
 *   GPIO4_0..7: a pressed key pulls its row low while its column
 *   is driven low; otherwise the row stays pulled up (high).
 *   GPIO4_5 is not a row (see the row map above) and always reads
 *   high.
 *
 * The two computations touch disjoint pin sets, and the pinctrl
 * only shows an external level on pins the guest holds as inputs,
 * so each guest sees exactly the direction it drives.
 */
static void brain_kbd_recalc(BrainKbdState *s)
{
    uint32_t cols2 = 0x3f0000;   /* columns idle high (pulled up) */
    uint32_t cols4 = 0x100;
    uint32_t rows = 0xff;        /* rows idle high (pulled up) */

    for (int r = 0; r < BRAIN_KBD_ROWS; r++) {
        int bit = brain_kbd_row_gpio_bit[r];

        for (int c = 0; c < BRAIN_KBD_COLS; c++) {
            if (!(s->pressed & CONTACT_BIT(r, c))) {
                continue;
            }
            if (brain_kbd_row_low(s, r)) {
                if (c < 6) {
                    cols2 &= ~(1u << (16 + c));
                } else {
                    cols4 &= ~(1u << 8);
                }
            }
            if (bit >= 0 && brain_kbd_col_low(s, c)) {
                rows &= ~(1u << bit);
            }
        }
    }

    mxs_pinctrl_set_input(s->pinctrl, 2, 0x3f0000, cols2);
    mxs_pinctrl_set_input(s->pinctrl, 4, 0x100, cols4);
    mxs_pinctrl_set_input(s->pinctrl, 4, 0xff, rows);
    /*
     * Re-arm the column IRQs: the guest KBD driver tears them down
     * after its first keypress and its IST never runs to re-arm
     * them, so without this the keyboard goes deaf after one key.
     * Safe when idle (columns high: no spurious IRQ) and when a key
     * is held (column low: the IRQ fires, delivering the keypress).
     */
    mxs_pinctrl_rearm_kbd_irq(s->pinctrl);
}

/*
 * Symbols-chord timing in the WinCE scan direction (see
 * brain_kbd_event()): the guest's driver latches 記号 as a
 * modifier only once it has been down on its own for a while --
 * a base contact that lands in the same scan window, or even a
 * couple of sweeps later, is reported as two independent key
 * presses.  So a chord's base contact is held back until the
 * guest has swept the matrix about twice with the modifiers
 * down (counted from its row-drive activity, ~14 transitions
 * per sweep) AND at least BRAIN_KBD_CHORD_MIN_MS of wall time
 * has passed, with a wall-clock fallback for a guest that is
 * not scanning.  Counting guest scan activity as well as time
 * keeps the stagger correct under host load.  Queued keys land
 * in press order, BRAIN_KBD_CHORD_GAP_MS apart, and a key whose
 * host release arrived while it was queued is released
 * BRAIN_KBD_CHORD_RELEASE_MS after its base lands, so even a
 * tap faster than the latch time completes as a whole chord.
 */
#define BRAIN_KBD_CHORD_SCAN_TICKS 28
#define BRAIN_KBD_CHORD_MIN_MS 200
#define BRAIN_KBD_CHORD_FALLBACK_MS 300
#define BRAIN_KBD_CHORD_GAP_MS 40
#define BRAIN_KBD_CHORD_RELEASE_MS 40

static void brain_kbd_chord_process(BrainKbdState *s);

/* pinctrl output hook: the guest re-drove one of the GPIO banks */
static void brain_kbd_col_notify(void *opaque, int bank, uint32_t out)
{
    BrainKbdState *s = opaque;

    if (bank == 4) {
        /* WinCE drives the rows on GPIO4_0..7; Brainux drives the
         * seventh column on GPIO4_8.  Refresh both views of bank 4
         * (effective output + output-enable) and re-derive levels. */
        s->row_out = out;
        s->row_doe = mxs_pinctrl_get_doe(s->pinctrl, bank);
        s->col4_out = out;
        s->col4_doe = s->row_doe;
        if (s->chord_qcount > 0 && s->chord_qstate[s->chord_queue[0]] == 1) {
            /*
             * A symbols chord is waiting for its base contact:
             * count the guest's row-drive transitions so the base
             * can land once the guest has swept the matrix with
             * the modifiers down (see brain_kbd_event()).
             */
            if (out != s->chord_last_out || s->row_doe != s->chord_last_doe) {
                s->chord_last_out = out;
                s->chord_last_doe = s->row_doe;
                s->chord_scan_ticks++;
                brain_kbd_chord_process(s);
            }
        }
        brain_kbd_recalc(s);
    } else if (bank == 2) {
        /* Brainux drives columns 0..5 on GPIO2_16..21. */
        s->col2_out = out;
        s->col2_doe = mxs_pinctrl_get_doe(s->pinctrl, bank);
        brain_kbd_recalc(s);
    }
}

/*
 * Close or open one matrix contact, reference-counted: several host
 * keys can hold the same contact at once (Left/Right Shift both mean
 * the Brain Shift key, Enter and KPEnter are the same contact, and
 * every symbols-layer key holds the 記号 contact in addition to its
 * own).  The contact opens only when its last holder releases.
 */
static void brain_kbd_set_contact(BrainKbdState *s, int r, int c, bool down)
{
    int idx;

    if (r < 0 || c < 0 || r >= BRAIN_KBD_ROWS || c >= BRAIN_KBD_COLS) {
        return;
    }
    idx = r * BRAIN_KBD_COLS + c;
    if (down) {
        if (s->contact_refs[idx] < UINT8_MAX) {
            s->contact_refs[idx]++;
        }
        s->pressed |= CONTACT_BIT(r, c);
    } else {
        if (s->contact_refs[idx] > 0) {
            s->contact_refs[idx]--;
        }
        if (s->contact_refs[idx] == 0) {
            s->pressed &= ~CONTACT_BIT(r, c);
        }
    }
}

/* ------------------------------------------------------------------ */
/* host keyboard input (events arrive as linux keycodes)               */
/* ------------------------------------------------------------------ */

/*
 * Drive the Shift contact from its three contributions: entries
 * that force it (BRAIN_KEY_SHIFT), the host Shift keys, and entries
 * that suppress the host Shift keys (BRAIN_KEY_NOSHIFT).  This is
 * the only path that touches the Shift contact, so it is applied
 * exactly on transitions.
 */
static void brain_kbd_sync_shift(BrainKbdState *s)
{
    const BrainKbdLayout *l = s->layout;
    bool want = s->shift_forced > 0 ||
                ((s->lshift_down || s->rshift_down) &&
                 s->shift_suppressed == 0);

    if (want == s->shift_applied) {
        return;
    }
    s->shift_applied = want;
    brain_kbd_set_contact(s, l->shift_row, l->shift_col, want);
}

/*
 * The modifier half of an entry: the 記号 contact plus the Shift
 * force/suppress bookkeeping.  Split out from brain_kbd_apply()
 * because a symbols chord in the WinCE scan direction applies its
 * modifiers first and its base contact a little later (see
 * brain_kbd_event()).
 */
static void brain_kbd_apply_mods(BrainKbdState *s, const BrainKeyDef *e,
                                 bool down)
{
    const BrainKbdLayout *l = s->layout;

    if (e->flags & BRAIN_KEY_SYMBOL) {
        brain_kbd_set_contact(s, l->symbol_row, l->symbol_col, down);
    }
    if (e->flags & BRAIN_KEY_SHIFT) {
        if (down) {
            s->shift_forced++;
        } else if (s->shift_forced) {
            s->shift_forced--;
        }
        brain_kbd_sync_shift(s);
    }
    if (e->flags & BRAIN_KEY_NOSHIFT) {
        if (down) {
            s->shift_suppressed++;
        } else if (s->shift_suppressed) {
            s->shift_suppressed--;
        }
        brain_kbd_sync_shift(s);
    }
}

static void brain_kbd_apply(BrainKbdState *s, const BrainKeyDef *e, bool down)
{
    if (!e || e->row < 0 || e->flags == 0) {
        return;
    }
    brain_kbd_set_contact(s, e->row, e->col, down);
    brain_kbd_apply_mods(s, e, down);
}

/*
 * Is the guest scanning in the Brainux/Linux direction (driving
 * the columns, reading the rows)?  That driver folds only chords
 * whose contacts land in the same scan pass, so chords must stay
 * atomic while it is in charge.  The WinCE driver (drives the
 * rows, reads the columns) is the opposite: it folds the symbols
 * layer only for a base key whose press edge arrives in a later
 * scan than the 記号 contact.
 */
static bool brain_kbd_columns_driven(BrainKbdState *s)
{
    return (s->col2_doe & 0x3f0000) != 0 || (s->col4_doe & 0x100) != 0;
}

/* Release the contacts a latched key-down closed. */
static void brain_kbd_release_entry(BrainKbdState *s, unsigned int key)
{
    const BrainKeyDef *e = &s->active[key];

    if (e->flags == 0) {
        return;
    }
    if (s->chord_base_deferred[key]) {
        /* The base contact never landed: release only the
         * modifiers the key-down applied. */
        s->chord_base_deferred[key] = false;
        brain_kbd_apply_mods(s, e, false);
    } else {
        brain_kbd_apply(s, e, false);
    }
    s->active[key].flags = 0;
    brain_kbd_recalc(s);
}

/* Arm the chord timer for the next pending landing or release. */
static void brain_kbd_chord_schedule(BrainKbdState *s)
{
    uint64_t next = UINT64_MAX, cand;
    unsigned int i;

    if (s->chord_qcount > 0) {
        int key = s->chord_queue[0];

        if (s->chord_qstate[key] == 1) {
            cand = s->chord_down_ns[key] +
                   BRAIN_KBD_CHORD_MIN_MS * SCALE_MS;
            next = MIN(next, cand);
            cand = s->chord_down_ns[key] +
                   BRAIN_KBD_CHORD_FALLBACK_MS * SCALE_MS;
            next = MIN(next, cand);
        } else {
            cand = s->chord_last_land_ns +
                   BRAIN_KBD_CHORD_GAP_MS * SCALE_MS;
            next = MIN(next, cand);
        }
    }
    for (i = 0; i < KEY_CNT; i++) {
        if (s->chord_release_at[i]) {
            next = MIN(next, s->chord_release_at[i]);
        }
    }
    if (next == UINT64_MAX) {
        timer_del(s->chord_timer);
    } else {
        timer_mod(s->chord_timer, next);
    }
}

/* Land the queue head's deferred contact if it is due. */
static bool brain_kbd_chord_land_head(BrainKbdState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    int key;
    const BrainKeyDef *e;

    if (s->chord_qcount == 0) {
        return false;
    }
    key = s->chord_queue[0];
    e = &s->active[key];
    if (s->chord_qstate[key] == 1) {
        uint64_t held = now - s->chord_down_ns[key];

        if (!((s->chord_scan_ticks >= BRAIN_KBD_CHORD_SCAN_TICKS &&
               held >= BRAIN_KBD_CHORD_MIN_MS * SCALE_MS) ||
              held >= BRAIN_KBD_CHORD_FALLBACK_MS * SCALE_MS)) {
            return false;
        }
        brain_kbd_set_contact(s, e->row, e->col, true);
        s->chord_base_deferred[key] = false;
    } else {
        if (now < s->chord_last_land_ns +
                  BRAIN_KBD_CHORD_GAP_MS * SCALE_MS) {
            return false;
        }
        brain_kbd_apply(s, e, true);
    }
    memmove(&s->chord_queue[0], &s->chord_queue[1],
            sizeof(s->chord_queue[0]) * (s->chord_qcount - 1));
    s->chord_qcount--;
    s->chord_qstate[key] = 0;
    s->chord_last_land_ns = now;
    s->chord_scan_ticks = 0;
    s->chord_last_out = s->row_out;
    s->chord_last_doe = s->row_doe;
    if (s->chord_release_deferred[key]) {
        s->chord_release_deferred[key] = false;
        s->chord_release_at[key] = now +
            BRAIN_KBD_CHORD_RELEASE_MS * SCALE_MS;
    }
    brain_kbd_recalc(s);
    return true;
}

/* Land every due queue head and perform every due deferred
 * release, then re-arm the timer.  Called from the chord timer
 * and from the guest's scan hook. */
static void brain_kbd_chord_process(BrainKbdState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    unsigned int i;

    while (brain_kbd_chord_land_head(s)) {
    }
    for (i = 0; i < KEY_CNT; i++) {
        if (s->chord_release_at[i] && s->chord_release_at[i] <= now) {
            s->chord_release_at[i] = 0;
            brain_kbd_release_entry(s, i);
        }
    }
    brain_kbd_chord_schedule(s);
}

static void brain_kbd_chord_cb(void *opaque)
{
    brain_kbd_chord_process(opaque);
}

static void brain_kbd_event(DeviceState *dev, QemuConsole *src,
                             QemuInputEvent *evt)
{
    BrainKbdState *s = BRAIN_KBD(dev);
    unsigned int key;
    bool down;
    const BrainKeyDef *e;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }
    key = evt->key.key;
    down = evt->key.down;

    if (key >= KEY_CNT) {
        return;
    }

    /*
     * Ctrl only selects the dictionary-key combo table, and Shift
     * drives the Shift contact through brain_kbd_sync_shift() (and
     * selects the "shifted" table at key-down).  Both are tracked
     * per key: frontends send autorepeat down events for a held
     * modifier, and counting those would inflate the state so that
     * the single matching key-up never clears it (Ctrl is the
     * dangerous one: every later key would become a combo key).
     */
    if (key == KEY_LEFTCTRL || key == KEY_RIGHTCTRL) {
        bool *slot = (key == KEY_LEFTCTRL) ? &s->lctrl_down
                                           : &s->rctrl_down;
        if (*slot != down) {
            *slot = down;
            s->ctrl_held = (s->lctrl_down ? 1 : 0) +
                           (s->rctrl_down ? 1 : 0);
        }
        return;
    }
    if (key == KEY_LEFTSHIFT || key == KEY_RIGHTSHIFT) {
        bool *slot = (key == KEY_LEFTSHIFT) ? &s->lshift_down
                                            : &s->rshift_down;
        if (*slot != down) {
            *slot = down;
            brain_kbd_sync_shift(s);
            brain_kbd_recalc(s);
        }
        return;
    }

    if (!down) {
        /*
         * Release exactly the contacts the matching key-down closed.
         * Frontends do not guarantee that modifiers are released
         * last: if the user lets go of Ctrl before the letter key,
         * looking the letter up again here (with Ctrl now up) would
         * release the plain letter contact -- which was never
         * closed -- and leave the combo's contact (e.g. 戻る) closed
         * forever.  A stuck contact keeps its column low, so the
         * guest sees the key held down indefinitely.
         */
        if (s->active[key].flags == 0) {
            return;
        }
        if (s->chord_qstate[key] != 0) {
            /*
             * The key is still queued behind its chord stagger:
             * remember the release; it takes effect shortly after
             * the base contact lands, so the guest sees the whole
             * chord even for a tap shorter than the stagger.
             */
            s->chord_release_deferred[key] = true;
            return;
        }
        if (s->chord_release_at[key]) {
            return;                 /* release already deferred */
        }
        brain_kbd_release_entry(s, key);
        return;
    }

    if (s->active[key].flags != 0) {
        return;                     /* autorepeat: already latched */
    }
    e = &s->direct_resolved[key];
    if (s->ctrl_held && s->layout->combo[key].flags != 0) {
        e = &s->layout->combo[key];
    } else if ((s->lshift_down || s->rshift_down) && s->shifted_tab &&
               s->shifted_tab[key].flags != 0) {
        e = &s->shifted_tab[key];
    }
    if (e->flags == 0) {
        return;
    }
    s->active[key] = *e;
    if ((e->flags & BRAIN_KEY_SYMBOL) && !brain_kbd_columns_driven(s)) {
        /*
         * WinCE scan direction: land the modifier contacts now and
         * queue the base contact behind the chord stagger, the way
         * a typist presses 記号 a moment before the base key.  If
         * the contacts all landed in one scan the guest would
         * report two independent key presses instead of the chord
         * (see the file header).  In the Brainux scan direction
         * the chord stays atomic, as that driver requires.
         */
        brain_kbd_apply_mods(s, e, true);
        s->chord_base_deferred[key] = true;
        s->chord_qstate[key] = 1;
        s->chord_queue[s->chord_qcount++] = key;
        s->chord_down_ns[key] = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        if (s->chord_qcount == 1) {
            s->chord_scan_ticks = 0;
            s->chord_last_out = s->row_out;
            s->chord_last_doe = s->row_doe;
        }
        brain_kbd_chord_schedule(s);
    } else if (s->chord_qcount > 0) {
        /*
         * A chord is still waiting for its stagger: queue this
         * key behind it untouched, so the guest sees the keys in
         * host order.
         */
        s->chord_qstate[key] = 2;
        s->chord_queue[s->chord_qcount++] = key;
        s->chord_down_ns[key] = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        brain_kbd_chord_schedule(s);
    } else {
        brain_kbd_apply(s, e, true);
    }
    brain_kbd_recalc(s);
}

static void brain_kbd_sync(DeviceState *dev)
{
    BrainKbdState *s = BRAIN_KBD(dev);

    brain_kbd_recalc(s);
}

static const QemuInputHandler brain_kbd_handler = {
    .name  = "brain-matrix-kbd",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = brain_kbd_event,
    .sync  = brain_kbd_sync,
};

/* ------------------------------------------------------------------ */
/* device boilerplate                                                  */
/* ------------------------------------------------------------------ */

static void brain_kbd_realize(DeviceState *dev, Error **errp)
{
    BrainKbdState *s = BRAIN_KBD(dev);
    QemuInputHandlerState *hs;

    if (!s->layout_name) {
        s->layout_name = g_strdup("pw-sh6");
    }
    s->layout = brain_kbd_find_layout(s->layout_name);
    if (!s->layout) {
        error_setg(errp, "brain-kbd: unknown keyboard layout '%s'",
                   s->layout_name);
        return;
    }

    /*
     * Resolve the host keyboard layout: host keycodes are
     * positional, and the glyph printed on a positional keycode
     * differs between US and JIS keyboards.  Default to JIS (the
     * Brain is a Japanese-market device); US keyboard users set
     * -global brain-kbd.host-layout=us.
     */
    if (!s->host_layout_name) {
        s->host_layout_name = g_strdup("jis");
    }
    if (!strcmp(s->host_layout_name, "jis")) {
        s->host_jis = true;
    } else if (!strcmp(s->host_layout_name, "us")) {
        s->host_jis = false;
    } else {
        error_setg(errp, "brain-kbd: unknown host-layout '%s' "
                   "(expected \"us\" or \"jis\")", s->host_layout_name);
        return;
    }
    memcpy(s->direct_resolved, s->layout->direct,
           sizeof(s->direct_resolved));
    if (s->host_jis && s->layout->jis_direct) {
        for (int i = 0; i < KEY_CNT; i++) {
            if (s->layout->jis_direct[i].flags != 0) {
                s->direct_resolved[i] = s->layout->jis_direct[i];
            }
        }
    }
    s->shifted_tab = s->host_jis ? s->layout->jis_shifted
                                 : s->layout->us_shifted;
    if (!s->pinctrl) {
        error_setg(errp, "brain-kbd: 'pinctrl' link not set");
        return;
    }

    /* snapshot the current row drive state, then observe changes.
     * The columns are pulled up on the board: drive them high now so
     * the driver sees idle-high columns (and no phantom interrupt)
     * when it arms the column IRQs during its init. */
    s->row_out = mxs_pinctrl_get_output(s->pinctrl, 4);
    s->row_doe = mxs_pinctrl_get_doe(s->pinctrl, 4);
    s->col4_out = s->row_out;
    s->col4_doe = s->row_doe;
    s->col2_out = mxs_pinctrl_get_output(s->pinctrl, 2);
    s->col2_doe = mxs_pinctrl_get_doe(s->pinctrl, 2);
    mxs_pinctrl_set_input(s->pinctrl, 2, 0x3f0000, 0x3f0000);
    mxs_pinctrl_set_input(s->pinctrl, 4, 0x100, 0x100);
    mxs_pinctrl_set_input(s->pinctrl, 4, 0xff, 0xff);
    mxs_pinctrl_set_output_hook(s->pinctrl, brain_kbd_col_notify, s);

    s->chord_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                  brain_kbd_chord_cb, s);
    s->chord_qcount = 0;

    hs = qemu_input_handler_register(dev, &brain_kbd_handler);
    (void)hs;    /* handler lives as long as the device */
}

static void brain_kbd_reset(DeviceState *dev)
{
    BrainKbdState *s = BRAIN_KBD(dev);

    s->pressed = 0;
    s->ctrl_held = 0;
    s->lctrl_down = s->rctrl_down = false;
    s->lshift_down = s->rshift_down = false;
    s->shift_forced = 0;
    s->shift_suppressed = 0;
    s->shift_applied = false;
    memset(s->active, 0, sizeof(s->active));
    memset(s->contact_refs, 0, sizeof(s->contact_refs));
    s->chord_qcount = 0;
    s->chord_scan_ticks = 0;
    memset(s->chord_qstate, 0, sizeof(s->chord_qstate));
    memset(s->chord_release_deferred, 0,
           sizeof(s->chord_release_deferred));
    memset(s->chord_release_at, 0, sizeof(s->chord_release_at));
    memset(s->chord_base_deferred, 0, sizeof(s->chord_base_deferred));
    if (s->chord_timer) {
        timer_del(s->chord_timer);
    }
    brain_kbd_recalc(s);
}

static const Property brain_kbd_properties[] = {
    DEFINE_PROP_STRING("layout", BrainKbdState, layout_name),
    DEFINE_PROP_STRING("host-layout", BrainKbdState, host_layout_name),
    DEFINE_PROP_LINK("pinctrl", BrainKbdState, pinctrl,
                     TYPE_MXS_PINCTRL, DeviceState *),
};

static void brain_kbd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "SHARP Brain GPIO matrix keyboard";
    dc->realize = brain_kbd_realize;
    device_class_set_legacy_reset(dc, brain_kbd_reset);
    device_class_set_props(dc, brain_kbd_properties);
    dc->user_creatable = false;
}

static const TypeInfo brain_kbd_info = {
    .name = TYPE_BRAIN_KBD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BrainKbdState),
    .class_init = brain_kbd_class_init,
};

static void brain_kbd_register(void)
{
    type_register_static(&brain_kbd_info);
}
type_init(brain_kbd_register)
