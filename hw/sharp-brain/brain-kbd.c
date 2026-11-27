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
 *   - Digits only exist as the blue "symbols" legend of the QWERTY
 *     keys (記号 modifier + key); pressing them on the host holds
 *     the symbols contact plus the base key contact, which is
 *     exactly what typing them on the real device does.  (On the
 *     guest side the folding is done in software: keybd_EDNA2
 *     reports the 記号 key as F12, which is not a modifier in the
 *     layout's ToUnicode tables -- those only know Shift/Ctrl/Alt
 *     -- and SHARP's own numeric fields, e.g. the setup wizard's
 *     date field and the owner-info PIN field, read the 記号 state
 *     themselves and interpret the letter as its blue digit.)
 *     A held host Shift must not turn the digit chord into a
 *     three-contact chord, so digit entries carry BRAIN_KEY_NOSHIFT:
 *     Shift+1 on the host is the digit chord, never a "!" chord.
 *   - Punctuation has no keys and no legends on the Brain at all,
 *     and no chord of matrix contacts can produce it in the guest:
 *     the only VKs the layout tables map to punctuation glyphs
 *     (the OEM and digit VKs of a PC keyboard) have no physical
 *     keys on this device, and 記号 is not a ToUnicode modifier
 *     (see above).  Host keys whose glyph would need a symbols
 *     chord other than the digits are therefore deliberately left
 *     unassigned -- pressing them does nothing, exactly like the
 *     F-keys.  The one punctuation glyph the device does type is
 *     '_', the shifted form of its only punctuation key, Minus
 *     (the guest layout maps VK_OEM_MINUS to '-'/'_'), and it is
 *     reached the way the device reaches it: Shift+Minus.
 *   - Host keycodes are positional, but with the punctuation chords
 *     gone the only positional differences between US and JIS hosts
 *     are gone too: both layouts resolve identically.  The
 *     "host-layout" property ("jis" by default -- the Brain is a
 *     Japanese-market device -- or "us") is still accepted for
 *     compatibility.  Keys that only exist on JIS keyboards
 *     (Henkan, Muhenkan, Zenkakuhankaku, KatakanaHiragana) and the
 *     JIS Ro key (Shift+Minus, '_') keep their explicit assignments
 *     in both layouts.
 *   - Brain-specific dictionary keys that have no PC equivalent
 *     (History, My辞書, 一覧, 国語, 音声, 英和, 記号,
 *     marker, search) are reached through Ctrl+letter combinations.
 *     戻る also has a combo (Ctrl+B) for keyboards without Esc.
 *     Primary assignments only use keys every PC keyboard has
 *     (letters, Ctrl, Shift, Esc, Enter, Space, Backspace, arrows):
 *     keys that compact keyboards lack or hide behind Fn (Home,
 *     Insert, PageUp/PageDown, the numpad, JIS-only keys) are kept
 *     as aliases only -- Home is an alias for Ctrl+O, PageUp and
 *     PageDown are aliases for Ctrl+P (previous page) and Ctrl+N
 *     (next page), Insert for Ctrl+H, Delete for Backspace.
 *   - The mapping a key-down applies is latched per host key, so a
 *     key-up always releases the same contacts even if Ctrl was
 *     released first; contacts are refcounted because several host
 *     keys can hold one contact (both Shifts, the symbols layer).
 *     Ctrl/Shift state is tracked per key so frontend autorepeat
 *     cannot inflate it.
 *   - Chord timing: whether a symbols chord folds into its glyph
 *     is decided on the guest side, past the matrix: the driver
 *     reports the 記号 and base contacts as a pair of key events
 *     at every stagger tried (measured 0 ms -- both contacts in
 *     one scan bitmap -- through 800 ms).  Fields that convert by
 *     key state (the setup wizard's date field, the owner-info
 *     PIN field, standard CE controls) fold the pair; the shell's
 *     own search fields treat a bare 記号 press as a first-class
 *     key -- it inserts the "~" blank-word wildcard, and "?" on a
 *     second press -- and commit it as soon as its event is
 *     processed, so in those fields a chord surfaces as "~" plus
 *     the base letter at any stagger (the shell's help text and
 *     its software keyboard, which carries dedicated "~" and "?"
 *     keys and no digits, present 記号 exactly that way).  The
 *     queue below therefore only has to reproduce a human chord:
 *     while the guest scans in the WinCE direction, a symbols
 *     chord's base contact is queued until the guest has swept
 *     the matrix about twice with the modifiers down (a 90 ms
 *     fallback covers a guest that is not sweeping), so the
 *     driver sees the modifier latched before the base edge, as
 *     with a typist's fingers; keys pressed meanwhile queue
 *     behind it in host order, and a release that arrives early
 *     takes effect just after the base lands, so the chord always
 *     completes whole.  The Brainux driver folds only same-scan
 *     chords, so while the guest drives the columns (its scan
 *     direction) chords stay atomic.
 *   - Release timing in the Brainux direction: that driver polls
 *     the matrix, so it only ever sees a press whose contact is
 *     still closed when its scan sweeps the key's column.  A host
 *     key-up arrives on wall time, but the guest's scan advances
 *     only when its vCPU is scheduled, so under host load a short
 *     press can open again before any sweep reaches it and the
 *     keystroke vanishes (at the Brainux login dialog this reads
 *     as "authentication error" for a correctly typed password).
 *     While the guest drives the columns, a release is therefore
 *     held back until the guest has driven the key's column low
 *     with the contact closed and moved on -- the scan provably
 *     sampled the rows in between -- and, because the guest's
 *     poll debounce only accepts a key that is still present at
 *     the next sweep, until a later sweep has witnessed it again;
 *     capped at 2000 ms of wall
 *     time for a guest that stops scanning (the Brainux guest
 *     can go longer than a second without sweeping while it is
 *     busy starting a session).  The held-back
 *     release lands as soon as the second witnessing sweep moves
 *     on, so the guest-visible hold stays short however
 *     long the press waited to be seen; the cap only bounds how
 *     long an unseen press may wait.  A press that arrives while
 *     an earlier one is still unwitnessed queues behind it, so a
 *     slow sweep cannot report a burst of presses in column
 *     order instead of the order they were typed in.
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
     * Digits: the blue "symbols" legends.  記号 + base key, exactly
     * like typing them on the real device.  NOSHIFT keeps the chord
     * to the two contacts a typist closes: a host Shift held at the
     * same time must not add the Shift contact and turn the key
     * into a three-contact chord for a glyph the device does not
     * have (! @ # $ % ^ & * ( )).
     *
     * Every other punctuation key of a PC keyboard is deliberately
     * absent here: the Brain has no such keys or legends, and no
     * contact chord produces those glyphs in the guest (see the
     * file header).  Unassigned host keys do nothing, like the
     * F-keys.
     */
    [KEY_1]                    = { 3, 5, S | N }, /* 記号+Q = 1 */
    [KEY_2]                    = { 5, 5, S | N }, /* 記号+W = 2 */
    [KEY_3]                    = { 1, 6, S | N }, /* 記号+E = 3 */
    [KEY_4]                    = { 6, 2, S | N }, /* 記号+R = 4 */
    [KEY_5]                    = { 4, 2, S | N }, /* 記号+T = 5 */
    [KEY_6]                    = { 0, 2, S | N }, /* 記号+Y = 6 */
    [KEY_7]                    = { 2, 2, S | N }, /* 記号+U = 7 */
    [KEY_8]                    = { 3, 1, S | N }, /* 記号+I = 8 */
    [KEY_9]                    = { 5, 1, S | N }, /* 記号+O = 9 */
    [KEY_0]                    = { 5, 0, S | N }, /* 記号+P = 0 */
    [KEY_KP1]                  = { 3, 5, S | N },
    [KEY_KP2]                  = { 5, 5, S | N },
    [KEY_KP3]                  = { 1, 6, S | N },
    [KEY_KP4]                  = { 6, 2, S | N },
    [KEY_KP5]                  = { 4, 2, S | N },
    [KEY_KP6]                  = { 0, 2, S | N },
    [KEY_KP7]                  = { 2, 2, S | N },
    [KEY_KP8]                  = { 3, 1, S | N },
    [KEY_KP9]                  = { 5, 1, S | N },
    [KEY_KP0]                  = { 5, 0, S | N },
    [KEY_KPMINUS]              = { 6, 0, D },   /* the Minus key */

    /*
     * Keys that exist only on JIS host keyboards.  These keycodes
     * never arrive from a US keyboard, so the assignments are the
     * same in both host layouts.
     */
    [KEY_RO]                   = { 6, 0, D | H }, /* _: Shift+Minus
                                                 * (the Brain's shift
                                                 * legend for Minus) */
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
 * There are no JIS (or US) override tables: the positional
 * differences between host layouts were all punctuation glyphs,
 * and those chords are gone (see pwsh6_direct).  What remains --
 * letters, digits, the direct keys and the JIS-only keys in the
 * base table -- resolves identically for both host layouts, and a
 * held host Shift is handled by the NOSHIFT flags on the digit
 * entries and by the plain Shift contact for letters and Minus.
 */

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
    [KEY_O]                    = { 3, 6, D },   /* Ctrl+O  -> ホーム
                                                 * (hOme; the Home key
                                                 * itself is only an
                                                 * alias -- compact
                                                 * keyboards lack it) */
    [KEY_P]                    = { 1, 4, D },   /* Ctrl+P  -> ページ送り
                                                 * 前 (Previous page;
                                                 * PageUp key is the
                                                 * alias) */
    [KEY_N]                    = { 3, 4, D },   /* Ctrl+N  -> ページ送り
                                                 * 次 (Next page;
                                                 * PageDown key is
                                                 * the alias) */
};

static const BrainKbdLayout pwsh6_layout = {
    .name        = "pw-sh6",
    .symbol_row  = SYM_ROW,
    .symbol_col  = SYM_COL,
    .shift_row   = 4,
    .shift_col   = 4,
    .direct      = pwsh6_direct,
    .combo       = pwsh6_combo,
    .jis_direct  = NULL,    /* no per-layout overrides: see the note
                             * above the combo table */
    .jis_shifted = NULL,
    .us_shifted  = NULL,    /* shifted host keys follow from the
                             * plain entry plus the passed-through
                             * host Shift contact (letters, Minus),
                             * or its NOSHIFT suppression (digits) */
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
 * brain_kbd_event()): a chord's base contact is held back until
 * the guest has swept the matrix about twice with the modifiers
 * down (counted from its row-drive activity, ~14 transitions
 * per sweep), with a short wall-clock fallback for a guest that
 * is not scanning.  That reproduces a human chord -- modifier
 * latched, then the base edge -- at human scale (the earlier
 * 200/300 ms constants made every digit cost a third of a
 * second).  Whether the guest then folds the pair is the
 * field's business, not the timing's (see the file header).
 * Queued keys land in press order, BRAIN_KBD_CHORD_GAP_MS
 * apart, and a key whose host release arrived while it was
 * queued is released BRAIN_KBD_CHORD_RELEASE_MS after its base
 * lands, so even a tap faster than the latch time completes as
 * a whole chord.
 */
#define BRAIN_KBD_CHORD_SCAN_TICKS 28
#define BRAIN_KBD_CHORD_MIN_MS 0
#define BRAIN_KBD_CHORD_FALLBACK_MS 90
#define BRAIN_KBD_CHORD_GAP_MS 40
#define BRAIN_KBD_CHORD_RELEASE_MS 40

static void brain_kbd_chord_process(BrainKbdState *s);
static bool brain_kbd_columns_driven(BrainKbdState *s);
/* Idle gap between two column-drive bursts that counts as a new
 * sweep of the guest's scan (its bursts are a few ms wide and its
 * poll period is about 100 ms). */
#define BRAIN_KBD_SCAN_SWEEP_GAP_MS 30

/* After a press has been witnessed in a second sweep the guest
 * has confirmed it; the release follows this short grace later
 * (its row read within that pass completes in well under a ms),
 * so the hold it reports stays far below its autorepeat delay. */
#define BRAIN_KBD_SCAN_CONFIRM_GRACE_MS 30

static void brain_kbd_scan_witness(BrainKbdState *s);

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
    if (brain_kbd_columns_driven(s)) {
        /* The guest just re-drove a column bank in its scan
         * direction: update the scan witnesses for held-back
         * releases (see brain_kbd_scan_witness()).  The guest
         * scans in bursts a few ms wide separated by idle gaps
         * of tens of ms; a drive notify after an idle gap starts
         * a new sweep (epoch). */
        uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

        if (now - s->scan_last_notify_ns >
            BRAIN_KBD_SCAN_SWEEP_GAP_MS * SCALE_MS) {
            s->scan_epoch++;
        }
        s->scan_last_notify_ns = now;
        brain_kbd_scan_witness(s);
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

/*
 * Scan-witnessed release in the Brainux/Linux scan direction (see
 * the file header): while the guest drives the columns, a latched
 * key's release is held back until the guest's scan has driven the
 * key's column low with the contact closed and moved on, or until
 * BRAIN_KBD_SCAN_HOLD_MAX_MS of wall time has passed since the
 * press.  The guest reads the rows while it holds a column low,
 * so a witnessed pass means the press was on the rows it sampled.
 * One witnessed sweep is not enough, though: the guest's poll
 * debounce accepts a key only if it is still present at the next
 * sweep, so the release additionally waits for a witness in a
 * later sweep (scan_seen2) before the scan moves past the column.
 * The release lands as soon as that second witnessing sweep moves
 * on, so the hold the guest sees stays short no matter how long
 * the press waited; the cap only keeps a guest that stops
 * scanning from wedging the contact.
 *
 * Presses are serialized the same way: a key that goes down while
 * an earlier press is still unwitnessed waits in a queue and only
 * closes its contact once the earlier press has been seen.  If
 * several presses were closed at once, a slow sweep would report
 * them in column order rather than in the order the user typed
 * them; queued, they land one per sweep, in host order.
 */
#define BRAIN_KBD_SCAN_HOLD_MAX_MS 2000

/* Arm the scan timer for the earliest pending release's cap.
 * Ripe releases (see below) wait for the guest's next drive
 * change instead, so they do not re-arm the timer. */
static void brain_kbd_scan_schedule(BrainKbdState *s)
{
    uint64_t next = UINT64_MAX;
    unsigned int i;

    for (i = 0; i < KEY_CNT; i++) {
        if (s->scan_pending[i] && !s->scan_ripe[i]) {
            next = MIN(next, s->scan_down_ns[i] +
                       BRAIN_KBD_SCAN_HOLD_MAX_MS * SCALE_MS);
            if (s->scan_seen2[i]) {
                next = MIN(next, s->scan_release_at[i]);
            }
        }
        if (s->scan_defer[i] && !s->scan_seen[i] && s->scan_qcount > 0) {
            /* A queued press waits on this key's witness: keep a
             * deadline so the queue drains even if the guest stops
             * scanning while the key is still held. */
            next = MIN(next, s->scan_down_ns[i] +
                       BRAIN_KBD_SCAN_HOLD_MAX_MS * SCALE_MS);
        }
    }
    if (next == UINT64_MAX) {
        timer_del(s->scan_timer);
    } else {
        timer_mod(s->scan_timer, next);
    }
}

/* Perform one key's held-back release. */
static void brain_kbd_scan_release(BrainKbdState *s, unsigned int key)
{
    s->scan_defer[key] = false;
    s->scan_seen[key] = false;
    s->scan_pending[key] = false;
    s->scan_ripe[key] = false;
    s->scan_seen2[key] = false;
    s->scan_release_at[key] = 0;
    s->scan_pass[key] = false;
    brain_kbd_release_entry(s, key);
}

/*
 * Is any latched press still waiting for its first witness?  While
 * one is, later presses queue behind it (see the block comment
 * above), so the guest can only ever meet one unwitnessed press
 * at a time and reports them in host order.
 */
static bool brain_kbd_scan_slot_busy(BrainKbdState *s)
{
    unsigned int i;

    for (i = 0; i < KEY_CNT; i++) {
        if ((s->scan_defer[i] || s->scan_pending[i]) &&
            !s->scan_seen[i]) {
            return true;
        }
    }
    return false;
}

/*
 * Close the queued head's contact once no earlier press is still
 * unwitnessed.  A queued key whose host key-up already arrived
 * lands in the just-released state, so its release follows the
 * same witness rules as any other tap.
 */
static void brain_kbd_scan_land(BrainKbdState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    int key;

    while (s->scan_qcount > 0 && !brain_kbd_scan_slot_busy(s)) {
        key = s->scan_queue[0];
        memmove(&s->scan_queue[0], &s->scan_queue[1],
                sizeof(s->scan_queue[0]) * (s->scan_qcount - 1));
        s->scan_qcount--;
        s->scan_queued[key] = false;
        brain_kbd_apply(s, &s->active[key], true);
        s->scan_seen[key] = false;
        s->scan_seen2[key] = false;
        s->scan_release_at[key] = 0;
        s->scan_pass[key] = false;
        s->scan_ripe[key] = false;
        s->scan_down_ns[key] = now;
        if (s->scan_qrelease[key]) {
            s->scan_qrelease[key] = false;
            s->scan_defer[key] = false;
            s->scan_pending[key] = true;
        } else {
            s->scan_defer[key] = true;
            s->scan_pending[key] = false;
        }
        brain_kbd_recalc(s);
    }
}

/*
 * Observe the guest's column drive (called from the PINCTRL
 * output hook whenever a bank is re-driven in the guest's scan
 * direction, and from the scan timer): record that a latched
 * key's column was driven low, and perform every pending release
 * whose column the scan has since passed, or whose hold cap has
 * expired.
 *
 * A pending release whose cap expires while its column is still
 * low is only marked ripe, not performed: the guest drives a
 * column low before it reads the rows, so opening the contact at
 * the very observation that finds the column low could still
 * beat the guest's read of this pass.  The next drive change
 * proves the guest has executed past that read, and the ripe
 * release lands there.
 */
static void brain_kbd_scan_witness(BrainKbdState *s)
{
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    unsigned int i;

    for (i = 0; i < KEY_CNT; i++) {
        int col;

        if ((!s->scan_defer[i] && !s->scan_pending[i]) ||
            s->active[i].flags == 0) {
            continue;
        }
        col = s->active[i].col;
        if (s->scan_seen2[i] && !brain_kbd_col_low(s, col)) {
            /* The sweep is on this key's column: remember the
             * confirming pass so its end can release the key. */
            s->scan_pass[i] = true;
        }
        if (s->scan_defer[i] && !s->scan_seen[i] &&
            s->scan_qcount > 0 &&
            now >= s->scan_down_ns[i] +
                   BRAIN_KBD_SCAN_HOLD_MAX_MS * SCALE_MS) {
            /* Still unwitnessed a full cap after the press while
             * later presses queue behind it and the guest shows no
             * sign of scanning: give the press up so the queue can
             * drain (the matching key-up becomes a no-op). */
            brain_kbd_scan_release(s, i);
        } else if (s->scan_pending[i] && s->scan_ripe[i]) {
            brain_kbd_scan_release(s, i);
        } else if (s->scan_pending[i] && s->scan_seen2[i] &&
                   s->scan_pass[i] && brain_kbd_col_low(s, col)) {
            /* The confirming pass just ended (the scan moved off
             * the key's column): release now rather than waiting
             * out the grace, so the reported hold stays short. */
            brain_kbd_scan_release(s, i);
        } else if (s->scan_pending[i] && s->scan_seen2[i] &&
                   now >= s->scan_release_at[i]) {
            /* Confirmed by a second sweep and the grace has run
             * out: release regardless of the drive state at this
             * notify (the timer also lands here). */
            brain_kbd_scan_release(s, i);
        } else if (brain_kbd_col_low(s, col)) {
            if (!s->scan_seen[i]) {
                s->scan_seen[i] = true;
                s->scan_seen_epoch[i] = s->scan_epoch;
            } else if (s->scan_epoch > s->scan_seen_epoch[i] &&
                       !s->scan_seen2[i]) {
                /* Witnessed again in a later sweep: the guest's
                 * debounce can now accept the key. */
                s->scan_seen2[i] = true;
                s->scan_seen2_epoch[i] = s->scan_epoch;
                s->scan_release_at[i] = now +
                    BRAIN_KBD_SCAN_CONFIRM_GRACE_MS * SCALE_MS;
            }
            if (s->scan_pending[i] &&
                now >= s->scan_down_ns[i] +
                       BRAIN_KBD_SCAN_HOLD_MAX_MS * SCALE_MS) {
                s->scan_ripe[i] = true;
            }
        } else {
            /* The scan is passing this key's column right now in
             * a later sweep than the first witness: that pass is
             * the second witness (the guest reads the rows during
             * it, and the grace release follows just after). */
            if (s->scan_seen[i] && !s->scan_seen2[i] &&
                s->scan_epoch > s->scan_seen_epoch[i]) {
                s->scan_seen2[i] = true;
                s->scan_seen2_epoch[i] = s->scan_epoch;
                s->scan_release_at[i] = now +
                    BRAIN_KBD_SCAN_CONFIRM_GRACE_MS * SCALE_MS;
            }
            if (s->scan_pending[i] &&
                now >= s->scan_down_ns[i] +
                       BRAIN_KBD_SCAN_HOLD_MAX_MS * SCALE_MS) {
                brain_kbd_scan_release(s, i);
            }
        }
    }
    brain_kbd_scan_land(s);
    brain_kbd_scan_schedule(s);
}

static void brain_kbd_scan_cb(void *opaque)
{
    BrainKbdState *s = opaque;

    if (brain_kbd_columns_driven(s)) {
        brain_kbd_scan_witness(s);
    }
}

/* Arm the chord timer for the next pending landing or release. */
static void brain_kbd_chord_schedule(BrainKbdState *s)
{
    uint64_t next = UINT64_MAX, cand;
    unsigned int i;

    if (s->chord_qcount > 0) {
        int key = s->chord_queue[0];

        if (s->chord_qstate[key] == 1) {
            uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

            /*
             * The scan-tick path re-runs the processor from the
             * GPIO notify hook, so the timer only owes us the two
             * wall-clock deadlines.  Once the minimum has passed
             * without enough witnessed sweeps, the next deadline
             * is the fallback alone: re-arming on the already-past
             * minimum would refire the timer immediately for the
             * whole wait, spinning the main loop.
             */
            cand = s->chord_down_ns[key] +
                   BRAIN_KBD_CHORD_MIN_MS * SCALE_MS;
            if (cand > now) {
                next = MIN(next, cand);
            }
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
        if (s->scan_queued[key]) {
            /*
             * Still queued behind an unwitnessed press: remember
             * the release; it takes effect through the witness
             * rules once the press lands, so the guest sees the
             * whole tap even if it was shorter than the wait.
             */
            s->scan_qrelease[key] = true;
            return;
        }
        if (s->scan_defer[key]) {
            /*
             * Brainux scan direction: hold the release back until
             * the guest's scan has witnessed the press (see
             * brain_kbd_scan_witness()), unless it already has and
             * has moved past the key's column.
             */
            s->scan_defer[key] = false;
            if (s->scan_seen2[key] &&
                qemu_clock_get_ns(QEMU_CLOCK_REALTIME) >=
                    s->scan_release_at[key]) {
                brain_kbd_release_entry(s, key);
                brain_kbd_scan_land(s);
                brain_kbd_scan_schedule(s);
            } else {
                s->scan_pending[key] = true;
                brain_kbd_scan_schedule(s);
            }
            return;
        }
        brain_kbd_release_entry(s, key);
        return;
    }

    if (s->scan_pending[key]) {
        /*
         * The same host key went down again while its previous
         * release was still held back for the guest's scan
         * witness: flush that release now, so this press starts
         * from an open contact.  (Presses that fall inside one
         * guest scan sweep merge into one, exactly as they do on
         * the physical matrix.)
         */
        brain_kbd_scan_release(s, key);
        brain_kbd_scan_land(s);
        brain_kbd_scan_schedule(s);
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
         * a typist presses 記号 a moment before the base key, so
         * the driver sees the modifier latched ahead of the base
         * edge (see the file header).  In the Brainux scan
         * direction the chord stays atomic, as that driver
         * requires.
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
    } else if (brain_kbd_columns_driven(s) &&
               (s->scan_qcount > 0 || brain_kbd_scan_slot_busy(s))) {
        /*
         * Brainux scan direction: an earlier press is still
         * unwitnessed (or others are already queued): queue this
         * press behind them so the guest meets the keys in host
         * order, one unwitnessed press at a time.  The entry is
         * already latched in s->active[]; brain_kbd_scan_land()
         * closes its contacts when its turn comes.
         */
        s->scan_queued[key] = true;
        s->scan_queue[s->scan_qcount++] = key;
        brain_kbd_scan_schedule(s);
    } else {
        brain_kbd_apply(s, e, true);
        if (brain_kbd_columns_driven(s)) {
            /*
             * Brainux scan direction: arm the scan-witnessed
             * release for this press (see brain_kbd_scan_witness()).
             */
            s->scan_defer[key] = true;
            s->scan_seen[key] = false;
            s->scan_seen2[key] = false;
            s->scan_release_at[key] = 0;
            s->scan_pass[key] = false;
            s->scan_pending[key] = false;
            s->scan_ripe[key] = false;
            s->scan_down_ns[key] =
                qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        }
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
     * Resolve the host keyboard layout.  The mapping no longer
     * depends on it (the punctuation chords that made positional
     * keycodes differ between US and JIS hosts are gone), but the
     * property is still validated and accepted so existing command
     * lines keep working.  Default is JIS (the Brain is a
     * Japanese-market device); US keyboard users may set
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
    s->scan_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                 brain_kbd_scan_cb, s);

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
    memset(s->scan_down_ns, 0, sizeof(s->scan_down_ns));
    memset(s->scan_defer, 0, sizeof(s->scan_defer));
    memset(s->scan_seen, 0, sizeof(s->scan_seen));
    memset(s->scan_pending, 0, sizeof(s->scan_pending));
    memset(s->scan_ripe, 0, sizeof(s->scan_ripe));
    s->scan_epoch = 0;
    s->scan_last_notify_ns = 0;
    memset(s->scan_seen_epoch, 0, sizeof(s->scan_seen_epoch));
    memset(s->scan_seen2, 0, sizeof(s->scan_seen2));
    memset(s->scan_seen2_epoch, 0, sizeof(s->scan_seen2_epoch));
    memset(s->scan_release_at, 0, sizeof(s->scan_release_at));
    memset(s->scan_pass, 0, sizeof(s->scan_pass));
    memset(s->scan_queue, 0, sizeof(s->scan_queue));
    s->scan_qcount = 0;
    memset(s->scan_queued, 0, sizeof(s->scan_queued));
    memset(s->scan_qrelease, 0, sizeof(s->scan_qrelease));
    if (s->scan_timer) {
        timer_del(s->scan_timer);
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
