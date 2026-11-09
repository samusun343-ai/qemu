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
 *   rows (inputs, pulled up):   GPIO4_0 .. GPIO4_7
 *   columns (outputs):          GPIO2_16..GPIO2_21, GPIO4_8
 *
 * The keyboard is scanned by driving one column low at a time while
 * the others stay hi-Z; a pressed key pulls its row low.  This model
 * keeps a bitmask of closed contacts and re-evaluates the row input
 * levels whenever the guest re-drives the columns (observed through
 * the PINCTRL output hook) or a host key event changes the contact
 * mask.
 *
 * Host key -> guest mapping policy (keys arrive as linux keycodes):
 *
 *   - PC keys that exist on the Brain (letters, space, return,
 *     backspace, arrows, shift, minus) close the corresponding
 *     matrix contact 1:1.
 *   - Digits and punctuation only exist as the blue "symbols" legend
 *     of the QWERTY keys (記号 modifier + key); pressing them on the
 *     host holds the symbols contact plus the base key contact, which
 *     is exactly what typing them on the real device does.
 *   - Brain-specific dictionary keys that have no PC equivalent
 *     (戻る, History, My辞書, 一覧, 国語, 音声, 英和, 記号, marker,
 *     search) are reached through Ctrl+letter combinations.
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
 * Physical matrix (row, col), from linux-brain imx28-pwsh6.dts:
 *
 *        col0     col1     col2      col3     col4     col5    col6
 *  row0  V        H        Y         戻る     C        D       (Home)
 *  row1  M        K        History   Down     PageUp   F       E
 *  row2  B        J        U         Left     記号     X       -
 *  row3  N        I        My辞書    Up       PageDown Q       Home
 *  row4  Backspace G      T         Right    Shift    A       -
 *  row5  P        O        一覧      Enter    Z        W       国語
 *  row6  Minus    L        R         Space    音声     S       英和
 *  row7  -        -        -         -        -        -       -
 *
 * col0..col5 = GPIO2_16..GPIO2_21, col6 = GPIO4_8.
 */

#define D BRAIN_KEY_DIRECT
#define S BRAIN_KEY_SYMBOL

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
    [KEY_BACKSPACE]            = { 4, 0, D },   /* Backspace */
    [KEY_UP]                   = { 3, 3, D },   /* Up */
    [KEY_DOWN]                 = { 1, 3, D },   /* Down */
    [KEY_LEFT]                 = { 2, 3, D },   /* Left */
    [KEY_RIGHT]                = { 4, 3, D },   /* Right */
    [KEY_LEFTSHIFT]            = { 4, 4, D },   /* Shift */
    [KEY_RIGHTSHIFT]           = { 4, 4, D },
    [KEY_ESC]                  = { 3, 6, D },   /* Home (guest Esc-like key) */
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
    [KEY_K]                    = { 5, 6, D },   /* Ctrl+K  -> 国語 (J-J dict) */
    [KEY_J]                    = { 6, 6, D },   /* Ctrl+J  -> 英和 (E-J dict) */
    [KEY_V]                    = { 6, 4, D },   /* Ctrl+V  -> 音声 (voice) */
    [KEY_S]                    = { SYM_ROW, SYM_COL, D }, /* Ctrl+S -> hold 記号 */
    [KEY_M]                    = { 5, 2, S },   /* Ctrl+M  -> マーカー (記号+一覧) */
    [KEY_F]                    = { 3, 6, S },   /* Ctrl+F  -> 検索 (記号+Home) */
};

static const BrainKbdLayout pwsh6_layout = {
    .name       = "pw-sh6",
    .symbol_row = SYM_ROW,
    .symbol_col = SYM_COL,
    .direct     = pwsh6_direct,
    .combo      = pwsh6_combo,
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

/* column c (0..6) is driven low by the guest? */
static bool brain_kbd_col_low(BrainKbdState *s, int c)
{
    if (c < 6) {
        return !(s->col_out[2] & (1u << (16 + c)));
    }
    return !(s->col_out[4] & (1u << 8));
}

static void brain_kbd_recalc(BrainKbdState *s)
{
    uint32_t rows = 0xff;       /* all rows pulled up */

    for (int r = 0; r < BRAIN_KBD_ROWS; r++) {
        for (int c = 0; c < BRAIN_KBD_COLS; c++) {
            if ((s->pressed & CONTACT_BIT(r, c)) && brain_kbd_col_low(s, c)) {
                rows &= ~(1u << r);
            }
        }
    }

    mxs_pinctrl_set_input(s->pinctrl, 4, 0xff, rows);
}

/* pinctrl output hook: the guest re-drove one of the GPIO banks */
static void brain_kbd_col_notify(void *opaque, int bank, uint32_t out)
{
    BrainKbdState *s = opaque;

    if (bank != 2 && bank != 4) {
        return;
    }
    s->col_out[bank] = out;
    brain_kbd_recalc(s);
}

static void brain_kbd_set_contact(BrainKbdState *s, int r, int c, bool down)
{
    uint64_t m = CONTACT_BIT(r, c);

    if (r < 0 || c < 0 || r >= BRAIN_KBD_ROWS || c >= BRAIN_KBD_COLS) {
        return;
    }
    if (down) {
        s->pressed |= m;
    } else {
        s->pressed &= ~m;
    }
}

/* ------------------------------------------------------------------ */
/* host keyboard input (events arrive as linux keycodes)               */
/* ------------------------------------------------------------------ */

static void brain_kbd_apply(BrainKbdState *s, const BrainKeyDef *e, bool down)
{
    const BrainKbdLayout *l = s->layout;

    if (!e || e->row < 0) {
        return;
    }
    brain_kbd_set_contact(s, e->row, e->col, down);
    if (e->flags & BRAIN_KEY_SYMBOL) {
        brain_kbd_set_contact(s, l->symbol_row, l->symbol_col, down);
    }
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

    /* Ctrl only selects the dictionary-key combo table */
    if (key == KEY_LEFTCTRL || key == KEY_RIGHTCTRL) {
        if (down) {
            s->ctrl_held++;
        } else if (s->ctrl_held) {
            s->ctrl_held--;
        }
        return;
    }
    if (key >= KEY_CNT) {
        return;
    }

    e = &s->layout->direct[key];
    if (s->ctrl_held && s->layout->combo[key].row >= 0) {
        e = &s->layout->combo[key];
    }
    brain_kbd_apply(s, e, down);
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
    if (!s->pinctrl) {
        error_setg(errp, "brain-kbd: 'pinctrl' link not set");
        return;
    }

    /* snapshot the current column drive state, then observe changes */
    for (int b = 0; b < 5; b++) {
        s->col_out[b] = mxs_pinctrl_get_output(s->pinctrl, b);
    }
    mxs_pinctrl_set_output_hook(s->pinctrl, brain_kbd_col_notify, s);

    hs = qemu_input_handler_register(dev, &brain_kbd_handler);
    (void)hs;    /* handler lives as long as the device */
}

static void brain_kbd_reset(DeviceState *dev)
{
    BrainKbdState *s = BRAIN_KBD(dev);

    s->pressed = 0;
    s->ctrl_held = 0;
    brain_kbd_recalc(s);
}

static const Property brain_kbd_properties[] = {
    DEFINE_PROP_STRING("layout", BrainKbdState, layout_name),
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
