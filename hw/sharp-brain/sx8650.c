/*
 * Semtech SX8650 resistive touch-screen controller: I2C model.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The SHARP Brain's touch panel is driven by a Semtech SX8650
 * ("Low Voltage Ultra Low Power 4/5-Wire Resistive Touch Screen
 * Controller with 12-bit ADC", datasheet ACS Revision V2.15).  The
 * guest's PDD is sx8650_touchscreen.dll (source path in the image:
 * SRC\EDNA2\MainTouch\SX8650Touch.cpp), which talks to the chip
 * through the CEDDK I2C library on device "I2C1:".
 *
 * Model scope (datasheet behaviour, nothing beyond it):
 *
 *   - I2C slave at 7-bit address 0x48 (A0 pin = GND; 0x49 if
 *     A0 = VDD), on the I2C bus at 0x80058000.
 *   - Register file: Ctrl0 (0x00), Ctrl1 (0x01), Ctrl2 (0x02),
 *     ChanMsk (0x04) are stored R/W; Stat (0x05) is read-only and
 *     computed from the pending CONVIRQ/PENIRQ bits; writing 0xDE
 *     to SoftReset (0x1F) restores the reset values.  A register
 *     write auto-increments the register address; a register read
 *     (command byte 010xxxxx) returns registers from that address
 *     until the Stop condition, after which reads revert to the
 *     channel-read format.
 *   - Host commands (single byte with bit 7 set): SELECT (1000xccc)
 *     and CONVERT (1001xccc) for channel ccc (X=0, Y=1, Z1=2, Z2=3,
 *     AUX=4, SEQ=7 = the ChanMsk set), MANAUTO (0xB0), PENDET
 *     (0xC0) and PENTRG (0xE0).
 *   - Modes: manual (RATE = 0), automatic (RATE > 0, periodic
 *     conversions; a new conversion only starts once the host has
 *     read the previous data), pen-detect (PENIRQ + NIRQ on pen
 *     down) and pen-trigger (pen down converts at once and raises
 *     CONVIRQ).  With CONDIRQ set, a conversion raises its IRQ
 *     only when the pen is down.
 *   - Channel data is read back as two bytes per converted channel
 *     in X, Y, Z1, Z2, AUX order: [0|chan|D11:8], [D7:0].  A
 *     conversion done with no pen down yields the invalid-data
 *     qualifier 0xFFFF per channel.  Reading the channel data
 *     clears CONVIRQ; reading the Stat register clears PENIRQ.
 *   - NIRQ is an open-drain active-low output, asserted while
 *     CONVIRQ or PENIRQ is pending; it is wired to GPIO3_1 (the
 *     brainux imx28-brain.dtsi sx8650 node: interrupts = <1
 *     IRQ_TYPE_EDGE_FALLING> under &gpio3) and driven through the
 *     pinctrl input API, like the matrix keyboard.
 *   - Pen position and pen down/up come from QEMU's absolute
 *     pointer input (left button = pen).  The 0..0x7FFF input
 *     range is scaled linearly onto the 12-bit ADC range; the
 *     guest's calibration data performs the ADC-to-pixel mapping,
 *     exactly as on hardware.  Z1/Z2 report a fixed plausible
 *     pressure pair while the pen is down.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "ui/input.h"
#include "brain.h"

/* registers (5-bit address) */
#define SX8650_REG_CTRL0      0x00
#define SX8650_REG_CTRL1      0x01
#define SX8650_REG_CTRL2      0x02
#define SX8650_REG_CHANMSK    0x04
#define SX8650_REG_STAT       0x05
#define SX8650_REG_SOFTRESET  0x1f

#define SX8650_CTRL0_RATE     0xf0  /* >0: automatic conversions */
#define SX8650_CTRL1_CONDIRQ  0x20  /* IRQ only if pen detected */
#define SX8650_STAT_CONVIRQ   0x80
#define SX8650_STAT_PENIRQ    0x40
#define SX8650_SOFTRESET_KEY  0xde

/* channels */
#define SX8650_CHAN_X         0
#define SX8650_CHAN_Y         1
#define SX8650_CHAN_Z1        2
#define SX8650_CHAN_Z2        3
#define SX8650_CHAN_AUX       4
#define SX8650_CHAN_SEQ       7
#define SX8650_NCHANS         5

/* host commands (bit 7 set) */
#define SX8650_CMD_SELECT     0x80  /* 1000xccc */
#define SX8650_CMD_CONVERT    0x90  /* 1001xccc */
#define SX8650_CMD_MANAUTO    0xb0  /* 1011xxxx */
#define SX8650_CMD_PENDET     0xc0  /* 1100xxxx */
#define SX8650_CMD_PENTRG     0xe0  /* 1110xxxx */

/* fixed pressure pair reported while the pen is down */
#define SX8650_Z1_DOWN        1500
#define SX8650_Z2_DOWN        2500

typedef enum Sx8650Mode {
    SX8650_MODE_MANUAL,
    SX8650_MODE_AUTO,
    SX8650_MODE_PENDET,
    SX8650_MODE_PENTRG,
} Sx8650Mode;

typedef struct Sx8650State {
    I2CSlave parent_obj;

    DeviceState *pinctrl;

    uint8_t ctrl0, ctrl1, ctrl2, chanmsk;
    Sx8650Mode mode;
    bool convirq, penirq;

    /* transaction state */
    bool first_byte;        /* next send() byte is the command byte */
    bool reg_read;          /* recv() serves registers, not channels */
    uint8_t reg_ptr;        /* register address for read/write */
    bool stat_read;         /* this transaction read the Stat reg */
    bool chan_read;         /* this transaction read channel data */

    /* last conversion result: 2 bytes per channel, in read order */
    uint8_t data[SX8650_NCHANS * 2];
    unsigned data_len;
    unsigned data_pos;

    /* pen state, from the QEMU absolute pointer input */
    bool pen_down;
    uint16_t pen_x, pen_y;  /* 12-bit ADC counts */

    QEMUTimer *conv_timer;
} Sx8650State;

OBJECT_DECLARE_SIMPLE_TYPE(Sx8650State, SX8650)

/* automatic-mode conversion rates, indexed by Ctrl0 RATE (cps) */
static const unsigned sx8650_rates[16] = {
    0, 10, 20, 40, 60, 80, 100, 200,
    400, 800, 1200, 1600, 2000, 3000, 4000, 5000,
};

static void sx8650_update_irq(Sx8650State *s)
{
    uint32_t mask = 1u << SX8650_NIRQ_GPIO_PIN;
    uint32_t level = (s->convirq || s->penirq) ? 0 : mask;

    if (s->pinctrl) {
        mxs_pinctrl_set_input(s->pinctrl, SX8650_NIRQ_GPIO_BANK,
                              mask, level);
    }
}

static uint8_t sx8650_reg_read(Sx8650State *s, uint8_t reg)
{
    switch (reg) {
    case SX8650_REG_CTRL0:
        return s->ctrl0;
    case SX8650_REG_CTRL1:
        return s->ctrl1;
    case SX8650_REG_CTRL2:
        return s->ctrl2;
    case SX8650_REG_CHANMSK:
        return s->chanmsk;
    case SX8650_REG_STAT:
        return (s->convirq ? SX8650_STAT_CONVIRQ : 0) |
               (s->penirq ? SX8650_STAT_PENIRQ : 0);
    default:
        return 0;
    }
}

static void sx8650_load_defaults(Sx8650State *s)
{
    s->ctrl0 = 0x00;                /* manual mode */
    s->ctrl1 = SX8650_CTRL1_CONDIRQ;
    s->ctrl2 = 0x00;
    s->chanmsk = 0xc0;              /* X and Y conversions enabled */
    s->mode = SX8650_MODE_MANUAL;
    s->convirq = s->penirq = false;
    s->data_len = s->data_pos = 0;
    if (s->conv_timer) {
        timer_del(s->conv_timer);
    }
    sx8650_update_irq(s);
}

static void sx8650_reg_write(Sx8650State *s, uint8_t reg, uint8_t val)
{
    switch (reg) {
    case SX8650_REG_CTRL0:
        s->ctrl0 = val;
        break;
    case SX8650_REG_CTRL1:
        s->ctrl1 = val;
        break;
    case SX8650_REG_CTRL2:
        s->ctrl2 = val;
        break;
    case SX8650_REG_CHANMSK:
        s->chanmsk = val;
        break;
    case SX8650_REG_SOFTRESET:
        if (val == SX8650_SOFTRESET_KEY) {
            sx8650_load_defaults(s);
        }
        break;
    case SX8650_REG_STAT:           /* read-only */
    default:
        break;
    }
}

/*
 * Run a conversion of the given channel set (bit i = channel i, in
 * X, Y, Z1, Z2, AUX order) and latch the read-back bytes.  With no
 * pen down, every channel is qualified invalid (0xFFFF), as on
 * hardware.
 */
static void sx8650_convert(Sx8650State *s, unsigned chans)
{
    unsigned n = 0;

    for (int ch = 0; ch < SX8650_NCHANS; ch++) {
        uint16_t v;

        if (!(chans & (1u << ch))) {
            continue;
        }
        if (!s->pen_down) {
            s->data[n++] = 0xff;
            s->data[n++] = 0xff;
            continue;
        }
        switch (ch) {
        case SX8650_CHAN_X:
            v = s->pen_x;
            break;
        case SX8650_CHAN_Y:
            v = s->pen_y;
            break;
        case SX8650_CHAN_Z1:
            v = SX8650_Z1_DOWN;
            break;
        case SX8650_CHAN_Z2:
            v = SX8650_Z2_DOWN;
            break;
        default:                    /* AUX: nothing connected */
            v = 0;
            break;
        }
        s->data[n++] = (ch << 4) | ((v >> 8) & 0x0f);
        s->data[n++] = v & 0xff;
    }
    s->data_len = n;
    s->data_pos = 0;

    if (!(s->ctrl1 & SX8650_CTRL1_CONDIRQ) || s->pen_down) {
        s->convirq = true;
    }
    sx8650_update_irq(s);
}

static unsigned sx8650_chanmsk_chans(Sx8650State *s)
{
    /* ChanMsk bits 7..3 enable X, Y, Z1, Z2, AUX conversions,
     * in that order (bit 7 = X, bit 3 = AUX) */
    unsigned chans = 0;

    for (int ch = 0; ch < SX8650_NCHANS; ch++) {
        if (s->chanmsk & (1u << (7 - ch))) {
            chans |= 1u << ch;
        }
    }
    return chans;
}

static void sx8650_auto_timer(void *opaque)
{
    Sx8650State *s = opaque;
    unsigned rate = sx8650_rates[(s->ctrl0 & SX8650_CTRL0_RATE) >> 4];

    if (s->mode != SX8650_MODE_AUTO || rate == 0) {
        return;
    }
    /* the chip does not start a new conversion before the host
     * has read the previous channel data */
    if (!s->convirq) {
        sx8650_convert(s, sx8650_chanmsk_chans(s));
    }
    timer_mod(s->conv_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / rate);
}

static void sx8650_auto_start(Sx8650State *s)
{
    unsigned rate = sx8650_rates[(s->ctrl0 & SX8650_CTRL0_RATE) >> 4];

    if (rate == 0) {
        s->mode = SX8650_MODE_MANUAL;
        return;
    }
    s->mode = SX8650_MODE_AUTO;
    timer_mod(s->conv_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / rate);
}

static void sx8650_command(Sx8650State *s, uint8_t cmd)
{
    if ((cmd & 0xf0) == SX8650_CMD_SELECT) {
        /* bias the selected channel: nothing observable */
    } else if ((cmd & 0xf0) == SX8650_CMD_CONVERT) {
        unsigned chan = cmd & 0x07;

        if (chan == SX8650_CHAN_SEQ) {
            sx8650_convert(s, sx8650_chanmsk_chans(s));
        } else if (chan < SX8650_NCHANS) {
            sx8650_convert(s, 1u << chan);
        }
    } else if ((cmd & 0xf0) == SX8650_CMD_MANAUTO) {
        if (s->ctrl0 & SX8650_CTRL0_RATE) {
            sx8650_auto_start(s);
        } else {
            s->mode = SX8650_MODE_MANUAL;
            timer_del(s->conv_timer);
        }
    } else if ((cmd & 0xf0) == SX8650_CMD_PENDET) {
        s->mode = SX8650_MODE_PENDET;
        timer_del(s->conv_timer);
        if (s->pen_down) {
            s->penirq = true;
            sx8650_update_irq(s);
        }
    } else if ((cmd & 0xf0) == SX8650_CMD_PENTRG) {
        s->mode = SX8650_MODE_PENTRG;
        timer_del(s->conv_timer);
        if (s->pen_down) {
            sx8650_convert(s, sx8650_chanmsk_chans(s));
        }
    }
}

static int sx8650_event(I2CSlave *i2c, enum i2c_event event)
{
    Sx8650State *s = SX8650(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->first_byte = true;
        break;
    case I2C_START_RECV:
        s->stat_read = false;
        s->chan_read = false;
        break;
    case I2C_FINISH:
        /* Stop condition: IRQ clear conditions take effect, and
         * reads revert to the channel-read format */
        if (s->stat_read && s->penirq) {
            s->penirq = false;
            sx8650_update_irq(s);
        }
        if (s->chan_read && s->convirq) {
            s->convirq = false;
            sx8650_update_irq(s);
        }
        s->stat_read = s->chan_read = false;
        s->reg_read = false;
        break;
    case I2C_NACK:
    default:
        break;
    }
    return 0;
}

static int sx8650_send(I2CSlave *i2c, uint8_t data)
{
    Sx8650State *s = SX8650(i2c);

    if (s->first_byte) {
        s->first_byte = false;
        if (data & 0x80) {
            /* host command */
            sx8650_command(s, data);
        } else if ((data & 0xe0) == 0x40) {
            /* register read: set the read pointer */
            s->reg_read = true;
            s->reg_ptr = data & 0x1f;
        } else {
            /* register write at RA, auto-incrementing */
            s->reg_read = false;
            s->reg_ptr = data & 0x1f;
        }
        return 0;   /* ACK */
    }

    sx8650_reg_write(s, s->reg_ptr, data);
    s->reg_ptr = (s->reg_ptr + 1) & 0x1f;
    return 0;       /* ACK */
}

static uint8_t sx8650_recv(I2CSlave *i2c)
{
    Sx8650State *s = SX8650(i2c);

    if (s->reg_read) {
        uint8_t v = sx8650_reg_read(s, s->reg_ptr);

        if (s->reg_ptr == SX8650_REG_STAT) {
            s->stat_read = true;
        }
        s->reg_ptr = (s->reg_ptr + 1) & 0x1f;
        return v;
    }

    s->chan_read = true;
    if (s->data_pos < s->data_len) {
        return s->data[s->data_pos++];
    }
    return 0xff;
}

static void sx8650_pen_update(Sx8650State *s)
{
    if (!s->pen_down) {
        return;
    }
    switch (s->mode) {
    case SX8650_MODE_PENDET:
        s->penirq = true;
        sx8650_update_irq(s);
        break;
    case SX8650_MODE_PENTRG:
        sx8650_convert(s, sx8650_chanmsk_chans(s));
        break;
    case SX8650_MODE_MANUAL:
    case SX8650_MODE_AUTO:
    default:
        break;
    }
}

static void sx8650_input_event(DeviceState *dev, QemuConsole *src,
                               QemuInputEvent *evt)
{
    Sx8650State *s = SX8650(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->pen_x = (evt->abs.value * 0xfff) / 0x7fff;
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->pen_y = (evt->abs.value * 0xfff) / 0x7fff;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT &&
            evt->btn.down != s->pen_down) {
            s->pen_down = evt->btn.down;
            sx8650_pen_update(s);
        }
        break;
    default:
        break;
    }
}

static const QemuInputHandler sx8650_input_handler = {
    .name  = "sx8650-touchscreen",
    .mask  = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = sx8650_input_event,
};

static void sx8650_realize(DeviceState *dev, Error **errp)
{
    Sx8650State *s = SX8650(dev);
    QemuInputHandlerState *hs;

    if (!s->pinctrl) {
        error_setg(errp, "sx8650: 'pinctrl' link not set");
        return;
    }
    s->conv_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 sx8650_auto_timer, s);
    /* NIRQ idles high (no interrupt pending) */
    sx8650_update_irq(s);
    hs = qemu_input_handler_register(dev, &sx8650_input_handler);
    (void)hs;    /* handler lives as long as the device */
}

static void sx8650_reset(DeviceState *dev)
{
    Sx8650State *s = SX8650(dev);

    sx8650_load_defaults(s);
    s->first_byte = true;
    s->reg_read = false;
    s->reg_ptr = 0;
    s->stat_read = s->chan_read = false;
}

static const Property sx8650_properties[] = {
    DEFINE_PROP_LINK("pinctrl", Sx8650State, pinctrl,
                     TYPE_MXS_PINCTRL, DeviceState *),
};

static void sx8650_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->desc = "Semtech SX8650 resistive touch-screen controller";
    dc->realize = sx8650_realize;
    device_class_set_legacy_reset(dc, sx8650_reset);
    device_class_set_props(dc, sx8650_properties);
    dc->user_creatable = false;
    sc->event = sx8650_event;
    sc->recv = sx8650_recv;
    sc->send = sx8650_send;
}

static const TypeInfo sx8650_info = {
    .name = TYPE_SX8650,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Sx8650State),
    .class_init = sx8650_class_init,
};

static void sx8650_register(void)
{
    type_register_static(&sx8650_info);
}
type_init(sx8650_register)
