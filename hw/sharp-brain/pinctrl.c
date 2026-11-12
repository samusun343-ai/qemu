/*
 * i.MX28 PINCTRL emulation (pin muxing + GPIO banks 0-4).
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * GPIO register offsets (i.MX28, per gpio-mxs.c):
 *   DOUT(b)   0x700 + b*0x10    data out
 *   DIN(b)    0x900 + b*0x10    data in
 *   DOE(b)    0xb00 + b*0x10    output enable
 *   PIN2IRQ   0x1000 + b*0x10
 *   IRQEN     0x1100 + b*0x10
 *   IRQLEV    0x1200 + b*0x10
 *   IRQPOL    0x1300 + b*0x10
 *   IRQSTAT   0x1400 + b*0x10
 *
 * External devices (matrix keyboard, card detect) inject input levels
 * through mxs_pinctrl_set_input(); when IRQEN/PIN2IRQ is armed the
 * corresponding ICOLL line fires.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "brain.h"

#define PINCTRL_BANKS 5

#define REG_DOUT(b)   (0x700 + (b) * 0x10)
#define REG_DIN(b)    (0x900 + (b) * 0x10)
#define REG_DOE(b)    (0xb00 + (b) * 0x10)
#define REG_PIN2IRQ(b) (0x1000 + (b) * 0x10)
#define REG_IRQEN(b)  (0x1100 + (b) * 0x10)
#define REG_IRQLEV(b) (0x1200 + (b) * 0x10)
#define REG_IRQPOL(b) (0x1300 + (b) * 0x10)
#define REG_IRQSTAT(b) (0x1400 + (b) * 0x10)


OBJECT_DECLARE_SIMPLE_TYPE(MxsPinctrlState, MXS_PINCTRL)

typedef struct MxsPinctrlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[PINCTRL_BANKS];

    uint32_t mux[0x700 / 16];      /* mux/drive/pull register file */
    uint32_t dout[PINCTRL_BANKS];
    uint32_t ext_input[PINCTRL_BANKS];   /* externally driven DIN bits */
    uint32_t doe[PINCTRL_BANKS];
    uint32_t pin2irq[PINCTRL_BANKS];
    uint32_t irqen[PINCTRL_BANKS];
    uint32_t irqlev[PINCTRL_BANKS];
    uint32_t irqpol[PINCTRL_BANKS];
    uint32_t irqstat[PINCTRL_BANKS];
    uint32_t prev_din[PINCTRL_BANKS];   /* for edge detection */

    /* GPIO output observer (matrix keyboard) */
    MxsGpioOutNotify *out_hook;
    void *out_opaque;
} MxsPinctrlState;

static uint32_t pinctrl_din(MxsPinctrlState *s, int b)
{
    /* a pin reads back its driven output when configured as output,
     * and the externally driven level (pull-up, key matrix, strap)
     * only when configured as an input */
    return (s->dout[b] & s->doe[b]) | (s->ext_input[b] & ~s->doe[b]);
}

static void pinctrl_update_irq(MxsPinctrlState *s, int b)
{
    uint32_t din = pinctrl_din(s, b);
    uint32_t armed = s->pin2irq[b] & s->irqen[b];
    uint32_t fire = 0;

    /*
     * i.MX28 GPIO IRQ semantics (linux gpio-mxs.c):
     *   IRQLEV=0 (edge): IRQPOL=0 -> falling edge, IRQPOL=1 -> rising edge
     *   IRQLEV=1 (level): IRQPOL=0 -> low level, IRQPOL=1 -> high level
     */
    uint32_t changed = din ^ s->prev_din[b];
    uint32_t rising = changed & din;
    uint32_t falling = changed & ~din;

    for (int bit = 0; bit < 32; bit++) {
        uint32_t m = 1u << bit;
        if (!(armed & m)) {
            continue;
        }
        bool level = !!(din & m);
        bool pol = !!(s->irqpol[b] & m);
        bool is_level = !!(s->irqlev[b] & m);
        if (is_level) {
            /* level-sensitive status follows the pin level: it must
             * clear when the level goes away, or the IRQ would stay
             * asserted forever after the first low (the keyboard
             * driver relies on this to deassert after a key release) */
            if (level == pol) {
                fire |= m;
            } else {
                s->irqstat[b] &= ~m;
            }
        } else {
            if (pol && (rising & m)) {
                s->irqstat[b] |= m;      /* rising edge */
            } else if (!pol && (falling & m)) {
                s->irqstat[b] |= m;      /* falling edge */
            }
        }
    }
    s->prev_din[b] = din;
    s->irqstat[b] |= fire & armed;
    if (s->irqstat[b] & armed) {
        qemu_set_irq(s->irq[b], 1);
    } else {
        qemu_set_irq(s->irq[b], 0);
    }
}

static void pinctrl_notify_output(MxsPinctrlState *s, int b)
{
    if (s->out_hook) {
        s->out_hook(s->out_opaque, b, s->dout[b] & s->doe[b]);
    }
}

/* public: inject external input levels (used by the keyboard) */
void mxs_pinctrl_set_input(DeviceState *dev, int bank, uint32_t mask,
                           uint32_t value)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    if (bank < 0 || bank >= PINCTRL_BANKS) {
        return;
    }
    s->ext_input[bank] = (s->ext_input[bank] & ~mask) | (value & mask);
    pinctrl_update_irq(s, bank);
}

/*
 * Re-arm the Brain keyboard column IRQs.
 *
 * The EDNA2 KBD driver's ISR tears down its column IRQs after the
 * first keypress (IRQEN CLR per column, then PIN2IRQ/IRQLEV/IRQPOL)
 * and its IST -- which would normally re-arm them -- never runs
 * in QEMU (the shell/GWES activation gap, Issue B).  The model
 * restores the columns to their initialized state so a later key
 * can fire IRQ 125 again: PIN2IRQ + IRQEN armed, level-sensitive,
 * low level active.  Only the keyboard columns are touched
 * (GPIO2_16..21 = 0x3f0000, GPIO4_8 = 0x100).  Safe to call when no
 * key is pressed (columns high: no IRQ fires) and when a key is
 * held (column low: the IRQ fires, delivering the keypress).
 */
void mxs_pinctrl_rearm_kbd_irq(DeviceState *dev)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    /* bank 2: columns 0..5 = GPIO2_16..21 */
    s->pin2irq[2] |= 0x3f0000u;
    s->irqen[2]   |= 0x3f0000u;
    s->irqlev[2]  |= 0x3f0000u;
    s->irqpol[2]  &= ~0x3f0000u;
    /* bank 4: column 6 = GPIO4_8 */
    s->pin2irq[4] |= 0x100u;
    s->irqen[4]   |= 0x100u;
    s->irqlev[4]  |= 0x100u;
    s->irqpol[4]  &= ~0x100u;
    pinctrl_update_irq(s, 2);
    pinctrl_update_irq(s, 4);
}

/* public: register a GPIO output observer (matrix keyboard) */
void mxs_pinctrl_set_output_hook(DeviceState *dev, MxsGpioOutNotify fn,
                                  void *opaque)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    s->out_hook = fn;
    s->out_opaque = opaque;
    for (int b = 0; b < PINCTRL_BANKS; b++) {
        pinctrl_notify_output(s, b);
    }
}

/* public: read the effective GPIO output state of a bank */
uint32_t mxs_pinctrl_get_output(DeviceState *dev, int bank)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    if (bank < 0 || bank >= PINCTRL_BANKS) {
        return 0;
    }
    return s->dout[bank] & s->doe[bank];
}

/* public: read the GPIO output-enable state of a bank */
uint32_t mxs_pinctrl_get_doe(DeviceState *dev, int bank)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    if (bank < 0 || bank >= PINCTRL_BANKS) {
        return 0;
    }
    return s->doe[bank];
}

static uint64_t pinctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsPinctrlState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (off != 0) {
        return 0;
    }

    if (reg < 0x700) {
        return s->mux[reg / 0x10];
    }
    for (int b = 0; b < PINCTRL_BANKS; b++) {
        if (reg == REG_DOUT(b)) {
            return s->dout[b];
        }
        if (reg == REG_DIN(b)) {
            return pinctrl_din(s, b);
        }
        if (reg == REG_DOE(b)) {
            return s->doe[b];
        }
        if (reg == REG_PIN2IRQ(b)) {
            return s->pin2irq[b];
        }
        if (reg == REG_IRQEN(b)) {
            return s->irqen[b];
        }
        if (reg == REG_IRQLEV(b)) {
            return s->irqlev[b];
        }
        if (reg == REG_IRQPOL(b)) {
            return s->irqpol[b];
        }
        if (reg == REG_IRQSTAT(b)) {
            return s->irqstat[b];
        }
    }

    qemu_log_mask(LOG_UNIMP, "pinctrl: unimplemented read 0x%" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

static void pinctrl_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    MxsPinctrlState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (reg < 0x700) {
        if (off == 0) {
            s->mux[reg / 0x10] = value;
        } else if (off == 4) {
            s->mux[reg / 0x10] |= value;
        } else if (off == 8) {
            s->mux[reg / 0x10] &= ~value;
        } else {
            s->mux[reg / 0x10] ^= value;
        }
        return;
    }

    for (int b = 0; b < PINCTRL_BANKS; b++) {
        uint32_t *target = NULL;
        bool irq_relevant = false;

        if (reg == REG_DOUT(b)) {
            target = &s->dout[b];
        } else if (reg == REG_DOE(b)) {
            target = &s->doe[b];
        } else if (reg == REG_PIN2IRQ(b)) {
            target = &s->pin2irq[b];
            irq_relevant = true;
        } else if (reg == REG_IRQEN(b)) {
            target = &s->irqen[b];
            irq_relevant = true;
        } else if (reg == REG_IRQLEV(b)) {
            target = &s->irqlev[b];
            irq_relevant = true;
        } else if (reg == REG_IRQPOL(b)) {
            target = &s->irqpol[b];
            irq_relevant = true;
        } else if (reg == REG_IRQSTAT(b)) {
            /* write-1-to-clear via the CLR alias or direct 0 */
            if (off == 8) {
                s->irqstat[b] &= ~value;
            } else if (off == 0) {
                s->irqstat[b] = value;
            }
            pinctrl_update_irq(s, b);
            return;
        }

        if (target) {
            switch (off) {
            case 0:
                *target = value;
                break;
            case 4:
                *target |= value;
                break;
            case 8:
                *target &= ~value;
                break;
            case 12:
                *target ^= value;
                break;
            }
            if (reg == REG_DOUT(b) || reg == REG_DOE(b)) {
                /* DIN follows the pin output; the keyboard watches it */
                pinctrl_update_irq(s, b);
                pinctrl_notify_output(s, b);
            } else if (irq_relevant) {
                pinctrl_update_irq(s, b);
            }
            return;
        }
    }

    qemu_log_mask(LOG_UNIMP, "pinctrl: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, value);
}

static const MemoryRegionOps pinctrl_ops = {
    .read = pinctrl_read,
    .write = pinctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void pinctrl_reset(DeviceState *dev)
{
    MxsPinctrlState *s = MXS_PINCTRL(dev);

    memset(s->mux, 0, sizeof(s->mux));
    memset(s->dout, 0, sizeof(s->dout));
    /*
     * ext_input[] is NOT cleared: it models levels driven from outside
     * the SoC (board straps like the SH6 GPIO0_5 microSD-present line,
     * the card-detect switch, matrix-keyboard rows).  Real pins keep
     * their external level across a chip reset, and EBOOT re-reads the
     * DIN0 strap after every (power-cycle) reset.
     */
    memset(s->doe, 0, sizeof(s->doe));
    memset(s->pin2irq, 0, sizeof(s->pin2irq));
    memset(s->irqen, 0, sizeof(s->irqen));
    memset(s->irqlev, 0, sizeof(s->irqlev));
    memset(s->irqpol, 0, sizeof(s->irqpol));
    memset(s->irqstat, 0, sizeof(s->irqstat));
    memset(s->prev_din, 0, sizeof(s->prev_din));
}

static void pinctrl_init(Object *obj)
{
    MxsPinctrlState *s = MXS_PINCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pinctrl_ops, s,
                          "mxs-pinctrl", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int b = 0; b < PINCTRL_BANKS; b++) {
        sysbus_init_irq(sbd, &s->irq[b]);
    }
}

static void pinctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pinctrl_reset);
}

static const TypeInfo pinctrl_info = {
    .name = TYPE_MXS_PINCTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsPinctrlState),
    .instance_init = pinctrl_init,
    .class_init = pinctrl_class_init,
};

static void pinctrl_register(void)
{
    type_register_static(&pinctrl_info);
}
type_init(pinctrl_register)
