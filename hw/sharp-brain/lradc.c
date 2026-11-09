/*
 * i.MX28 LRADC emulation with 4-wire resistive touchscreen support.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Plate switch encoding (HW_LRADC_CTRL0, i.MX28):
 *   bit 23 TOUCH_DETECT_ENABLE, bit 21 YNNSW, bit 20 YPNSW,
 *   bit 19 YPPSW, bit 18 XNNSW, bit 17 XNPSW, bit 16 XPPSW.
 *
 * Sampling modes used by touchscreen drivers:
 *   X: XPPSW|XNNSW (read Y+), Y: YPPSW|YNNSW (read X-),
 *   pressure: YPPSW|XNNSW, detect: TOUCH_DETECT_ENABLE.
 *
 * Virtual channels 6/7 can be mapped onto physical channels 2..5
 * through HW_LRADC_CTRL4 (LRADCSELECT fields).
 *
 * Battery/auxiliary channels report fixed plausible values so the
 * WinCE boot loader's battery polling (channel 7) completes.
 *
 * Registers use the stmp 16-byte layout (REG/SET/CLR/TOG).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "ui/console.h"
#include "ui/input.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "brain.h"

#define HW_LRADC_CTRL0        0x000
#define HW_LRADC_CTRL1        0x010
#define HW_LRADC_CTRL2        0x020
#define HW_LRADC_CTRL3        0x030
#define HW_LRADC_STATUS       0x040
#define HW_LRADC_CH(n)        (0x050 + (n) * 0x10)
#define HW_LRADC_DELAY(n)     (0x0d0 + (n) * 0x10)
#define HW_LRADC_CONVERSION   0x130
#define HW_LRADC_CTRL4        0x140
#define HW_LRADC_THRESHOLD(n) (0x150 + (n) * 0x10)
#define HW_LRADC_VERSION      0x170

/* CTRL0 bits */
#define BM_CTRL0_TOUCH_DETECT_EN (1 << 23)
#define BM_CTRL0_YNNSW          (1 << 21)
#define BM_CTRL0_YPPSW          (1 << 19)
#define BM_CTRL0_XNNSW          (1 << 18)
#define BM_CTRL0_XPPSW          (1 << 16)
#define BM_CTRL0_SCHEDULE       0xff

/* CTRL1 bits */
#define BM_CTRL1_CH_IRQ_EN(n)   (1 << (16 + (n)))
#define BM_CTRL1_CH_IRQ(n)      (1 << (n))
#define BM_CTRL1_TOUCH_IRQ_EN   (1 << 24)
#define BM_CTRL1_TOUCH_IRQ      (1 << 8)
#define CTRL1_ALL_IRQ           0x00000fff
#define CTRL1_ALL_IRQ_EN        0x0fff0000

/* STATUS bits */
#define BM_STATUS_TOUCH_PRESENT  (1 << 24)
#define BM_STATUS_TOUCH_RAW      (1 << 0)   /* 1 = touch detected */

/* CH bits */
#define BM_CH_TOGGLE     (1 << 31)
#define BM_CH_ACCUMULATE (1 << 29)
#define BP_CH_NUM_SAMPLES 24
#define BM_CH_VALUE      0x0003ffff

/* DELAY bits */
#define BP_DELAY_TRIGGER_LRADCS 24
#define BM_DELAY_KICK    (1 << 20)

#define CH_XP 2
#define CH_YP 3
#define CH_XM 4
#define CH_YM 5

static void lradc_update_irq(MxsLradcState *s)
{
    uint32_t irq = s->ctrl1 & CTRL1_ALL_IRQ;
    uint32_t en = (s->ctrl1 & CTRL1_ALL_IRQ_EN) >> 16;

    /* touchscreen line: touch detect + all channel conversions */
    bool ts = ((s->ctrl1 & BM_CTRL1_TOUCH_IRQ) &&
               (s->ctrl1 & BM_CTRL1_TOUCH_IRQ_EN)) ||
              !!(irq & en);
    qemu_set_irq(s->irqs[0], ts);

    for (int n = 0; n < 8; n++) {
        qemu_set_irq(s->irqs[3 + n], !!(irq & en & (1 << n)));
    }
}

/*
 * Sample the currently selected plate configuration.
 * Returns an 18-bit LRADC value for the given (physical) channel.
 */
static uint32_t lradc_sample(MxsLradcState *s, int ch)
{
    uint32_t plate = s->ctrl0;
    bool x_mode = (plate & BM_CTRL0_XPPSW) && (plate & BM_CTRL0_XNNSW);
    bool y_mode = (plate & BM_CTRL0_YPPSW) && (plate & BM_CTRL0_YNNSW);
    bool p_mode = (plate & BM_CTRL0_YPPSW) && (plate & BM_CTRL0_XNNSW);
    bool detect = plate & BM_CTRL0_TOUCH_DETECT_EN;

    /* full scale ~ 1.85V in 12 bits (LRADC range), values fit 18 bits */
    if (!s->touch_down) {
        /* open circuit: no contact, plates float */
        if (detect) {
            return 0;         /* no touch */
        }
        return 0;
    }

    if (x_mode && ch == CH_YP) {
        return ((uint64_t)s->x * 4095) / MAX(s->x_max, 1);
    }
    if (y_mode && ch == CH_XM) {
        return ((uint64_t)s->y * 4095) / MAX(s->y_max, 1);
    }
    if (p_mode && (ch == CH_YM || ch == CH_XP)) {
        return 100;           /* some contact resistance */
    }
    /* battery / voltage channels */
    if (ch == 0) {
        return 3000;          /* Li-ion battery ~75% (divider scaled) */
    }
    if (ch == 6) {
        return 4095;          /* VDD5V present */
    }
    if (ch == 7) {
        return 3000;          /* Li-ion battery ~75% */
    }
    return 0;
}

/* run one conversion pass on the channels in the mask (bit n = ch n) */
static void lradc_convert(MxsLradcState *s, uint32_t chmask)
{
    for (int n = 0; n < MXS_LRADC_NUM_CH; n++) {
        int phys = n;
        if (!(chmask & (1 << n))) {
            continue;
        }
        /* virtual channels 6/7 map through CTRL4 */
        if (n == 6) {
            phys = (s->ctrl4 >> 24) & 0xf;
        } else if (n == 7) {
            phys = (s->ctrl4 >> 28) & 0xf;
        }
        if (phys > 7) {
            phys = 7;
        }
        /* account for oversampling: VALUE = sample * (NUM_SAMPLES+1) */
        uint32_t nums = (s->ch[n] >> BP_CH_NUM_SAMPLES) & 0x1f;
        uint32_t val = lradc_sample(s, phys) * (nums + 1);
        s->ch[n] = (s->ch[n] & ~BM_CH_VALUE) | (val & BM_CH_VALUE);
        /* conversion complete IRQ (ch n status at bit n) */
        s->ctrl1 |= BM_CTRL1_CH_IRQ(n);
        qemu_set_irq(s->irqs[3 + n],
                     !!(s->ctrl1 & (1 << n) & ((s->ctrl1 >> 16) & 0xff)));
    }
    lradc_update_irq(s);
}

static void lradc_touch_event_check(MxsLradcState *s)
{
    bool raw = s->touch_down;

    if (raw != !(s->status & BM_STATUS_TOUCH_RAW)) {
        /* state changed */
        if (raw) {
            s->status |= BM_STATUS_TOUCH_RAW | BM_STATUS_TOUCH_PRESENT;
        } else {
            s->status &= ~(BM_STATUS_TOUCH_RAW | BM_STATUS_TOUCH_PRESENT);
        }
        if (s->ctrl0 & BM_CTRL0_TOUCH_DETECT_EN) {
            s->ctrl1 |= BM_CTRL1_TOUCH_IRQ;
        }
        lradc_update_irq(s);
    }
}

/* ------------------------------------------------------------------ */
/* touchscreen input                                                   */

static void lradc_input_event(DeviceState *dev, QemuConsole *src,
                              QemuInputEvent *evt)
{
    MxsLradcState *s = MXS_LRADC(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            s->touch_down = evt->btn.down;
        }
        break;
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->x = qemu_input_scale_axis(evt->abs.value,
                                         INPUT_EVENT_ABS_MIN,
                                         INPUT_EVENT_ABS_MAX,
                                         0, s->x_max);
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->y = qemu_input_scale_axis(evt->abs.value,
                                         INPUT_EVENT_ABS_MIN,
                                         INPUT_EVENT_ABS_MAX,
                                         0, s->y_max);
        }
        break;
    default:
        break;
    }
}

static void lradc_input_sync(DeviceState *dev)
{
    MxsLradcState *s = MXS_LRADC(dev);

    lradc_touch_event_check(s);
}

static const QemuInputHandler lradc_handler = {
    .name  = "imx28-lradc-touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = lradc_input_event,
    .sync  = lradc_input_sync,
};

/* ------------------------------------------------------------------ */
/* register access                                                     */

static uint64_t lradc_read(void *opaque, hwaddr offset, unsigned size)
{
    MxsLradcState *s = opaque;

    switch (offset & ~0xcull) {
    case HW_LRADC_CTRL0:
        return s->ctrl0;
    case HW_LRADC_CTRL1:
        /* CTRL1 read returns enables + status merged */
        return s->ctrl1;
    case HW_LRADC_CTRL2:
        return s->ctrl2;
    case HW_LRADC_CTRL3:
        return s->ctrl3;
    case HW_LRADC_CTRL4:
        return s->ctrl4;
    case HW_LRADC_STATUS:
    {
        uint32_t v = s->status & ~BM_STATUS_TOUCH_RAW;
        if (s->touch_down) {
            v |= BM_STATUS_TOUCH_RAW | BM_STATUS_TOUCH_PRESENT;
        }
        return v;
    }
    case HW_LRADC_CONVERSION:
        return s->conversion;
    case HW_LRADC_VERSION:
        return 0x010000;
    default:
        for (int n = 0; n < MXS_LRADC_NUM_CH; n++) {
            if ((offset & ~0xcull) == HW_LRADC_CH(n)) {
                return s->ch[n];
            }
        }
        for (int n = 0; n < 4; n++) {
            if ((offset & ~0xcull) == HW_LRADC_DELAY(n)) {
                return s->delay[n];
            }
        }
        for (int n = 0; n < 2; n++) {
            if ((offset & ~0xcull) == HW_LRADC_THRESHOLD(n)) {
                return s->threshold[n];
            }
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "mxs-lradc: read of unknown reg 0x%" HWADDR_PRIx "\n",
                  offset);
    return 0;
}

static void lradc_write(void *opaque, hwaddr offset, uint64_t val,
                        unsigned size)
{
    MxsLradcState *s = opaque;

    switch (offset & ~0xcull) {
    case HW_LRADC_CTRL0:
    {
        uint32_t old = s->ctrl0;

        switch (offset & 0xc) {
        case 0x0: s->ctrl0 = val; break;
        case 0x4: s->ctrl0 |= val; break;
        case 0x8: s->ctrl0 &= ~val; break;
        case 0xc: s->ctrl0 ^= val; break;
        }
        /* SFTRST rising edge -> clock gates once */
        if (!(old & (1u << 31)) && (s->ctrl0 & (1u << 31))) {
            s->ctrl0 |= 1u << 30;
        }
        /* writing SCHEDULE bits triggers an immediate conversion */
        if (val & BM_CTRL0_SCHEDULE) {
            lradc_convert(s, val & BM_CTRL0_SCHEDULE);
        }
        return;
    }
    case HW_LRADC_CTRL1:
        /* write to CLR offset clears IRQ status bits */
        if ((offset & 0xc) == 0x8) {
            s->ctrl1 &= ~(val & CTRL1_ALL_IRQ);
        } else {
            uint32_t old = s->ctrl1;
            switch (offset & 0xc) {
            case 0x0: s->ctrl1 = val; break;
            case 0x4: s->ctrl1 |= val; break;
            case 0xc: s->ctrl1 ^= val; break;
            }
            s->ctrl1 = (s->ctrl1 & ~CTRL1_ALL_IRQ) | (old & CTRL1_ALL_IRQ);
        }
        lradc_update_irq(s);
        return;
    case HW_LRADC_CTRL2:
        switch (offset & 0xc) {
        case 0x0: s->ctrl2 = val; break;
        case 0x4: s->ctrl2 |= val; break;
        case 0x8: s->ctrl2 &= ~val; break;
        case 0xc: s->ctrl2 ^= val; break;
        }
        return;
    case HW_LRADC_CTRL3:
        switch (offset & 0xc) {
        case 0x0: s->ctrl3 = val; break;
        case 0x4: s->ctrl3 |= val; break;
        case 0x8: s->ctrl3 &= ~val; break;
        case 0xc: s->ctrl3 ^= val; break;
        }
        return;
    case HW_LRADC_CTRL4:
        switch (offset & 0xc) {
        case 0x0: s->ctrl4 = val; break;
        case 0x4: s->ctrl4 |= val; break;
        case 0x8: s->ctrl4 &= ~val; break;
        case 0xc: s->ctrl4 ^= val; break;
        }
        return;
    case HW_LRADC_CONVERSION:
        switch (offset & 0xc) {
        case 0x0: s->conversion = val; break;
        case 0x4: s->conversion |= val; break;
        case 0x8: s->conversion &= ~val; break;
        case 0xc: s->conversion ^= val; break;
        }
        return;
    default:
        for (int n = 0; n < MXS_LRADC_NUM_CH; n++) {
            if ((offset & ~0xcull) == HW_LRADC_CH(n)) {
                switch (offset & 0xc) {
                case 0x0: s->ch[n] = val; break;
                case 0x4: s->ch[n] |= val; break;
                case 0x8: s->ch[n] &= ~val; break;
                case 0xc: s->ch[n] ^= val; break;
                }
                return;
            }
        }
        for (int n = 0; n < 4; n++) {
            if ((offset & ~0xcull) == HW_LRADC_DELAY(n)) {
                switch (offset & 0xc) {
                case 0x0: s->delay[n] = val; break;
                case 0x4: s->delay[n] |= val; break;
                case 0x8: s->delay[n] &= ~val; break;
                case 0xc: s->delay[n] ^= val; break;
                }
                if (s->delay[n] & BM_DELAY_KICK) {
                    uint32_t mask = s->delay[n] >> BP_DELAY_TRIGGER_LRADCS;
                    s->delay[n] &= ~BM_DELAY_KICK;
                    lradc_convert(s, mask);
                }
                return;
            }
        }
        for (int n = 0; n < 2; n++) {
            if ((offset & ~0xcull) == HW_LRADC_THRESHOLD(n)) {
                switch (offset & 0xc) {
                case 0x0: s->threshold[n] = val; break;
                case 0x4: s->threshold[n] |= val; break;
                case 0x8: s->threshold[n] &= ~val; break;
                case 0xc: s->threshold[n] ^= val; break;
                }
                return;
            }
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "mxs-lradc: write to unknown reg 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", offset, val);
}

static const MemoryRegionOps lradc_ops = {
    .read = lradc_read,
    .write = lradc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void lradc_reset(DeviceState *dev)
{
    MxsLradcState *s = MXS_LRADC(dev);

    s->ctrl0 = s->ctrl1 = s->ctrl2 = s->ctrl3 = s->ctrl4 = 0;
    s->status = 0;
    s->conversion = 0;
    memset(s->ch, 0, sizeof(s->ch));
    memset(s->delay, 0, sizeof(s->delay));
    memset(s->threshold, 0, sizeof(s->threshold));
    s->touch_down = false;
    for (int n = 0; n < MXS_LRADC_NUM_IRQ; n++) {
        qemu_set_irq(s->irqs[n], false);
    }
}

static void lradc_init(Object *obj)
{
    MxsLradcState *s = MXS_LRADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &lradc_ops, s,
                          "mxs-lradc", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int n = 0; n < MXS_LRADC_NUM_IRQ; n++) {
        sysbus_init_irq(sbd, &s->irqs[n]);
    }
}

static void lradc_realize(DeviceState *dev, Error **errp)
{
    MxsLradcState *s = MXS_LRADC(dev);

    /* default panel resolution (overridden by the machine) */
    if (s->x_max == 0) {
        s->x_max = 853;
    }
    if (s->y_max == 0) {
        s->y_max = 479;
    }
    QemuInputHandlerState *hs = qemu_input_handler_register(dev,
                                                            &lradc_handler);
    (void)hs;    /* handler lives as long as the device */
}

static const Property lradc_properties[] = {
    DEFINE_PROP_UINT32("x-max", MxsLradcState, x_max, 853),
    DEFINE_PROP_UINT32("y-max", MxsLradcState, y_max, 479),
};

static void lradc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "i.MX28 LRADC + 4-wire touchscreen";
    dc->realize = lradc_realize;
    device_class_set_legacy_reset(dc, lradc_reset);
    device_class_set_props(dc, lradc_properties);
}

static const TypeInfo lradc_info = {
    .name = TYPE_MXS_LRADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsLradcState),
    .instance_init = lradc_init,
    .class_init = lradc_class_init,
};

static void lradc_register(void)
{
    type_register_static(&lradc_info);
}
type_init(lradc_register)
