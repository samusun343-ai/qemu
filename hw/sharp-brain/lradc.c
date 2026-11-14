/*
 * i.MX28 LRADC emulation with 4-wire resistive touchscreen support.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * HW_LRADC, base 0x80050000.
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
#define BM_CTRL0_SFTRST          (1 << 31)
#define BM_CTRL0_CLKGATE         (1 << 30)
#define BM_CTRL0_TOUCH_DETECT_EN (1 << 23)
#define BM_CTRL0_YNNSW           (1 << 21)
#define BM_CTRL0_YPNSW           (1 << 20)
#define BM_CTRL0_YPPSW           (1 << 19)
#define BM_CTRL0_XNNSW           (1 << 18)
#define BM_CTRL0_XNPSW           (1 << 17)
#define BM_CTRL0_XPPSW           (1 << 16)
#define BM_CTRL0_SCHEDULE        0xff

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

/* REG/SET/CLR/TOG alias write (alias selector = offset bits 3:2) */
static void lradc_alias_write(uint32_t *reg, uint32_t off, uint32_t value)
{
    switch (off) {
    case 0x0: *reg = value; break;
    case 0x4: *reg |= value; break;
    case 0x8: *reg &= ~value; break;
    case 0xc: *reg ^= value; break;
    }
}

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
 * Sample one conversion.
 *
 * The 4-wire touch plate readings only exist while the pen is down;
 * every other channel (battery, VDD5V, card detect) is a plain
 * voltage input and must return its value regardless of the pen.
 * The old structure returned 0 for *all* channels when the pen was
 * up, which fed the guest 0 V batteries and a dead card-detect line.
 */
static uint32_t lradc_sample(MxsLradcState *s, int ch)
{
    uint32_t plate = s->ctrl0;
    bool x_mode = (plate & BM_CTRL0_XPPSW) && (plate & BM_CTRL0_XNNSW);
    bool y_mode = (plate & BM_CTRL0_YPPSW) && (plate & BM_CTRL0_YNNSW);
    bool p_mode = (plate & BM_CTRL0_YPPSW) && (plate & BM_CTRL0_XNNSW);

    /* full scale ~ 1.85V in 12 bits (LRADC range), values fit 18 bits */
    if (s->touch_down) {
        /*
         * Panel transfer functions, derived from the guest's own
         * stored calibration (HKLM\HARDWARE\DEVICEMAP\TOUCH
         * \CalibrationData in the eMMC image's registry hive):
         *   "1931,1961 889,2993 888,906 2961,920 2958,3039"
         * i.e. raw (1931,1961) at screen centre (427,240), and raw
         * (889,2993) / (888,906) / (2961,920) / (2958,3039) at the
         * four calibration points (171,96) / (171,384) /
         * (683,384) / (683,96) -- the guest's calibration UI uses
         * 20%-inset points, as verified from where its mapped
         * points land (a 10%-inset reading of the same string
         * compresses the right half of the panel into the middle
         * tiles).  Solving raw = affine(screen) for those five
         * points (residual <= 11 counts at every point):
         *   Xval =  198 + 4.041*x
         *   Yval = 3673 + 0.090*x - 7.247*y      (Y is inverted)
         * s->x/s->y are in pixel units (x_max=853, y_max=479).
         *
         * The functions previously used here were fitted to the
         * reference implementation's conversion traces instead and
         * do not match this guest's stored calibration, and its
         * pressure phase (below) returned the coordinate values,
         * which made the guest discard every right-half point
         * (taps with host x >= ~460 never acted, anywhere).  The
         * stored calibration is the ground truth for this guest:
         * it is what the real panel was calibrated against.
         */
        uint32_t xv = 198 + (uint32_t)((uint64_t)s->x * 4041 / 1000);
        uint32_t yv = 3673 + (uint32_t)((uint64_t)s->x * 90 / 1000)
                           - (uint32_t)((uint64_t)s->y * 7247 / 1000);
        if (x_mode && (ch == CH_YP || ch == CH_XP)) {
            return xv;
        }
        if (y_mode && (ch == CH_XM || ch == CH_XP)) {
            return yv;
        }
        if (p_mode && (ch == CH_XP || ch == CH_YM)) {
            /*
             * Pressure phase (YP driven, XN driven): on the real
             * panel the sensed terminals sit on one divider chain
             *   YP -> Y film -> contact -> X film -> XN,
             * with XP tapping the X film at the contact and YM
             * tapping the Y film at the contact, so YM always reads
             * higher than XP (the contact resistance drops between
             * them).  Returning the coordinate values here instead
             * (XP = Xval, YM = Yval) inverts that relation on the
             * right half of the panel, and the guest discards every
             * such point (measured 2026-11-14: points are accepted
             * iff the pressure phase reads YM > XP).  Model the
             * divider with equal plate resistances and a contact
             * resistance of ~12% of one plate.
             */
            uint32_t fxm = (uint32_t)s->x * 1000 / 853;
            uint32_t fym = (uint32_t)s->y * 1000 / 479;
            uint32_t denom = fym + 120 + fxm;

            if (ch == CH_XP) {
                return 4095 * fxm / denom;
            }
            return 4095 * (120 + fxm) / denom;
        }
    }

    /* battery / voltage channels (plain inputs, pen-independent) */
    if (ch == 0) {
        return 3000;          /* Li-ion battery ~75% (divider scaled) */
    }
    if (ch == 6) {
        return 4095;          /* VDD5V present */
    }
    if (ch == 7) {
        return 3000;          /* Li-ion battery ~75% */
    }
    /*
     * Card-detect channels: the SSP0/SSP1 card-detect pads are routed
     * to LRADC channels 4/5 as analog inputs (the EBOOT/BSP
     * BSPSDHCIsCardPresent() reads them through the LRADC value
     * registers).  A closed (present) card switch reads low; the
     * soldered eMMC on channel 4 is always present, the microSD slot
     * on channel 5 reflects the inserted state.
     */
    if (ch == 4) {
        return 0;             /* eMMC: always present */
    }
    if (ch == 5) {
        return s->sd_card_present ? 0 : 4095;
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
        /*
         * Account for oversampling: VALUE = sample * (NUM_SAMPLES+1).
         * The TOGGLE bit (ch reg bit 31) inverts at the end of every
         * conversion on the channel: the i.MX28 DKA/LRADC helpers use
         * it to recognize that a *new* sample arrived, so a conversion
         * that never flips TOGGLE looks like a stuck channel to the
         * guest (EBOOT's BSPSDHCIsCardPresent chain times out and
         * exits "NO SD card" before any bus command is issued).
         */
        uint32_t nums = (s->ch[n] >> BP_CH_NUM_SAMPLES) & 0x1f;
        uint32_t val = lradc_sample(s, phys) * (nums + 1);
        s->ch[n] = (s->ch[n] & ~BM_CH_VALUE) | (val & BM_CH_VALUE);
        s->ch[n] ^= BM_CH_TOGGLE;
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

    if (raw != !!(s->status & BM_STATUS_TOUCH_RAW)) {
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

/*
 * Sidebar touchkey strip.
 *
 * The Brain's right screen edge (x >= 800 of the 854-wide display)
 * is not panel area: it is a strip of touch keys （手書き, 50音,
 * abc, the D-pad, ツール, 決定, 戻る, ...) wired to the EDNA2
 * sub-MCU.  On the real machine (and in the reference QEMU, whose
 * "[lradc-strip]" / "[touchkey]" traces this table is taken from)
 * a press in the strip never reaches the LRADC: the sub-MCU posts
 * an event code into the mailbox slot at +0x404 and the guest's
 * TouchSub thread dispatches it.  The down codes are an active-low
 * key-matrix pattern (0xf7e with one bit cleared); the release
 * code is 0xf7e for every key.  Y band boundaries and all codes
 * were measured against the reference binary (2026-11-14).
 */
#define LRADC_STRIP_X_MIN    800
#define LRADC_STRIP_NKEYS    9
#define LRADC_STRIP_CODE_UP  0x00000f7e

static const int lradc_strip_yband[LRADC_STRIP_NKEYS - 1] = {
    44, 116, 164, 212, 260, 308, 380, 428
};
static const uint32_t lradc_strip_code[LRADC_STRIP_NKEYS] = {
    0xf7c, 0xf7a, 0xf76, 0xf5e, 0xf3e, 0xe7e, 0xd7e, 0xb7e, 0x77e
};

static int lradc_strip_index(int y)
{
    int idx = 0;

    while (idx < LRADC_STRIP_NKEYS - 1 && y >= lradc_strip_yband[idx]) {
        idx++;
    }
    return idx;
}

static void lradc_strip_post(MxsLradcState *s, uint32_t code)
{
    if (s->edna2_mb) {
        brain_edna2_mb_post_touchkey(BRAIN_EDNA2_MB(s->edna2_mb), code);
    }
}

static void lradc_input_event(DeviceState *dev, QemuConsole *src,
                              QemuInputEvent *evt)
{
    MxsLradcState *s = MXS_LRADC(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            if (evt->btn.down) {
                if (s->x >= LRADC_STRIP_X_MIN) {
                    /* strip press: goes to the sub-MCU, not the ADC */
                    s->strip_idx = lradc_strip_index(s->y);
                    lradc_strip_post(s, lradc_strip_code[s->strip_idx]);
                } else {
                    s->touch_down = true;
                }
            } else {
                if (s->strip_idx >= 0) {
                    lradc_strip_post(s, LRADC_STRIP_CODE_UP);
                    s->strip_idx = -1;
                } else {
                    s->touch_down = false;
                }
            }
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
        if (s->strip_idx >= 0) {
            /* slide along the strip: follow into the new band */
            int idx = (s->x >= LRADC_STRIP_X_MIN) ?
                      lradc_strip_index(s->y) : -1;
            if (idx >= 0 && idx != s->strip_idx) {
                s->strip_idx = idx;
                lradc_strip_post(s, lradc_strip_code[idx]);
            }
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
        qemu_log_mask(LOG_TRACE, "mxs-lradc: read STATUS -> 0x%08x\n",
                      s->status);
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
                qemu_log_mask(LOG_TRACE,
                              "mxs-lradc: read CH%d -> 0x%08x\n", n, s->ch[n]);
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

    qemu_log_mask(LOG_TRACE, "mxs-lradc: write 0x%03x = 0x%08x\n",
                  (uint32_t)offset, (uint32_t)val);
    switch (offset & ~0xcull) {
    case HW_LRADC_CTRL0:
    {
        uint32_t old = s->ctrl0;

        lradc_alias_write(&s->ctrl0, offset & 0xc, val);
        /* SFTRST rising edge -> clock gates once */
        if (!(old & BM_CTRL0_SFTRST) && (s->ctrl0 & BM_CTRL0_SFTRST)) {
            s->ctrl0 |= BM_CTRL0_CLKGATE;
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
        lradc_alias_write(&s->ctrl2, offset & 0xc, val);
        return;
    case HW_LRADC_CTRL3:
        lradc_alias_write(&s->ctrl3, offset & 0xc, val);
        return;
    case HW_LRADC_CTRL4:
        lradc_alias_write(&s->ctrl4, offset & 0xc, val);
        return;
    case HW_LRADC_CONVERSION:
        lradc_alias_write(&s->conversion, offset & 0xc, val);
        return;
    default:
        for (int n = 0; n < MXS_LRADC_NUM_CH; n++) {
            if ((offset & ~0xcull) == HW_LRADC_CH(n)) {
                lradc_alias_write(&s->ch[n], offset & 0xc, val);
                return;
            }
        }
        for (int n = 0; n < 4; n++) {
            if ((offset & ~0xcull) == HW_LRADC_DELAY(n)) {
                lradc_alias_write(&s->delay[n], offset & 0xc, val);
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
                lradc_alias_write(&s->threshold[n], offset & 0xc, val);
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
    s->strip_idx = -1;
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
    DEFINE_PROP_LINK("edna2-mb", MxsLradcState, edna2_mb,
                     TYPE_BRAIN_EDNA2_MB, DeviceState *),
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
