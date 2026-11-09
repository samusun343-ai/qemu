/*
 * i.MX28 RTC emulation (stmp3xxx RTC block).
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The block carries a battery-backed seconds/milliseconds counter
 * plus six "persistent" registers that survive resets; the Windows
 * CE power driver arms PERSISTENT0.AUTO_RESTART (bit 17) and polls
 * until it reads back, and the OAL refuses to use the RTC unless the
 * VERSION register identifies a present block.
 *
 * Registers (stmp 16-byte layout):
 *   CTRL          0x00   (SFTRST/CLKGATE coupling, IRQ enables)
 *   STAT          0x10
 *   MILLISECONDS  0x20   free-running ms (write re-phases)
 *   SECONDS       0x30   wall-clock seconds (write sets the epoch)
 *   ALARM         0x40
 *   WATCHDOG      0x50   (stored; reset not modelled yet)
 *   PERSISTENT0-5 0x60..0xb0
 *   DEBUG         0xc0
 *   VERSION       0x130  major 1 (stmp3xxx-style block on i.MX28)
 *
 * IRQ: ICOLL 29 (alarm / onemsec).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "brain.h"

#define HW_RTC_CTRL           0x000
#define HW_RTC_STAT           0x010
#define HW_RTC_MILLISECONDS   0x020
#define HW_RTC_SECONDS        0x030
#define HW_RTC_ALARM          0x040
#define HW_RTC_WATCHDOG       0x050
#define HW_RTC_PERSISTENT(n)  (0x060 + (n) * 0x10)
#define HW_RTC_DEBUG          0x0c0
#define HW_RTC_VERSION        0x130

#define BM_RTC_CTRL_SFTRST        (1u << 31)
#define BM_RTC_CTRL_CLKGATE       (1u << 30)
#define BM_RTC_CTRL_ALARM_IRQ     (1u << 2)
#define BM_RTC_CTRL_ONEMSEC_IRQ   (1u << 3)
#define BM_RTC_CTRL_WATCHDOGEN    (1u << 4)
#define BM_RTC_CTRL_IRQ           (BM_RTC_CTRL_ALARM_IRQ | \
                                   BM_RTC_CTRL_ONEMSEC_IRQ)

#define BM_RTC_PERSISTENT0_AUTO_RESTART   (1u << 17)

/* epoch the seconds counter starts from: 2012-05-07 (kernel build) */
#define RTC_DEFAULT_EPOCH_SECONDS  1336348800ULL

OBJECT_DECLARE_SIMPLE_TYPE(MxsRtcState, MXS_RTC)

struct MxsRtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ctrl, stat;
    uint32_t alarm, watchdog;
    uint32_t persistent[6];
    uint32_t debug;

    int64_t sec_base_ns;       /* host ns when the seconds epoch was set */
    uint64_t epoch_seconds;    /* guest-visible seconds at sec_base_ns */
    int64_t ms_base_ns;        /* host ns when MILLISECONDS was written  */
    uint32_t ms_offset;        /* guest-visible ms value at ms_base_ns   */
};

static void rtc_irq_update(MxsRtcState *s)
{
    bool irq = !!(s->ctrl & BM_RTC_CTRL_IRQ);
    qemu_set_irq(s->irq, irq);
}

static uint32_t rtc_seconds(MxsRtcState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return s->epoch_seconds + (uint64_t)((now - s->sec_base_ns) / 1000000000);
}

static uint32_t rtc_milliseconds(MxsRtcState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return s->ms_offset + (uint32_t)((now - s->ms_base_ns) / 1000000);
}

static uint64_t rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsRtcState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (off != 0) {
        return 0;
    }

    switch (reg) {
    case HW_RTC_CTRL:
        return s->ctrl;
    case HW_RTC_STAT:
        return s->stat;
    case HW_RTC_MILLISECONDS:
        return rtc_milliseconds(s);
    case HW_RTC_SECONDS:
        return rtc_seconds(s);
    case HW_RTC_ALARM:
        return s->alarm;
    case HW_RTC_WATCHDOG:
        return s->watchdog;
    case HW_RTC_DEBUG:
        return s->debug;
    case HW_RTC_VERSION:
        return 0x010000;       /* major 1, minor 0 */
    default:
        if (reg >= HW_RTC_PERSISTENT(0) && reg <= HW_RTC_PERSISTENT(5)) {
            return s->persistent[(reg - HW_RTC_PERSISTENT(0)) / 0x10];
        }
        break;
    }

    qemu_log_mask(LOG_UNIMP, "mxs-rtc: unimplemented read 0x%" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

static void rtc_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    MxsRtcState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (reg >= HW_RTC_PERSISTENT(0) && reg <= HW_RTC_PERSISTENT(5)) {
        int n = (reg - HW_RTC_PERSISTENT(0)) / 0x10;

        switch (off) {
        case 0:  s->persistent[n] = value; break;
        case 4:  s->persistent[n] |= value; break;
        case 8:  s->persistent[n] &= ~value; break;
        case 12: s->persistent[n] ^= value; break;
        }
        return;
    }

    switch (reg) {
    case HW_RTC_CTRL:
    {
        uint32_t old = s->ctrl;

        switch (off) {
        case 0:  s->ctrl = value; break;
        case 4:  s->ctrl |= value; break;
        case 8:  s->ctrl &= ~value; break;
        case 12: s->ctrl ^= value; break;
        }
        /* SFTRST rising edge gates the clock (guest reset poll) */
        if (!(old & BM_RTC_CTRL_SFTRST) && (s->ctrl & BM_RTC_CTRL_SFTRST)) {
            s->ctrl |= BM_RTC_CTRL_CLKGATE;
        }
        /* the IRQ status bits clear through every alias */
        if (off == 8 || off == 12) {
            s->ctrl &= ~BM_RTC_CTRL_IRQ;
        }
        if (s->ctrl & BM_RTC_CTRL_WATCHDOGEN) {
            qemu_log_mask(LOG_UNIMP, "mxs-rtc: watchdog enabled "
                          "(0x%x); reset not modelled yet\n",
                          s->watchdog);
        }
        rtc_irq_update(s);
        return;
    }
    case HW_RTC_STAT:
        s->stat &= ~value;
        return;
    case HW_RTC_MILLISECONDS:
        s->ms_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->ms_offset = value;
        return;
    case HW_RTC_SECONDS:
        s->sec_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->epoch_seconds = value;
        return;
    case HW_RTC_ALARM:
        s->alarm = value;
        return;
    case HW_RTC_WATCHDOG:
        s->watchdog = value;
        return;
    case HW_RTC_DEBUG:
        s->debug = value;
        return;
    default:
        break;
    }

    qemu_log_mask(LOG_UNIMP, "mxs-rtc: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, value);
}

static const MemoryRegionOps rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void rtc_reset(DeviceState *dev)
{
    MxsRtcState *s = MXS_RTC(dev);

    s->ctrl = 0;
    s->stat = 0;
    s->alarm = 0;
    s->watchdog = 0;
    s->debug = 0;
    /* persistent registers survive resets (battery backed) */
    s->sec_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->epoch_seconds = RTC_DEFAULT_EPOCH_SECONDS;
    s->ms_base_ns = s->sec_base_ns;
    s->ms_offset = 0;
    qemu_set_irq(s->irq, 0);
}

static void rtc_init(Object *obj)
{
    MxsRtcState *s = MXS_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rtc_ops, s, "mxs-rtc", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "i.MX28 RTC (stmp3xxx block)";
    device_class_set_legacy_reset(dc, rtc_reset);
}

static const TypeInfo rtc_info = {
    .name = TYPE_MXS_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsRtcState),
    .instance_init = rtc_init,
    .class_init = rtc_class_init,
};

static void rtc_register(void)
{
    type_register_static(&rtc_info);
}
type_init(rtc_register)
