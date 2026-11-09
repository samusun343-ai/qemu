/*
 * i.MX28 TIMROT emulation.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Four 32-bit timers counting down at 24 MHz (SELECT=TICK_ALWAYS),
 * 32 kHz / 8 kHz / 100 Hz / 1 kHz / 1 Hz (possibly prescaled).
 *
 * Two counting modes (HW_TIMROT_TIMCTRLn):
 *
 *   - fixed-count mode (MATCH_MODE=0, the default): the counter is
 *     loaded from FIXED_COUNT (directly or through UPDATE=1) and
 *     counts down; it holds at zero when it expires.  With RELOAD=1
 *     it restarts from FIXED_COUNT.  EBOOT uses this for short
 *     delays.
 *
 *   - match mode (MATCH_MODE=1, bit 11): the counter free-runs over
 *     the whole 32-bit range (wrapping 0 -> 0xffffffff) and the IRQ
 *     is raised when the running count crosses MATCH_COUNT going
 *     down.  The Windows CE kernel uses this for the variable system
 *     tick: it programs MATCH_COUNT = current - ticks and rewrites it
 *     in the ISR (observed values 0xffffd120 = 1 ms and 0xffedb080 =
 *     100 ms idle tick at 12 MHz = 24 MHz / 2 prescale).
 *
 * Layout: ROTCTRL 0x00, ROTCOUNT 0x10, VERSION 0x120, then per
 * timer n (0x40 stride): TIMCTRLn 0x20, RUNNING_COUNT 0x30,
 * FIXED_COUNT 0x40, MATCH_COUNT 0x50.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "brain.h"

#define TIMROT_TIMCTRL(n)     (0x20 + (n) * 0x40)
#define TIMROT_RUNNING(n)     (0x30 + (n) * 0x40)
#define TIMROT_FIXED(n)       (0x40 + (n) * 0x40)
#define TIMROT_MATCH(n)       (0x50 + (n) * 0x40)
#define TIMROT_VERSION        0x120

#define TIMCTRL_IRQ           (1 << 15)
#define TIMCTRL_IRQ_EN        (1 << 14)
#define TIMCTRL_MATCH_MODE    (1 << 11)
#define TIMCTRL_POLARITY      (1 << 8)
#define TIMCTRL_UPDATE        (1 << 7)
#define TIMCTRL_RELOAD        (1 << 6)
#define TIMCTRL_PRESCALE      (3 << 4)
#define TIMCTRL_SELECT        0xf

#define SELECT_NEVER 0
#define SELECT_32KHZ 8
#define SELECT_8KHZ  9
#define SELECT_100HZ 10
#define SELECT_1KHZ  11
#define SELECT_1HZ   12
#define SELECT_TICK  15      /* 24 MHz "tick always" */

OBJECT_DECLARE_SIMPLE_TYPE(MxsTimrotState, MXS_TIMROT)

typedef struct MxsTimrotState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[4];

    uint32_t rotctrl, rotcount;
    uint32_t timctrl[4];
    uint32_t fixed_count[4];
    uint32_t match_count[4];

    /*
     * Counter model: at host time base_ns[n] the count was
     * base_count[n]; it decrements at timrot_freq() while
     * counting[n].  Match mode wraps at 32 bits, fixed-count mode
     * clamps at zero (and stops unless RELOAD).
     */
    int64_t base_ns[4];
    uint32_t base_count[4];
    bool counting[4];
    QEMUTimer *tick[4];
} MxsTimrotState;

static uint64_t timrot_freq(uint32_t ctrl)
{
    uint32_t sel = ctrl & TIMCTRL_SELECT;
    uint32_t prescale = 1 << ((ctrl & TIMCTRL_PRESCALE) >> 4);

    switch (sel) {
    case SELECT_32KHZ: return 32000 / prescale;
    case SELECT_8KHZ:  return 8000 / prescale;
    case SELECT_100HZ: return 100 / prescale;
    case SELECT_1KHZ:  return 1000 / prescale;
    case SELECT_1HZ:   return 1 / prescale;
    case SELECT_TICK:  return 24000000 / prescale;
    default:           return 0;      /* PWM/rotary inputs: not modelled */
    }
}

static int64_t timrot_ns_per_tick(uint32_t ctrl)
{
    uint64_t freq = timrot_freq(ctrl);

    if (freq == 0) {
        return 0;
    }
    return 1000000000LL / (int64_t)freq;
}

/* current value of RUNNING_COUNTn */
static uint32_t timrot_count(MxsTimrotState *s, int n)
{
    uint32_t ctrl = s->timctrl[n];
    int64_t nspt = timrot_ns_per_tick(ctrl);

    if (!s->counting[n] || nspt == 0) {
        return s->base_count[n];
    }

    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->base_ns[n];
    if (elapsed < 0) {
        elapsed = 0;
    }
    uint64_t ticks = (uint64_t)(elapsed / nspt);

    if (ctrl & TIMCTRL_MATCH_MODE) {
        /* free-running, wraps around the full 32-bit range */
        return s->base_count[n] - (uint32_t)ticks;
    }

    /* fixed-count mode: clamps at zero */
    if (ticks >= s->base_count[n]) {
        return 0;
    }
    return s->base_count[n] - (uint32_t)ticks;
}

/* freeze the current count value into the shadow state */
static void timrot_freeze(MxsTimrotState *s, int n)
{
    s->base_count[n] = timrot_count(s, n);
    s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/*
 * The QEMUTimer callback needs the device plus the timer index; use a
 * small thunk array instead of embedding structs.
 */
typedef struct TimrotCb {
    MxsTimrotState *s;
    int n;
} TimrotCb;

static TimrotCb timrot_cbs[4];     /* filled in realize (one device) */

/* (re)arm the expiry timer for timer n */
static void timrot_arm(MxsTimrotState *s, int n)
{
    uint32_t ctrl = s->timctrl[n];
    int64_t nspt = timrot_ns_per_tick(ctrl);

    timer_del(s->tick[n]);

    if (!s->counting[n] || nspt == 0) {
        return;
    }

    if (ctrl & TIMCTRL_MATCH_MODE) {
        if (!(ctrl & TIMCTRL_IRQ_EN)) {
            return;     /* comparator IRQ is the only event */
        }
        /*
         * Ticks until the running count drops to MATCH_COUNT
         * (wrap-aware: the count is heading down through 32 bits).
         */
        uint32_t delta = s->base_count[n] - s->match_count[n];
        int64_t when = s->base_ns[n] + delta * nspt;
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (when <= now) {
            /* match is in the past: next crossing is one wrap away */
            when = now + (((1ULL << 32) - (uint32_t)(now - when))
                          / (uint64_t)nspt + 1) * nspt;
        }
        timer_mod(s->tick[n], when);
    } else {
        /* fixed-count mode: expiry when the count reaches zero */
        if (s->base_count[n] == 0) {
            if (ctrl & TIMCTRL_RELOAD && s->fixed_count[n]) {
                s->base_count[n] = s->fixed_count[n];
                s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            } else {
                return;
            }
        }
        timer_mod(s->tick[n],
                  s->base_ns[n] + s->base_count[n] * nspt);
    }
}

static void timrot_expire(void *opaque)
{
    TimrotCb *cb = opaque;
    MxsTimrotState *s = cb->s;
    int n = cb->n;
    uint32_t ctrl = s->timctrl[n];

    if (ctrl & TIMCTRL_MATCH_MODE) {
        /* running count just crossed MATCH_COUNT going down */
        if (ctrl & TIMCTRL_IRQ_EN) {
            s->timctrl[n] |= TIMCTRL_IRQ;
            qemu_set_irq(s->irq[n], 1);
        }
        if (ctrl & TIMCTRL_RELOAD) {
            /* counter restarts from FIXED_COUNT on match */
            s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->base_count[n] = s->fixed_count[n];
        }
        /* comparator stays armed for the next wrap crossing */
        timrot_arm(s, n);
        return;
    }

    /* fixed-count expiry */
    if (ctrl & TIMCTRL_IRQ_EN) {
        s->timctrl[n] |= TIMCTRL_IRQ;
        qemu_set_irq(s->irq[n], 1);
    }
    if (ctrl & TIMCTRL_RELOAD) {
        s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->base_count[n] = s->fixed_count[n];
        timrot_arm(s, n);
    } else {
        s->counting[n] = false;
        s->base_count[n] = 0;
        s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void timctrl_updated(MxsTimrotState *s, int n)
{
    uint32_t ctrl = s->timctrl[n];
    bool want_run = timrot_freq(ctrl) != 0;

    if (want_run && !s->counting[n]) {
        /* (re)start counting from the current shadow value */
        s->counting[n] = true;
        s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    } else if (!want_run && s->counting[n]) {
        timrot_freeze(s, n);
        s->counting[n] = false;
    }

    if ((ctrl & TIMCTRL_UPDATE) && !(ctrl & TIMCTRL_MATCH_MODE)) {
        /* UPDATE transfers FIXED_COUNT into the running count */
        s->base_count[n] = s->fixed_count[n];
        s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }

    timrot_arm(s, n);
}

static uint64_t timrot_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsTimrotState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (off != 0) {
        return 0;
    }

    if (reg == 0x00) {
        return s->rotctrl;
    }
    if (reg == 0x10) {
        return s->rotcount;
    }
    /* VERSION register */
    if (reg == TIMROT_VERSION) {
        return (2 << 24);       /* major version 2 (i.MX28) */
    }

    if (reg >= TIMROT_TIMCTRL(0) && reg < TIMROT_TIMCTRL(3) + 0x40) {
        int n = (reg - 0x20) / 0x40;
        if (reg == TIMROT_TIMCTRL(n)) {
            return s->timctrl[n];
        }
        if (reg == TIMROT_RUNNING(n)) {
            return timrot_count(s, n);
        }
        if (reg == TIMROT_FIXED(n)) {
            return s->fixed_count[n];
        }
        if (reg == TIMROT_MATCH(n)) {
            return s->match_count[n];
        }
    }

    qemu_log_mask(LOG_UNIMP, "timrot: unimplemented read 0x%" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

static void timrot_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    MxsTimrotState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    qemu_log_mask(LOG_TRACE, "timrot: write 0x%03x off %u = 0x%08x\n",
                  (unsigned)reg, off, (uint32_t)value);

    if (reg == 0x00) {
        switch (off) {
        case 0:
            s->rotctrl = value;
            break;
        case 4:
            s->rotctrl |= value;
            break;
        case 8:
            s->rotctrl &= ~value;
            break;
        case 12:
            s->rotctrl ^= value;
            break;
        }
        return;
    }
    if (reg == 0x10) {
        s->rotcount = value;
        return;
    }

    if (reg >= TIMROT_TIMCTRL(0) && reg < TIMROT_TIMCTRL(3) + 0x40) {
        int n = (reg - 0x20) / 0x40;

        if (reg == TIMROT_TIMCTRL(n)) {
            switch (off) {
            case 0:
                s->timctrl[n] = value;
                break;
            case 4:
                s->timctrl[n] |= value;
                break;
            case 8:
                s->timctrl[n] &= ~value;
                /* clearing IRQ deasserts the line */
                if (value & TIMCTRL_IRQ) {
                    qemu_set_irq(s->irq[n], 0);
                }
                break;
            case 12:
                s->timctrl[n] ^= value;
                break;
            }
            timctrl_updated(s, n);
            return;
        }
        if (reg == TIMROT_FIXED(n)) {
            s->fixed_count[n] = value;
            if (!(s->timctrl[n] & TIMCTRL_MATCH_MODE)) {
                s->base_count[n] = value;
                s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            }
            timrot_arm(s, n);
            return;
        }
        if (reg == TIMROT_RUNNING(n)) {
            s->base_count[n] = value;
            s->base_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->counting[n] = timrot_freq(s->timctrl[n]) != 0;
            timrot_arm(s, n);
            return;
        }
        if (reg == TIMROT_MATCH(n)) {
            s->match_count[n] = value;
            timrot_arm(s, n);
            return;
        }
    }

    qemu_log_mask(LOG_UNIMP, "timrot: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, value);
}

static const MemoryRegionOps timrot_ops = {
    .read = timrot_read,
    .write = timrot_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void timrot_reset(DeviceState *dev)
{
    MxsTimrotState *s = MXS_TIMROT(dev);

    s->rotctrl = 0;
    s->rotcount = 0;
    memset(s->timctrl, 0, sizeof(s->timctrl));
    memset(s->fixed_count, 0, sizeof(s->fixed_count));
    memset(s->match_count, 0, sizeof(s->match_count));
    memset(s->base_count, 0, sizeof(s->base_count));
    memset(s->base_ns, 0, sizeof(s->base_ns));
    memset(s->counting, 0, sizeof(s->counting));

    for (int n = 0; n < 4; n++) {
        timer_del(s->tick[n]);
        qemu_set_irq(s->irq[n], 0);
    }
}

static void timrot_init(Object *obj)
{
    MxsTimrotState *s = MXS_TIMROT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &timrot_ops, s,
                          "mxs-timrot", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int n = 0; n < 4; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
    }
}

static void timrot_realize(DeviceState *dev, Error **errp)
{
    MxsTimrotState *s = MXS_TIMROT(dev);

    for (int n = 0; n < 4; n++) {
        timrot_cbs[n].s = s;
        timrot_cbs[n].n = n;
        s->tick[n] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  timrot_expire, &timrot_cbs[n]);
    }
}

static void timrot_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = timrot_realize;
    device_class_set_legacy_reset(dc, timrot_reset);
}

static const TypeInfo timrot_info = {
    .name = TYPE_MXS_TIMROT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsTimrotState),
    .instance_init = timrot_init,
    .class_init = timrot_class_init,
};

static void timrot_register(void)
{
    type_register_static(&timrot_info);
}
type_init(timrot_register)
