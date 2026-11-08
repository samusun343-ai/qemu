/*
 * i.MX28 TIMROT emulation.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Four 32-bit timers counting down from FIXED_COUNT at 24 MHz
 * (SELECT=TICK_ALWAYS) or 32 kHz / 8 kHz / 100 Hz / 1 kHz / 1 Hz
 * variants.  IRQ on reaching zero (IRQ_EN), RELOAD mode restarts.
 *
 * Layout: ROTCTRL 0x00, ROTCOUNT 0x10, then per timer n (0x40 stride):
 *   TIMCTRLn      0x20 + n*0x40
 *   RUNNING_COUNT 0x30 + n*0x40
 *   FIXED_COUNT   0x40 + n*0x40
 *   MATCH_COUNT   0x50 + n*0x40
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/timer.h"
#include "hw/core/ptimer.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "brain.h"

#define TIMROT_TIMCTRL(n)     (0x20 + (n) * 0x40)
#define TIMROT_RUNNING(n)     (0x30 + (n) * 0x40)
#define TIMROT_FIXED(n)       (0x40 + (n) * 0x40)
#define TIMROT_MATCH(n)       (0x50 + (n) * 0x40)

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

typedef struct TimrotCb {
    MxsTimrotState *s;
    int n;
} TimrotCb;

typedef struct MxsTimrotState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[4];

    uint32_t rotctrl, rotcount;
    uint32_t timctrl[4];
    uint32_t fixed_count[4];
    uint32_t match_count[4];

    ptimer_state *timer[4];
    TimrotCb cb[4];
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

static void timrot_timer_expire(void *opaque)
{
    TimrotCb *cb = opaque;
    MxsTimrotState *s = cb->s;
    int n = cb->n;

    if (ptimer_get_count(s->timer[n]) == 0 &&
        (s->timctrl[n] & TIMCTRL_IRQ_EN)) {
        s->timctrl[n] |= TIMCTRL_IRQ;
        qemu_set_irq(s->irq[n], 1);
    }
}

static void timctrl_updated(MxsTimrotState *s, int n)
{
    uint64_t freq = timrot_freq(s->timctrl[n]);

    ptimer_transaction_begin(s->timer[n]);
    if (freq == 0) {
        ptimer_stop(s->timer[n]);
    } else {
        ptimer_set_freq(s->timer[n], freq);
        uint32_t cnt = ptimer_get_count(s->timer[n]);
        if (cnt == 0) {
            cnt = s->fixed_count[n];
        }
        if (s->timctrl[n] & TIMCTRL_RELOAD) {
            ptimer_set_count(s->timer[n], s->fixed_count[n]);
            ptimer_set_limit(s->timer[n], s->fixed_count[n], 1);
        } else {
            ptimer_set_count(s->timer[n], cnt);
        }
        ptimer_run(s->timer[n], 0);
    }
    ptimer_transaction_commit(s->timer[n]);
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
    if (reg >= 0x150 && reg < 0x2000) {
        return (2 << 24);       /* major version 2 (i.MX28) */
    }

    if (reg >= TIMROT_TIMCTRL(0) && reg < TIMROT_TIMCTRL(3) + 0x40) {
        int n = (reg - 0x20) / 0x40;
        if (reg == TIMROT_TIMCTRL(n)) {
            return s->timctrl[n];
        }
        if (reg == TIMROT_RUNNING(n)) {
            return ptimer_get_count(s->timer[n]);
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

    if (reg == 0x00) {
        if (off == 0) {
            s->rotctrl = value;
        } else if (off == 4) {
            s->rotctrl |= value;
        } else if (off == 8) {
            s->rotctrl &= ~value;
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
            ptimer_transaction_begin(s->timer[n]);
            ptimer_set_count(s->timer[n], value);
            if (s->timctrl[n] & TIMCTRL_RELOAD) {
                ptimer_set_limit(s->timer[n], value, 1);
            }
            ptimer_transaction_commit(s->timer[n]);
            return;
        }
        if (reg == TIMROT_RUNNING(n)) {
            ptimer_transaction_begin(s->timer[n]);
            ptimer_set_count(s->timer[n], value);
            ptimer_transaction_commit(s->timer[n]);
            return;
        }
        if (reg == TIMROT_MATCH(n)) {
            s->match_count[n] = value;
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

    for (int n = 0; n < 4; n++) {
        ptimer_transaction_begin(s->timer[n]);
        ptimer_stop(s->timer[n]);
        ptimer_set_count(s->timer[n], 0);
        ptimer_transaction_commit(s->timer[n]);
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
        s->cb[n].s = s;
        s->cb[n].n = n;
        s->timer[n] = ptimer_init(timrot_timer_expire, &s->cb[n],
                                  PTIMER_POLICY_LEGACY);
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
