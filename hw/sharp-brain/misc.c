/*
 * i.MX28 misc blocks: DIGCTL, CLKCTRL, POWER, OCOTP stubs.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * These are modelled as generic stmp register files (REG/SET/CLR/TOG
 * every 16 bytes) with a few live bits:
 *   - DIGCTL microseconds counter (0xc0) runs at 24 MHz
 *   - CLKCTRL busy/lock bits always report "ready" so guest polling
 *     loops terminate; SFTRST/CLKGATE bits read back as cleared
 *   - POWER reports 5V present / battery okay
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "brain.h"

typedef struct MxsMiscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[0x2000 / 16];
    uint8_t kind;               /* which block */
} MxsMiscState;

/*
 * All the mxs-misc family types ("mxs-digctl", "mxs-clkctrl", ...) share
 * this instance layout, so a direct cast is safe and avoids needing a
 * common abstract type name.
 */
static inline MxsMiscState *MISC_OF(void *obj)
{
    return (MxsMiscState *)obj;
}

#define KIND_DIGCTL   0
#define KIND_CLKCTRL  1
#define KIND_POWER    2
#define KIND_OCOTP    3
#define KIND_LRADC    4

#define DIGCTL_MICROSECONDS_IDX  (0xc0 / 0x10)

static uint64_t mxs_misc_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsMiscState *s = opaque;
    uint32_t idx = addr / 0x10;
    uint32_t off = addr & 0xf;
    uint32_t v;

    if (off != 0 || idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mxs-misc: bad read 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }

    switch (s->kind) {
    case KIND_DIGCTL:
        if (idx == DIGCTL_MICROSECONDS_IDX) {
            /* 24 MHz free-running microsecond counter */
            return (uint32_t)muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                      24, 1000);
        }
        return s->regs[idx];
    case KIND_CLKCTRL:
        v = s->regs[idx];
        if (idx == 0) {
            v |= (1u << 31);   /* PLLCTRL0: locked */
        } else {
            /* SFTRST/CLKGATE busy bits read as clear (running) */
            v &= ~((1u << 31) | (1u << 30));
        }
        return v;
    case KIND_POWER:
        v = s->regs[idx];
        /* 5V present, battery okay, DCDCs happy */
        v |= (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7);
        return v;
    default:
        return s->regs[idx];
    }
}

static void mxs_misc_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    MxsMiscState *s = opaque;
    uint32_t idx = addr / 0x10;
    uint32_t off = addr & 0xf;

    if (idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mxs-misc: bad write 0x%" HWADDR_PRIx
                      "\n", addr);
        return;
    }

    switch (off) {
    case 0:
        s->regs[idx] = value;
        break;
    case 4:
        s->regs[idx] |= value;
        break;
    case 8:
        s->regs[idx] &= ~value;
        break;
    case 12:
        s->regs[idx] ^= value;
        break;
    }
}

static const MemoryRegionOps mxs_misc_ops = {
    .read = mxs_misc_read,
    .write = mxs_misc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mxs_misc_reset(DeviceState *dev)
{
    MxsMiscState *s = MISC_OF(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void mxs_misc_init(Object *obj)
{
    MxsMiscState *s = MISC_OF(obj);
    const char *t = object_get_typename(obj);

    if (strcmp(t, TYPE_MXS_DIGCTL) == 0) {
        s->kind = KIND_DIGCTL;
    } else if (strcmp(t, TYPE_MXS_CLKCTRL) == 0) {
        s->kind = KIND_CLKCTRL;
    } else if (strcmp(t, TYPE_MXS_POWER) == 0) {
        s->kind = KIND_POWER;
    } else if (strcmp(t, TYPE_MXS_OCOTP) == 0) {
        s->kind = KIND_OCOTP;
    } else {
        s->kind = KIND_LRADC;
    }

    memory_region_init_io(&s->iomem, obj, &mxs_misc_ops, s,
                          "mxs-misc", 0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void mxs_misc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_misc_reset);
}

static void mxs_misc_register_types(void)
{
    const char *names[] = {
        TYPE_MXS_DIGCTL, TYPE_MXS_CLKCTRL, TYPE_MXS_POWER, TYPE_MXS_OCOTP,
        TYPE_MXS_LRADC,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(names); i++) {
        TypeInfo ti = {
            .name = names[i],
            .parent = TYPE_SYS_BUS_DEVICE,
            .instance_size = sizeof(MxsMiscState),
            .instance_init = mxs_misc_init,
            .class_init = mxs_misc_class_init,
        };
        type_register_static(&ti);
    }
}

type_init(mxs_misc_register_types)
