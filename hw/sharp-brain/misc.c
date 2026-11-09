/*
 * i.MX28 misc blocks: DIGCTL, CLKCTRL, POWER, OCOTP, LRADC stubs.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * These are modelled as generic stmp register files (REG/SET/CLR/TOG
 * every 16 bytes) with a few live bits:
 *
 *   - DIGCTL microseconds counter (0xc0) runs at 24 MHz; the chip id
 *     (0x310) identifies an i.MX28 (TO1.2)
 *   - CLKCTRL: PLL0CTRL1/PLL1CTRL1 always report LOCK; SFTRST/CLKGATE
 *     and the CPU/HBUS/EMI busy bits read back as clear so guest
 *     polling loops terminate
 *   - POWER status (0xc0): 5V present, DC-DC converters OK, no
 *     brownouts (DC_OK | VDD5V_GT_VDDIO | PWRUP_SOURCE_5V |
 *     PSWITCH high)
 *   - EMI (DDR controller): the debug register (0xe8) always reports
 *     DLL calibration complete so the boot stage0 init sequence
 *     finishes
 *   - OCOTP: read back as written
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
#define KIND_EMI      4

/* DIGCTL register indexes (offset / 16) */
#define DIGCTL_MICROSECONDS  0x0c
#define DIGCTL_CHIPID        0x31

/* POWER register indexes */
#define POWER_STS            0x0c
#define POWER_BATTMONITOR    0x0e

/*
 * Battery voltage: BATTVAL field (bits 31:16) is in 8 mV steps.
 * Report a healthy Li-ion pack (~3.7 V) so the boot loader's
 * battery-fatal check passes.
 */
#define POWER_BATTMONITOR_BATTVAL_MV   3700

/* EMI register indexes */
#define EMI_DEBUG            0x0e
#define EMI_DEBUG_DLL_LOCK   (1u << 20)

/* POWER_STS bits (i.MX28) */
#define POWER_STS_PWRUP_SOURCE_5V    (0x20u << 24)
#define POWER_STS_PSWITCH_HIGH       (0x2u << 20)
#define POWER_STS_DC_OK              (1u << 9)
#define POWER_STS_VDD5V_GT_VDDIO     (1u << 5)

/* CLKCTRL PLL control 1 LOCK bit */
#define CLKCTRL_PLL_LOCK             (1u << 31)
/* SFTRST / CLKGATE / busy bits that must read back as clear */
#define CLKCTRL_BUSY_BITS            ((0xfu << 28))

static uint64_t mxs_misc_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsMiscState *s = opaque;
    uint32_t idx = addr / 0x10;
    uint32_t v;

    /*
     * stmp register layout: all four 16-byte aliases (REG/SET/CLR/TOG)
     * read back the same value.
     */
    if (idx >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR, "mxs-misc: bad read 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }

    switch (s->kind) {
    case KIND_DIGCTL:
        if (idx == DIGCTL_MICROSECONDS) {
            /* 24 MHz free-running microsecond counter */
            return (uint32_t)muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                      24, 1000);
        }
        if (idx == DIGCTL_CHIPID) {
            /* i.MX28, TO1.2 */
            return 0x28000012;
        }
        return s->regs[idx];
    case KIND_CLKCTRL:
        v = s->regs[idx];
        if (idx == 1 || idx == 3) {
            /* PLL0CTRL1 / PLL1CTRL1: locked */
            v |= CLKCTRL_PLL_LOCK;
        } else {
            /* SFTRST/CLKGATE busy bits read as clear (running) */
            v &= ~CLKCTRL_BUSY_BITS;
        }
        return v;
    case KIND_POWER:
        v = s->regs[idx];
        if (idx == POWER_STS) {
            /* 5V powered, DCDCs happy, no brownout, power switch high */
            v |= POWER_STS_PWRUP_SOURCE_5V | POWER_STS_PSWITCH_HIGH |
                 POWER_STS_DC_OK | POWER_STS_VDD5V_GT_VDDIO;
        }
        if (idx == POWER_BATTMONITOR) {
            v &= 0x0000ffff;
            v |= (POWER_BATTMONITOR_BATTVAL_MV / 8) << 16;
        }
        return v;
    case KIND_EMI:
        v = s->regs[idx];
        if (idx == EMI_DEBUG) {
            /* DDR DLL calibration always reports complete */
            v |= EMI_DEBUG_DLL_LOCK;
        }
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

    uint32_t old = s->regs[idx];

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

    /*
     * stmp soft-reset coupling (all blocks with a CTRL register):
     * on the SFTRST rising edge the block's clock gates, so guest
     * reset sequences ("set SFTRST, poll CLKGATE") terminate.  The
     * guest clears CLKGATE itself after releasing SFTRST.
     */
    if (idx == 0) {
        if (!(old & (1u << 31)) && (s->regs[0] & (1u << 31))) {
            s->regs[0] |= 1u << 30;
        }
    }
}

static const MemoryRegionOps mxs_misc_ops = {
    .read = mxs_misc_read,
    .write = mxs_misc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/*
 * CLKCTRL reset defaults.  The power-on values are not documented in
 * the public datasheet; these follow the "boot ROM + stage0 have run"
 * state: every clock divider field is valid (non-zero) so clock-rate
 * computations in the boot loader never divide by zero.  Registers
 * that stage0/EBOOT program themselves are overwritten at runtime.
 */
static const struct {
    uint32_t idx;
    uint32_t val;
} clkctrl_reset[] = {
    { 0x00, 0x00000000 },   /* PLL0CTRL0 */
    { 0x02, 0x00000000 },   /* PLL1CTRL0 */
    { 0x04, 0x00000000 },   /* PLL2CTRL0 */
    { 0x05, 0x00000001 },   /* CPU      (div 1) */
    { 0x06, 0x00000001 },   /* HBUS     (div 1) */
    { 0x07, 0x00000001 },   /* XBUS     (div 1) */
    { 0x08, 0x00000001 },   /* XTAL     (div 1) */
    { 0x09, 0x00000001 },   /* SSP0     (div 1) */
    { 0x0a, 0x00000001 },   /* SSP1     (div 1) */
    { 0x0b, 0x00000001 },   /* SSP2     (div 1) */
    { 0x0c, 0x00000001 },   /* SSP3     (div 1) */
    { 0x0d, 0x00000001 },   /* GPMI     (div 1) */
    { 0x0e, 0x00000001 },   /* SPDIF    (div 1) */
    { 0x0f, 0x00000001 },   /* EMI      (div 1) */
    { 0x10, 0x00000001 },   /* SAIF0    (div 1) */
    { 0x11, 0x00000001 },   /* SAIF1    (div 1) */
    { 0x12, 0x00000001 },   /* LCDIF    (div 1) */
    { 0x13, 0x00000001 },   /* ETM      (div 1) */
    { 0x14, 0x00000001 },   /* ENET     (div 1) */
    { 0x15, 0x00000001 },   /* HSADC    (div 1) */
    { 0x16, 0x00000001 },   /* FLEXCAN  (div 1) */
};

static void mxs_misc_reset(DeviceState *dev)
{
    MxsMiscState *s = MISC_OF(dev);

    memset(s->regs, 0, sizeof(s->regs));

    if (s->kind == KIND_CLKCTRL) {
        for (unsigned i = 0; i < ARRAY_SIZE(clkctrl_reset); i++) {
            s->regs[clkctrl_reset[i].idx] = clkctrl_reset[i].val;
        }
    }
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
        s->kind = KIND_EMI;
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
        TYPE_MXS_EMI,
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
