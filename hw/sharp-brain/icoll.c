/*
 * i.MX28 ICOLL (interrupt collector) emulation.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Register layout (16-byte "stmp" blocks, REG/SET/CLR/TOG):
 *   0x0000 VECTOR     - read: vector address of current interrupt
 *                       write: latch interrupt number for vectoring
 *   0x0010 LEVELACK   - write: acknowledge interrupt level
 *   0x0020 CTRL
 *   0x0070 STAT       - read: number of highest-priority active IRQ
 *   0x0120 + n*0x10 INTERRUPTn - enable (bit 2) / priority (bits 7:4? kept)
 *
 * The hardware contains a software-programmed vector table; the
 * Windows CE BSP arms it by writing the IRQ number to VECTOR and then
 * reading back the handler address. We keep a 128-entry vector table
 * that can be written by the guest through the VECTOR register
 * after latching an interrupt (observed behaviour on the real
 * machine is that the ROM leaves a default table in IRAM; the
 * BSP installs its own handlers by patching that table, so we
 * redirect vector reads to a table the guest can control through
 * the INTERRUPTn registers' unused upper bits is NOT used - instead
 * the WinCE handler pattern is:
 *     irqnr = readl(STAT); writel(irqnr, VECTOR); fn = readl(VECTOR);
 * and the value read back is the handler address stored in the ICOLL.
 * Freescale's BSP writes handler addresses into the table via
 * VECTOR+0x20+4*n (see HW_ICOLL_VECTOR fields in the WinCE BSP).
 * We implement that alias region as the vector table itself.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "brain.h"

#define ICOLL_VECTOR     0x0000
#define ICOLL_LEVELACK   0x0010
#define ICOLL_CTRL       0x0020
#define ICOLL_STAT       0x0070
#define ICOLL_INTR(n)    (0x0120 + (n) * 0x10)
#define ICOLL_INTR_NREG  ((0x2000 - 0x120) / 0x10)

#define ICOLL_INTR_ENABLE   (1 << 2)
#define ICOLL_INTR_SOFTIRQ  (1 << 1)
#define ICOLL_INTR_FIQ      (1 << 0)

struct MxsIcollState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq cpu_irq;

    uint32_t level;
    uint32_t ctrl;
    uint32_t vector_base;       /* software vector table base (VECTOR reg) */
    uint32_t vector_latch;      /* last IRQ number written to VECTOR */
    uint32_t vectors[ICOLL_INTR_NREG];   /* handler addresses (ROM flow) */
    uint32_t intr[ICOLL_INTR_NREG];      /* enable/prio per IRQ */
    uint32_t pending[ICOLL_INTR_NREG];   /* software-pending state */
    bool asserted[ICOLL_INTR_NREG];      /* level state of input lines */
};

OBJECT_DECLARE_SIMPLE_TYPE(MxsIcollState, MXS_ICOLL)

static void icoll_update(MxsIcollState *s)
{
    int i, active = -1;

    for (i = 0; i < ICOLL_INTR_NREG; i++) {
        if (s->asserted[i] && (s->intr[i] & ICOLL_INTR_ENABLE)) {
            active = i;
            break;
        }
    }

    if (active >= 0) {
        qemu_irq_raise(s->cpu_irq);
    } else {
        qemu_irq_lower(s->cpu_irq);
    }
}

static void icoll_set_irq(void *opaque, int n, int level)
{
    MxsIcollState *s = opaque;

    if (n >= ICOLL_INTR_NREG) {
        return;
    }
    if (!!level != s->asserted[n]) {
        qemu_log_mask(LOG_TRACE, "icoll: irq %d -> %d\n", n, !!level);
    }
    s->asserted[n] = !!level;
    icoll_update(s);
}

static int icoll_current_irq(MxsIcollState *s)
{
    for (int i = 0; i < ICOLL_INTR_NREG; i++) {
        if (s->asserted[i] && (s->intr[i] & ICOLL_INTR_ENABLE)) {
            return i;
        }
    }
    return -1;
}

static uint64_t icoll_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsIcollState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    switch (reg) {
    case ICOLL_VECTOR:
        if (off == 0) {
            /*
             * Hardware vectoring: reading VECTOR returns the address
             * of the vector slot for the highest-priority active
             * interrupt: vector_base + 4 * irq.  The Windows CE
             * dispatcher extracts the IRQ number with
             * (value >> 2) & 0x7f (the base is 0x200-aligned), the
             * ROM-era flow instead reads the handler address back
             * after writing the IRQ number (kept working through
             * vector_latch/vectors[] for compatibility).
             */
            int n = icoll_current_irq(s);

            if (n < 0) {
                n = 0x7f;      /* no interrupt: mirrors STAT */
            } else {
                s->vector_latch = n;
            }
            uint32_t v = s->vector_base + 4 * (uint32_t)n;
            qemu_log_mask(LOG_TRACE, "icoll: VECTOR -> 0x%x (n=%d)\n", v, n);
            return v;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "icoll: unimplemented VECTOR offset 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    case ICOLL_LEVELACK:
        return s->level;
    case ICOLL_CTRL:
        return s->ctrl;
    case ICOLL_STAT:
    {
        int i;
        for (i = 0; i < ICOLL_INTR_NREG; i++) {
            if (s->asserted[i] && (s->intr[i] & ICOLL_INTR_ENABLE)) {
                qemu_log_mask(LOG_TRACE, "icoll: STAT -> %d\n", i);
                return i;
            }
        }
        qemu_log_mask(LOG_TRACE, "icoll: STAT -> none\n");
        return 0x7f;    /* no interrupt */
    }
    default:
        if (reg >= 0x120 && reg < 0x2000 && ((reg - 0x120) % 0x10) == 0) {
            int n = (reg - 0x120) / 0x10;
            return s->intr[n];
        }
        /* vector table alias: HW_ICOLL_VECTOR + 0x20 + 4*n (WinCE BSP) */
        if (reg >= 0x20 && reg < 0x120) {
            int n = (reg - 0x20) / 4;
            if (n < ICOLL_INTR_NREG) {
                return s->vectors[n];
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "icoll: unimplemented read 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void icoll_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    MxsIcollState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (reg >= 0x120) {
        int n = (reg - 0x120) / 0x10;
        if (((reg - 0x120) % 0x10) == 0) {
            qemu_log_mask(LOG_TRACE, "icoll: INTR[%d] %s 0x%08x\n", n,
                          off == 0 ? "REG" : off == 4 ? "SET" :
                          off == 8 ? "CLR" : "TOG", (uint32_t)value);
        }
    } else if (reg >= 0x20 && reg < 0x120) {
        qemu_log_mask(LOG_TRACE, "icoll: alias 0x%03x %s 0x%08x\n",
                      (unsigned)reg,
                      off == 0 ? "REG" : off == 4 ? "SET" :
                      off == 8 ? "CLR" : "TOG", (uint32_t)value);
    } else if (reg == 0) {
        qemu_log_mask(LOG_TRACE, "icoll: VECTOR w 0x%08x\n", (uint32_t)value);
    }

    switch (reg) {
    case ICOLL_VECTOR:
        if (off == 0) {
            /*
             * Writing VECTOR: the ROM-era flow writes the IRQ
             * number to latch it (reading then returns the handler
             * address through vectors[]); an OS can alternatively
             * write a vector-table base, which the read path adds
             * to 4*irq.  Heuristic: small values latch the IRQ
             * number, page-aligned values program the base.
             */
            if (value & ~0x1ffu) {
                s->vector_base = value & ~0x1ffu;
            } else {
                s->vector_latch = value & 0x7f;
            }
            return;
        }
        break;
    case ICOLL_LEVELACK:
        s->level = value;
        qemu_log_mask(LOG_TRACE, "icoll: LEVELACK 0x%x\n", (uint32_t)value);
        icoll_update(s);
        return;
    case ICOLL_CTRL:
        if (off == 0) {
            s->ctrl = value;
        } else if (off == 4) {
            s->ctrl |= value;
        } else if (off == 8) {
            s->ctrl &= ~value;
        } else {
            s->ctrl ^= value;
        }
        return;
    default:
        break;
    }

    if (reg >= 0x120 && reg < 0x2000) {
        int n = (reg - 0x120) / 0x10;
        if (n < ICOLL_INTR_NREG) {
            switch (off) {
            case 0:
                s->intr[n] = value;
                break;
            case 4:
                s->intr[n] |= value;
                break;
            case 8:
                s->intr[n] &= ~value;
                break;
            case 12:
                s->intr[n] ^= value;
                break;
            }
            icoll_update(s);
            return;
        }
    }

    /* vector table alias region */
    if (reg >= 0x20 && reg < 0x120 && (addr & 3) == 0) {
        int n = (reg - 0x20) / 4;
        if (n < ICOLL_INTR_NREG) {
            s->vectors[n] = value;
            return;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "icoll: unimplemented write 0x%" HWADDR_PRIx " = 0x%" PRIx64
                  "\n", addr, value);
}

static const MemoryRegionOps icoll_ops = {
    .read = icoll_read,
    .write = icoll_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void icoll_reset(DeviceState *dev)
{
    MxsIcollState *s = MXS_ICOLL(dev);

    memset(s->asserted, 0, sizeof(s->asserted));
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->intr, 0, sizeof(s->intr));
    memset(s->vectors, 0, sizeof(s->vectors));
    s->ctrl = 0;
    s->level = 0;
    s->vector_latch = 0;
}

static void icoll_init(Object *obj)
{
    MxsIcollState *s = MXS_ICOLL(obj);

    memory_region_init_io(&s->iomem, obj, &icoll_ops, s, "mxs-icoll", 0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cpu_irq);
    qdev_init_gpio_in_named_with_opaque(DEVICE(obj), icoll_set_irq, s,
                                        "irqs", ICOLL_INTR_NREG);
}

static void icoll_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, icoll_reset);
}

static const TypeInfo icoll_info = {
    .name = TYPE_MXS_ICOLL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsIcollState),
    .instance_init = icoll_init,
    .class_init = icoll_class_init,
};

static void icoll_register(void)
{
    type_register_static(&icoll_info);
}
type_init(icoll_register)
