/*
 * i.MX28 USB OTG controller (USB0) register stub.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The i.MX28 USB0 block at 0x80080000 is the ChipIdea (ARC) USB OTG
 * core.  The SHARP Brain has no USB cable attached in normal use, so
 * this is a register-level stub, NOT a USB host/device stack:
 *
 *   - Control registers read back as written (USBCMD, USBINTR,
 *     USBMODE, ENDPTCTRL0-15, ENDPTNAKEN, SBUSCFG, BURSTSIZE,
 *     TTCTRL, DEVICEADDR, ENDPTLISTADDR, GPTIMERs).  TXFILLTUNING
 *     is the exception: it reads 0 (see the note in
 *     usb_otg_reg_value).
 *   - Capability registers report the core's documented reset
 *     values (CAPLENGTH 0x20, HCIVERSION 0x0100, HCSPARAMS 1 port,
 *     DCCPARAMS 16 endpoints, device+host capable).
 *   - Status registers always read 0: no VBUS / no session (OTGSC),
 *     no device connected (PORTSC1 CCS/PE clear), no interrupts
 *     pending (USBSTS), no setup packets / NAKs / completions, and
 *     ENDPTPRIME / ENDPTFLUSH complete instantly.  That is exactly
 *     the "no cable" state, so the WinCE USB function driver
 *     (UfnPdd) proceeds down its normal "USB is not detected" path.
 *   - The single IRQ output (ICOLL 93, linux imx28.dtsi) is wired
 *     but never raised: with no cable there is no event to signal.
 *     (An earlier hang at CSP_Init was first blamed on this IRQ
 *     line, but a bisect proved it was the TXFILLTUNING poll
 *     below; the IRQ connection is innocent.)
 *   - Self-clearing command bits do not stick: USBCMD RST/SUTW/
 *     ATDTW and ENDPTCTRL TXR/RXR are cleared on write, and
 *     ENDPTCTRL reads back only the persistent R/W bits (TXE/TXT/
 *     TXS/RXE/RXT/RXS, mask 0x008D008D) -- on real hardware a driver
 *     that sets a reset bit polls for it to clear, and a stuck bit
 *     would hang it (linux-brain drivers/usb/chipidea/bits.h for
 *     the bit numbers).
 *
 * The guest's boot-time accesses (TXFILLTUNING 0x140, ENDPTNAK
 * 0x144, ENDPTNAKEN 0x148, PORTSC1 0x150, OTGSC 0x160, USBMODE
 * 0x164, ENDPTFLUSH 0x170, ENDPTCTRL2 0x184, ENDPTCTRL10/11
 * 0x1a4/0x1a8, SBUSCFG 0x90) are all covered by this map; the
 * register block itself is 0x10000 bytes (imx28.dtsi) but only the
 * low 0x400 bytes hold registers.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "brain.h"

/* register offsets (ChipIdea USB OTG core) */
#define USB_ID             0x000
#define USB_HWGENERAL      0x004
#define USB_HWHOST         0x008
#define USB_HWDEVICE       0x00c
#define USB_HWTXBUF        0x010
#define USB_HWRXBUF        0x014
#define USB_GPTIMER0LD     0x080
#define USB_GPTIMER0CTL    0x084
#define USB_GPTIMER1LD     0x088
#define USB_GPTIMER1CTL    0x08c
#define USB_SBUSCFG        0x090
#define USB_CAPLENGTH      0x100    /* + HCIVERSION at 0x102 */
#define USB_HCSPARAMS      0x104
#define USB_HCCPARAMS      0x108
#define USB_DCIVERSION     0x110
#define USB_DCCPARAMS      0x114
#define USB_USBCMD         0x120
#define USB_USBSTS         0x124
#define USB_USBINTR        0x128
#define USB_FRINDEX        0x12c
#define USB_DEVICEADDR     0x130
#define USB_ENDPTLISTADDR  0x134
#define USB_TTCTRL         0x138
#define USB_BURSTSIZE      0x13c
#define USB_TXFILLTUNING   0x140
#define USB_ENDPTNAK       0x144
#define USB_ENDPTNAKEN     0x148
#define USB_PORTSC1        0x150
#define USB_OTGSC          0x160
#define USB_USBMODE        0x164
#define USB_ENDPTSETUPSTAT 0x168
#define USB_ENDPTPRIME     0x16c
#define USB_ENDPTFLUSH     0x170
#define USB_ENDPTSTAT      0x174
#define USB_ENDPTCOMPLETE  0x178
#define USB_ENDPTCTRL(n)   (0x17c + (n) * 4)

/* PORTSC1 bits kept on read (writable config); CCS/PE/SUSP and the
 * line-status bits are forced 0: no device is connected. */
#define PORTSC_READ_MASK   0x0003f000u

/* USBCMD self-clearing bits (ChipIdea bits.h): RST = reset,
 * SUTW/ATDTW = tripwires. */
#define USBCMD_SELF_CLEAR  (BIT(1) | BIT(13) | BIT(14))

/* ENDPTCTRL persistent R/W bits read back: TXE, TXT, TXS, RXE, RXT,
 * RXS.  TXR/RXR (data-toggle reset, self-clearing) and the
 * reserved TXI/RXI bits never read as set. */
#define ENDPTCTRL_READ_MASK   0x008d008du
#define ENDPTCTRL_SELF_CLEAR  (BIT(22) | BIT(6))

/* capability-register reset values (ChipIdea core) */
#define CAPLENGTH_HCIVERSION  0x01000020u  /* CAPLENGTH 0x20, EHCI 1.0 */
#define HCSPARAMS_VALUE        0x00010011u  /* 1 port */
#define DCIVERSION_VALUE       0x00000100u  /* DCI version 1.0 */
#define DCCPARAMS_VALUE        0x00000070u  /* 16 endpoints,
                                              * device + host capable */

#define USB_OTG_NREGS   (0x400 / 4)

typedef struct MxsUsbOtgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[USB_OTG_NREGS];
} MxsUsbOtgState;

OBJECT_DECLARE_SIMPLE_TYPE(MxsUsbOtgState, MXS_USB_OTG)

/*
 * 32-bit value of a register for a read.  Control/capability
 * registers come from the stored array (or the fixed reset value);
 * status registers are sensor-driven and read 0 with no cable.
 */
static uint32_t usb_otg_reg_value(MxsUsbOtgState *s, uint32_t off)
{
    uint32_t idx = off / 4;

    if (off >= USB_ENDPTCTRL(0) && off <= USB_ENDPTCTRL(15)) {
        /* data-toggle reset bits (TXR/RXR) and reserved bits never
         * read as set; only the persistent R/W bits are visible */
        return s->regs[idx] & ENDPTCTRL_READ_MASK;
    }

    switch (off) {
    case USB_CAPLENGTH:
        return CAPLENGTH_HCIVERSION;
    case USB_HCSPARAMS:
        return HCSPARAMS_VALUE;
    case USB_HCCPARAMS:
        return 0;
    case USB_DCIVERSION:
        return DCIVERSION_VALUE;
    case USB_DCCPARAMS:
        return DCCPARAMS_VALUE;
    case USB_USBSTS:
    case USB_FRINDEX:
    case USB_ENDPTNAK:
    case USB_OTGSC:
    case USB_ENDPTSETUPSTAT:
    case USB_ENDPTPRIME:
    case USB_ENDPTFLUSH:
    case USB_ENDPTSTAT:
    case USB_ENDPTCOMPLETE:
        /* status: no events, no session, no completions */
        return 0;
    case USB_TXFILLTUNING:
        /*
         * Tuning register: the WinCE driver writes a value here
         * (2, at boot) and then polls the register until it reads
         * 0 (observed in boottest57: ~200 consecutive reads of
         * 0x140 after a write of 2).  A read-as-written model
         * hangs the driver in that poll.  The unimplemented device
         * (which always read 0) let the driver proceed, so the
         * register is modelled as write-accepted / reads-0: the
         * tuning value is applied to the TX scheduler and does not
         * read back.
         */
        return 0;
    case USB_PORTSC1:
        /* writable config bits only; CCS/PE/SUSP read 0 */
        return s->regs[idx] & PORTSC_READ_MASK;
    default:
        return s->regs[idx];
    }
}

static uint64_t usb_otg_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsUsbOtgState *s = opaque;
    uint32_t off = addr & ~3u;
    uint32_t byte_off = addr & 3u;
    uint32_t v;

    if (off >= USB_OTG_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "mxs-usb-otg: bad read 0x%"
                      HWADDR_PRIx "\n", addr);
        return 0;
    }

    v = usb_otg_reg_value(s, off);
    /* sub-word read: extract the requested bytes, little-endian */
    v >>= 8 * byte_off;
    if (size < 4) {
        v &= (1u << (8 * size)) - 1;
    }
    return v;
}

static void usb_otg_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    MxsUsbOtgState *s = opaque;
    uint32_t off = addr & ~3u;
    uint32_t byte_off = addr & 3u;
    uint32_t idx = off / 4;

    if (off >= USB_OTG_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "mxs-usb-otg: bad write 0x%"
                      HWADDR_PRIx " = 0x%" PRIx64 "\n", addr, value);
        return;
    }

    switch (off) {
    case USB_CAPLENGTH:
    case USB_HCSPARAMS:
    case USB_HCCPARAMS:
    case USB_DCIVERSION:
    case USB_DCCPARAMS:
        /* capability registers are read-only */
        return;
    case USB_USBSTS:
    case USB_ENDPTNAK:
    case USB_OTGSC:
    case USB_ENDPTSETUPSTAT:
    case USB_ENDPTCOMPLETE:
        /* status registers: write-1-clear on the real core; all
         * bits already read 0 with no cable, so nothing to do */
        return;
    case USB_ENDPTPRIME:
    case USB_ENDPTFLUSH:
        /* prime/flush complete instantly; the bits read back 0 */
        return;
    case USB_TXFILLTUNING:
        /* tuning value is applied, not stored; the register reads 0
         * (see usb_otg_reg_value) */
        return;
    case USB_FRINDEX:
    case USB_ENDPTSTAT:
        /* read-only status */
        return;
    default:
        break;
    }

    /* stored control register: merge the written bytes in */
    if (size == 4 && byte_off == 0) {
        s->regs[idx] = value;
    } else {
        uint32_t mask = ((1u << (8 * size)) - 1) << (8 * byte_off);

        s->regs[idx] = (s->regs[idx] & ~mask) |
                       ((uint32_t)value << (8 * byte_off));
    }

    /* self-clearing command bits never stick: on real hardware a
     * driver that sets one polls for it to clear */
    if (off == USB_USBCMD) {
        s->regs[idx] &= ~USBCMD_SELF_CLEAR;
    } else if (off >= USB_ENDPTCTRL(0) && off <= USB_ENDPTCTRL(15)) {
        s->regs[idx] &= ~ENDPTCTRL_SELF_CLEAR;
    }
}

static const MemoryRegionOps usb_otg_ops = {
    .read = usb_otg_read,
    .write = usb_otg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void usb_otg_reset(DeviceState *dev)
{
    MxsUsbOtgState *s = MXS_USB_OTG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
}

static void usb_otg_init(Object *obj)
{
    MxsUsbOtgState *s = MXS_USB_OTG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &usb_otg_ops, s,
                          "mxs-usb-otg", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void usb_otg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "i.MX28 USB OTG controller (USB0)";
    device_class_set_legacy_reset(dc, usb_otg_reset);
}

static const TypeInfo usb_otg_info = {
    .name = TYPE_MXS_USB_OTG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsUsbOtgState),
    .instance_init = usb_otg_init,
    .class_init = usb_otg_class_init,
};

static void usb_otg_register(void)
{
    type_register_static(&usb_otg_info);
}
type_init(usb_otg_register)
