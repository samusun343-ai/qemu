/*
 * i.MX28 APBH/APBX DMA emulation.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The mxs DMA engines walk command descriptor chains ("CCW"s) located
 * in guest memory:
 *
 *   struct ccw {
 *       u32 next;          // next descriptor address, 0 = stop
 *       u16 bits;          // COMMAND/CHAIN/IRQ/... flags
 *       u16 xfer_bytes;    // data byte count
 *       u32 bufaddr;       // data buffer address
 *       u32 pio_words[16]; // up to 16 PIO words
 *   };
 *
 * bits:
 *   [1:0]  COMMAND: 0 = PIO (no transfer), 1 = WRITE (dev->mem),
 *                    2 = READ (mem->dev), 3 = sense
 *   [2]    CHAIN
 *   [3]    IRQ
 *   [5]    WAIT4RDY
 *   [6]    DEC_SEM
 *   [7]    WAIT4END
 *   [8]    HALT_ON_TERM
 *   [9]    TERM_FLUSH
 *   [15:12] PIO_NUM
 *
 * A channel starts when software writes the descriptor head to
 * NXTCMDAR and increments the semaphore (SEMA).  PIO descriptors
 * write their words to consecutive 16-byte-spaced registers of the
 * bound peripheral (SSP CTRL0, CMD0, CMD1, ...); completion of the
 * PIO word sequence triggers the peripheral command (this is how
 * the DMA starts the SSP without the CPU touching RUN itself).
 * Data descriptors then move payload between the peripheral and
 * memory.  CCW_IRQ raises the channel IRQ and sets the completion
 * bit in CTRL1; CCW_DEC_SEM decrements the channel semaphore.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "brain.h"

#define DMA_REG_CTRL0         0x000
#define DMA_REG_CTRL1         0x010
#define DMA_REG_CTRL2         0x020
#define DMA_REG_CHANNEL_CTRL  0x030
#define DMA_REG_NXTCMDAR(n)   (0x110 + (n) * 0x70)
#define DMA_REG_BAR(n)        (0x130 + (n) * 0x70)
#define DMA_REG_SEMA(n)       (0x140 + (n) * 0x70)
#define DMA_REG_DEBUG1(n)     (0x150 + (n) * 0x70)

#define CCW_COMMAND(bits)     ((bits) & 3)
#define CCW_CHAIN             (1 << 2)
#define CCW_IRQ               (1 << 3)
#define CCW_WAIT4RDY          (1 << 5)
#define CCW_DEC_SEM           (1 << 6)
#define CCW_WAIT4END          (1 << 7)
#define CCW_HALT_ON_TERM      (1 << 8)
#define CCW_TERM_FLUSH        (1 << 9)
#define CCW_PIO_NUM(bits)     (((bits) >> 12) & 0xf)

static void mxs_dma_irq_update(MxsDmaState *s, int ch)
{
    qemu_set_irq(s->irq[ch], !!(s->ctrl1 & (1 << ch)));
}

static void mxs_dma_set_error(MxsDmaState *s, int ch, uint32_t bits)
{
    s->ctrl2 |= bits;
    qemu_log_mask(LOG_GUEST_ERROR, "mxs-dma: channel %d error 0x%x\n",
                  ch, bits);
}

/* Read a CCW descriptor from guest memory. */
static bool mxs_dma_read_desc(dma_addr_t addr,
                              uint32_t *next, uint32_t *bits,
                              uint32_t *xfer, uint32_t *bufaddr,
                              uint32_t *pio, unsigned pio_max)
{
    struct {
        uint32_t next;
        uint32_t bits_xfer;    /* u16 bits + u16 xfer_bytes */
        uint32_t bufaddr;
    } hdr;

    if (address_space_read(&address_space_memory, addr,
                           MEMTXATTRS_UNSPECIFIED, &hdr,
                           sizeof(hdr)) != MEMTX_OK) {
        return false;
    }
    *next = le32_to_cpu(hdr.next);
    *bits = le16_to_cpu(hdr.bits_xfer & 0xffff);
    *xfer = le16_to_cpu(hdr.bits_xfer >> 16);
    *bufaddr = le32_to_cpu(hdr.bufaddr);

    unsigned n = CCW_PIO_NUM(*bits);
    if (n > pio_max) {
        n = pio_max;
    }
    if (n) {
        if (address_space_read(&address_space_memory, addr + 12,
                               MEMTXATTRS_UNSPECIFIED, pio,
                               n * 4) != MEMTX_OK) {
            return false;
        }
        for (unsigned i = 0; i < n; i++) {
            pio[i] = le32_to_cpu(pio[i]);
        }
    }
    return true;
}

/*
 * Run one channel until the descriptor chain stops.  Each step is
 * executed synchronously: PIO words are poked into the peripheral,
 * data descriptors call the peripheral's data hook.
 */
static void mxs_dma_run_channel(MxsDmaState *s, int ch)
{
    dma_addr_t cur = s->nxctmdar[ch];
    int steps = 0;

    while (cur && steps++ < 1024) {
        uint32_t next, bits, xfer, bufaddr;
        uint32_t pio[16] = { 0 };
        MxsSspState *ssp = (ch < 4) ? s->ssp[ch] : NULL;

        if (!mxs_dma_read_desc(cur, &next, &bits, &xfer, &bufaddr,
                               pio, ARRAY_SIZE(pio))) {
            mxs_dma_set_error(s, ch, 1 << ch);  /* AHB bus error */
            return;
        }

        unsigned pio_num = CCW_PIO_NUM(bits);

        switch (CCW_COMMAND(bits)) {
        case 0: /* PIO: write words into the bound peripheral */
            if (ch < 4 && ssp) {
                for (unsigned i = 0; i < pio_num; i++) {
                    mxs_ssp_dma_pio(ssp, i, pio[i]);
                }
                /* the peripheral command is implicitly started after
                 * the PIO word sequence (hardware behaviour) */
                mxs_ssp_dma_pio(ssp, -1, 0);
            } else {
                qemu_log_mask(LOG_UNIMP,
                              "mxs-dma: PIO descriptor on clientless "
                              "channel %d\n", ch);
            }
            break;
        case 1: /* WRITE: peripheral -> memory */
            if (ch < 4 && ssp) {
                mxs_ssp_dma_data(ssp, false, bufaddr, xfer);
            } else if (ch == 13 && s->lcdif) {
                mxs_lcdif_dma_data(s->lcdif, false, bufaddr, xfer);
            } else {
                qemu_log_mask(LOG_UNIMP,
                              "mxs-dma: WRITE on clientless channel %d\n",
                              ch);
            }
            break;
        case 2: /* READ: memory -> peripheral */
            if (ch < 4 && ssp) {
                mxs_ssp_dma_data(ssp, true, bufaddr, xfer);
            } else if (ch == 13 && s->lcdif) {
                mxs_lcdif_dma_data(s->lcdif, true, bufaddr, xfer);
            } else {
                qemu_log_mask(LOG_UNIMP,
                              "mxs-dma: READ on clientless channel %d\n",
                              ch);
            }
            break;
        default:
            break;
        }

        if (bits & CCW_IRQ) {
            s->ctrl1 |= 1 << ch;
            mxs_dma_irq_update(s, ch);
        }

        if (bits & CCW_DEC_SEM) {
            if (s->sema[ch]) {
                s->sema[ch]--;
            }
        }

        if (bits & CCW_CHAIN) {
            cur = next;
        } else {
            /* remember the current BAR for progress queries */
            s->bar[ch] = bufaddr + xfer;
            cur = 0;
        }
    }

    if (steps >= 1024) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mxs-dma: channel %d descriptor chain too long\n", ch);
    }
}

static uint64_t mxs_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsDmaState *s = opaque;

    switch (addr) {
    case DMA_REG_CTRL0:
        return s->ctrl0;
    case DMA_REG_CTRL1:
        return s->ctrl1;
    case DMA_REG_CTRL2:
        return s->ctrl2;
    case DMA_REG_CHANNEL_CTRL:
        return s->channel_ctrl;
    default:
        break;
    }

    if (addr >= DMA_REG_NXTCMDAR(0) &&
        addr < DMA_REG_NXTCMDAR(MXS_DMA_CHANNELS)) {
        int ch = (addr - 0x110) / 0x70;
        uint32_t off = (addr - 0x110) % 0x70;
        switch (off) {
        case 0x00:
            return s->nxctmdar[ch];
        case 0x20:
            return s->bar[ch];
        case 0x30:
            return s->sema[ch];
        case 0x40:
            return 0;   /* DEBUG1 */
        default:
            return 0;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "mxs-dma: unimplemented read 0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void mxs_dma_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    MxsDmaState *s = opaque;
    uint32_t off;

    switch (addr) {
    case DMA_REG_CTRL0:
    case DMA_REG_CTRL0 + 4:
    case DMA_REG_CTRL0 + 8:
    case DMA_REG_CTRL0 + 12:
    {
        uint32_t old = s->ctrl0;

        off = addr & 0xf;
        if (off == 0) {
            s->ctrl0 = value;
        } else if (off == 4) {
            s->ctrl0 |= value;
        } else if (off == 8) {
            s->ctrl0 &= ~value;
        } else {
            s->ctrl0 ^= value;
        }
        /*
         * When SFTRST is asserted the block enters reset and its
         * clock gates; boot loaders poll for CLKGATE reading back
         * set before releasing the block.  Model this as a one-shot
         * on the SFTRST rising edge so an explicit CLKGATE clear
         * (issued while SFTRST is still asserted) sticks.
         */
        if (!(old & (1u << 31)) && (s->ctrl0 & (1u << 31))) {
            s->ctrl0 |= 1u << 30;
        }
        return;
    }
    case DMA_REG_CTRL1:
    case DMA_REG_CTRL1 + 4:
    case DMA_REG_CTRL1 + 8:
    case DMA_REG_CTRL1 + 12:
        off = addr & 0xf;
        if (off == 8) {
            /* writing to CTRL1 CLR clears the completion/error bits */
            s->ctrl1 &= ~value;
            s->ctrl2 &= ~value;
            for (int i = 0; i < MXS_DMA_CHANNELS; i++) {
                mxs_dma_irq_update(s, i);
            }
        }
        return;
    case DMA_REG_CTRL2:
    case DMA_REG_CTRL2 + 8:
        off = addr & 0xf;
        if (off == 8) {
            s->ctrl2 &= ~value;
        }
        return;
    case DMA_REG_CHANNEL_CTRL:
    case DMA_REG_CHANNEL_CTRL + 4:
    case DMA_REG_CHANNEL_CTRL + 8:
        off = addr & 0xf;
        if (off == 0) {
            s->channel_ctrl = value;
        } else if (off == 4) {
            s->channel_ctrl |= value;
        } else if (off == 8) {
            s->channel_ctrl &= ~value;
        }
        return;
    default:
        break;
    }

    if (addr >= DMA_REG_NXTCMDAR(0) &&
        addr < DMA_REG_NXTCMDAR(MXS_DMA_CHANNELS)) {
        int ch = (addr - 0x110) / 0x70;
        off = (addr - 0x110) % 0x70;
        switch (off) {
        case 0x00:  /* NXTCMDAR */
            s->nxctmdar[ch] = value;
            return;
        case 0x30:  /* SEMA: PHORE field increments and starts the channel */
        {
            uint32_t phore = value & 0xff;
            if (phore) {
                s->sema[ch] = (s->sema[ch] & 0xff000000) | phore;
                mxs_dma_run_channel(s, ch);
            } else {
                s->sema[ch] = value;
            }
            return;
        }
        default:
            return;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "mxs-dma: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, value);
}

static const MemoryRegionOps mxs_dma_ops = {
    .read = mxs_dma_read,
    .write = mxs_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mxs_dma_reset(DeviceState *dev)
{
    MxsDmaState *s = MXS_DMA(dev);

    s->ctrl0 = 0;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->channel_ctrl = 0;
    memset(s->nxctmdar, 0, sizeof(s->nxctmdar));
    memset(s->sema, 0, sizeof(s->sema));
    memset(s->bar, 0, sizeof(s->bar));
}

static void mxs_dma_init(Object *obj)
{
    MxsDmaState *s = MXS_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_dma_ops, s,
                          "mxs-dma", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < MXS_DMA_CHANNELS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void mxs_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_dma_reset);
}

static const TypeInfo mxs_dma_info = {
    .name = TYPE_MXS_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsDmaState),
    .instance_init = mxs_dma_init,
    .class_init = mxs_dma_class_init,
};

static void mxs_dma_register(void)
{
    type_register_static(&mxs_dma_info);
}
type_init(mxs_dma_register)
