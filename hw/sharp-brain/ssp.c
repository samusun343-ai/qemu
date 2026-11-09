/*
 * i.MX28 SSP (Synchronous Serial Port) emulation - SD/MMC mode.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The SSP block is i.MX28's SD/MMC host controller (SSP0: 8-bit eMMC,
 * SSP1: 4-bit SD card on the Brain).  Command execution is started
 * either by a CPU write to CTRL0 with RUN set, or implicitly by the
 * APBH DMA after a PIO descriptor writes the CTRL0/CMD0/CMD1
 * registers.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "brain.h"

/* 16-byte spaced register offsets */
#define SSP_CTRL0       0x000
#define SSP_CMD0        0x010
#define SSP_CMD1        0x020
#define SSP_XFER_SIZE   0x030
#define SSP_BLOCK_SIZE  0x040
#define SSP_COMPREF     0x050
#define SSP_COMPMASK    0x060
#define SSP_TIMING      0x070
#define SSP_CTRL1       0x080
#define SSP_DATA        0x090
#define SSP_SDRESP(n)   (0x0a0 + (n) * 0x10)
#define SSP_DDR_CTRL    0x0e0
#define SSP_DLL_CTRL    0x0f0
#define SSP_STATUS      0x100
#define SSP_DLL_STS     0x110
#define SSP_DEBUG       0x120
#define SSP_VERSION     0x130

#define CTRL0_SFTRST        (1 << 31)
#define CTRL0_CLKGATE       (1 << 30)
#define CTRL0_RUN           (1 << 29)
#define CTRL0_IGNORE_CRC    (1 << 26)
#define CTRL0_READ          (1 << 25)
#define CTRL0_DATA_XFER     (1 << 24)
#define CTRL0_BUS_WIDTH     (3 << 22)
#define CTRL0_WAIT_FOR_IRQ  (1 << 21)
#define CTRL0_LONG_RESP     (1 << 19)
#define CTRL0_CHECK_RESP    (1 << 18)
#define CTRL0_GET_RESP      (1 << 17)
#define CTRL0_ENABLE        (1 << 16)

/*
 * CTRL1 (i.MX28 layout, u-boot regs-ssp.h): the IRQ and IRQ_ENABLE
 * bits are interleaved (IRQ at bit 2n+1, enable at bit 2n), NOT two
 * separate status/enable bytes:
 *   31 SDIO_IRQ            30 SDIO_IRQ_EN
 *   29 RESP_ERR_IRQ        28 RESP_ERR_IRQ_EN
 *   27 RESP_TIMEOUT_IRQ    26 RESP_TIMEOUT_IRQ_EN
 *   25 DATA_TIMEOUT_IRQ    24 DATA_TIMEOUT_IRQ_EN
 *   23 DATA_CRC_IRQ        22 DATA_CRC_IRQ_EN
 *   21 FIFO_UNDERRUN_IRQ   20 FIFO_UNDERRUN_EN
 *   19 CEATA_CCS_ERR_IRQ   18 CEATA_CCS_ERR_IRQ_EN
 *   17 RECV_TIMEOUT_IRQ    16 RECV_TIMEOUT_IRQ_EN
 *   15 FIFO_OVERRUN_IRQ    14 FIFO_OVERRUN_IRQ_EN
 *   13 DMA_ENABLE
 */
#define CTRL1_SDIO_IRQ            (1u << 31)
#define CTRL1_SDIO_IRQ_EN         (1u << 30)
#define CTRL1_RESP_ERR_IRQ        (1u << 29)
#define CTRL1_RESP_ERR_IRQ_EN     (1u << 28)
#define CTRL1_RESP_TIMEOUT_IRQ    (1u << 27)
#define CTRL1_RESP_TIMEOUT_IRQ_EN (1u << 26)
#define CTRL1_DATA_TIMEOUT_IRQ    (1u << 25)
#define CTRL1_DATA_TIMEOUT_IRQ_EN (1u << 24)
#define CTRL1_DATA_CRC_IRQ        (1u << 23)
#define CTRL1_DATA_CRC_IRQ_EN     (1u << 22)
#define CTRL1_FIFO_UNDERRUN_IRQ   (1u << 21)
#define CTRL1_FIFO_UNDERRUN_EN    (1u << 20)
#define CTRL1_CEATA_CCS_ERR_IRQ   (1u << 19)
#define CTRL1_CEATA_CCS_ERR_IRQ_EN (1u << 18)
#define CTRL1_RECV_TIMEOUT_IRQ    (1u << 17)
#define CTRL1_RECV_TIMEOUT_IRQ_EN (1u << 16)
#define CTRL1_FIFO_OVERRUN_IRQ    (1u << 15)
#define CTRL1_FIFO_OVERRUN_IRQ_EN (1u << 14)
#define CTRL1_DMA_ENABLE          (1u << 13)

#define STATUS_PRESENT          (1 << 31)
#define STATUS_SD_PRESENT       (1 << 29)
#define STATUS_CARD_DETECT      (1 << 28)
#define STATUS_DMAREQ           (1 << 19)
#define STATUS_DMAEND           (1 << 18)
#define STATUS_RESP_CRC_ERR     (1 << 16)
#define STATUS_RESP_ERR         (1 << 15)
#define STATUS_RESP_TIMEOUT     (1 << 14)
#define STATUS_DATA_CRC_ERR     (1 << 13)
#define STATUS_TIMEOUT          (1 << 12)
#define STATUS_FIFO_OVRFLW      (1 << 9)
#define STATUS_FIFO_FULL        (1 << 8)
#define STATUS_FIFO_EMPTY       (1 << 5)
#define STATUS_FIFO_UNDRFLW     (1 << 4)
#define STATUS_CMD_BUSY         (1 << 3)
#define STATUS_DATA_BUSY        (1 << 2)
#define STATUS_BUSY             (1 << 0)

/* CMD0 bits */
#define CMD0_CMD_MASK           0xff
#define CMD0_APPEND_8CYC        (1 << 20)

#define SSP_FIFO_WORDS  32

static void mxs_ssp_irq_update(MxsSspState *s)
{
    uint32_t fires = 0;

    /* interleaved IRQ / IRQ_EN pairs, bits 15..31 */
    for (uint32_t m = CTRL1_FIFO_OVERRUN_IRQ;
         m && m <= CTRL1_SDIO_IRQ; m <<= 2) {
        if ((s->ctrl1 & m) && (s->ctrl1 & (m >> 1))) {
            fires |= m;
        }
    }

    if (fires) {
        qemu_set_irq(s->irq, 1);
    } else {
        qemu_set_irq(s->irq, 0);
    }
}

/* ------------------------------------------------------------------ */
/* command execution                                                  */

static void mxs_ssp_fill_fifo(MxsSspState *s);

static void mxs_ssp_execute(MxsSspState *s)
{
    SDRequest req;
    uint8_t resp[16];
    size_t rlen;

    req.cmd = s->cmd0 & CMD0_CMD_MASK;
    req.arg = s->cmd1;
    req.crc = 0;

    qemu_log_mask(LOG_TRACE, "mxs-ssp: CMD%d arg 0x%08x (ctrl0 0x%08x)\n",
                  req.cmd, req.arg, s->ctrl0);

    if (!sdbus_get_inserted(&s->sdbus)) {
        s->status |= STATUS_RESP_TIMEOUT | STATUS_TIMEOUT;
        s->ctrl1 |= CTRL1_RESP_TIMEOUT_IRQ;
        mxs_ssp_irq_update(s);
        return;
    }

    rlen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    if (rlen == 0) {
        /*
         * No response: only a timeout when the host actually expects
         * one (GET_RESP/CHECK_RESP).  Commands like CMD0 finish
         * cleanly on real hardware and must not raise RESP_TIMEOUT.
         */
        if (s->ctrl0 & (CTRL0_GET_RESP | CTRL0_CHECK_RESP)) {
            s->status |= STATUS_RESP_TIMEOUT;
            s->ctrl1 |= CTRL1_RESP_TIMEOUT_IRQ;
        }
        mxs_ssp_irq_update(s);
        return;
    }

    /*
     * Response mapping (verified against the Linux mxs-mmc driver):
     *   48-bit response: SDRESP0 = the 32-bit value (wire order)
     *   136-bit response: SDRESP0..3 = wire words 0..3
     */
    s->sdresp[0] = s->sdresp[1] = s->sdresp[2] = s->sdresp[3] = 0;
    if (rlen == 4) {
        s->sdresp[0] = ldl_be_p(resp);
    } else if (rlen >= 16) {
        s->sdresp[0] = ldl_be_p(&resp[0]);
        s->sdresp[1] = ldl_be_p(&resp[4]);
        s->sdresp[2] = ldl_be_p(&resp[8]);
        s->sdresp[3] = ldl_be_p(&resp[12]);
    }

    /* command done: reset sticky error/status bits */
    s->status &= ~(STATUS_CMD_BUSY | STATUS_BUSY |
                   STATUS_RESP_TIMEOUT | STATUS_TIMEOUT |
                   STATUS_RESP_CRC_ERR | STATUS_RESP_ERR |
                   STATUS_DATA_CRC_ERR | STATUS_FIFO_OVRFLW |
                   STATUS_FIFO_UNDRFLW);

    /* prepare data phase */
    if (s->ctrl0 & CTRL0_DATA_XFER) {
        s->data_dir_read = !!(s->ctrl0 & CTRL0_READ);
        s->data_remaining = s->xfer_size;
        s->fifo_level = 0;
        s->status |= STATUS_DATA_BUSY;
        if (s->data_dir_read) {
            /* pull the first FIFO words from the card immediately so
             * STATUS.FIFO_EMPTY clears for the polling data path */
            mxs_ssp_fill_fifo(s);
        }
    }

    /* raise the "transfer complete" event so a DMA engine waiting on
     * WAIT4END completes */
    s->status |= STATUS_DMAEND;
}

/* ------------------------------------------------------------------ */
/* data handling                                                      */

static void mxs_ssp_fill_fifo(MxsSspState *s)
{
    /* pull data from the card into the FIFO for CPU access */
    while (s->data_dir_read && s->data_remaining >= 4 &&
           s->fifo_level < SSP_FIFO_WORDS) {
        uint8_t buf[4];
        sdbus_read_data(&s->sdbus, buf, 4);
        /*
         * The SSP packs the first wire byte into bits [7:0] of the
         * FIFO word (u-boot mxsmmc reads the data register directly
         * as native-endian 32-bit words).
         */
        s->fifo[s->fifo_level++] = ldl_le_p(buf);
        s->data_remaining -= 4;
    }
    if (s->data_remaining == 0) {
        s->status &= ~STATUS_DATA_BUSY;
    }
}

/* DMA client hook: PIO descriptor word */
int mxs_ssp_dma_pio(MxsSspState *s, int word_idx, uint32_t value)
{
    if (word_idx < 0) {
        /* descriptor complete: kick the command if ENABLE is set */
        if (s->ctrl0 & CTRL0_ENABLE) {
            s->ctrl0 |= CTRL0_RUN;
            mxs_ssp_execute(s);
        }
        return 1;
    }

    switch (word_idx * 0x10) {
    case SSP_CTRL0:
        s->ctrl0 = value & ~CTRL0_RUN;
        break;
    case SSP_CMD0:
        s->cmd0 = value;
        break;
    case SSP_CMD1:
        s->cmd1 = value;
        break;
    case SSP_XFER_SIZE:
        s->xfer_size = value;
        break;
    case SSP_BLOCK_SIZE:
        s->block_size = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-ssp: dma pio word %d (0x%08x) ignored\n",
                      word_idx, value);
        break;
    }
    return 0;
}

/* DMA client hook: data descriptor */
int mxs_ssp_dma_data(MxsSspState *s, bool to_device,
                     dma_addr_t bufaddr, uint32_t len)
{
    uint8_t *buf = g_malloc(len);
    MemTxResult res;

    qemu_log_mask(LOG_TRACE,
                  "mxs-ssp: dma %s %u bytes at 0x%" HWADDR_PRIx "\n",
                  to_device ? "write" : "read", len, bufaddr);

    if (to_device) {
        res = address_space_read(&address_space_memory, bufaddr,
                                 MEMTXATTRS_UNSPECIFIED, buf, len);
        if (res == MEMTX_OK) {
            sdbus_write_data(&s->sdbus, buf, len);
        }
    } else {
        if (!(s->ctrl0 & CTRL0_DATA_XFER) || !s->data_dir_read) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mxs-ssp: dma read without pending read data\n");
            g_free(buf);
            return -1;
        }
        uint32_t n = MIN(len, s->data_remaining);
        sdbus_read_data(&s->sdbus, buf, n);
        res = address_space_write(&address_space_memory, bufaddr,
                                  MEMTXATTRS_UNSPECIFIED, buf, n);
        s->data_remaining -= n;
        if (s->data_remaining == 0) {
            s->status &= ~STATUS_DATA_BUSY;
        }
    }

    g_free(buf);
    return res == MEMTX_OK ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* register access                                                    */

static uint64_t mxs_ssp_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsSspState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    switch (reg) {
    case SSP_CTRL0:
        return s->ctrl0;
    case SSP_CMD0:
        return s->cmd0;
    case SSP_CMD1:
        return s->cmd1;
    case SSP_XFER_SIZE:
        return s->xfer_size;
    case SSP_BLOCK_SIZE:
        return s->block_size;
    case SSP_COMPREF:
        return s->compref;
    case SSP_COMPMASK:
        return s->compmask;
    case SSP_TIMING:
        return s->timing;
    case SSP_CTRL1:
        return s->ctrl1;
    case SSP_DATA:
    {
        uint32_t v = 0;
        if (s->data_dir_read) {
            mxs_ssp_fill_fifo(s);
            if (s->fifo_level) {
                v = s->fifo[0];
                memmove(&s->fifo[0], &s->fifo[1],
                        (s->fifo_level - 1) * sizeof(uint32_t));
                s->fifo_level--;
            }
        }
        return v;
    }
    case SSP_SDRESP(0):
    case SSP_SDRESP(1):
    case SSP_SDRESP(2):
    case SSP_SDRESP(3):
        return s->sdresp[(reg - 0xa0) / 0x10];
    case SSP_STATUS:
    {
        uint32_t st = s->status;
        /* live bits */
        st &= ~(STATUS_PRESENT | STATUS_SD_PRESENT | STATUS_CARD_DETECT |
                STATUS_FIFO_FULL | STATUS_FIFO_EMPTY |
                STATUS_CMD_BUSY | STATUS_DATA_BUSY | STATUS_BUSY);
        if (sdbus_get_inserted(&s->sdbus)) {
            st |= STATUS_PRESENT | STATUS_SD_PRESENT;
            /* CARD_DETECT reads 0 when a card is present */
        } else {
            st |= STATUS_CARD_DETECT;
        }
        if (s->fifo_level >= SSP_FIFO_WORDS) {
            st |= STATUS_FIFO_FULL;
        }
        if (s->fifo_level == 0) {
            st |= STATUS_FIFO_EMPTY;
        }
        return st;
    }
    case SSP_DLL_STS:
        return (1 << 1);        /* REF lock */
    case SSP_VERSION:
        return s->version;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-ssp: unimplemented read 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
    (void)off;
}

static void mxs_ssp_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    MxsSspState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    switch (reg) {
    case SSP_CTRL0:
    {
        uint32_t old = s->ctrl0;

        switch (off) {
        case 0:
            s->ctrl0 = value & ~CTRL0_RUN;
            if (value & CTRL0_RUN) {
                mxs_ssp_execute(s);
            }
            break;
        case 4:
            s->ctrl0 |= value & ~CTRL0_RUN;
            if (value & CTRL0_RUN) {
                mxs_ssp_execute(s);
            }
            break;
        case 8:
            s->ctrl0 &= ~value;
            return;
        case 12:
            s->ctrl0 ^= value;
            return;
        }
        /* SFTRST rising edge -> clock gates once */
        if (!(old & CTRL0_SFTRST) && (s->ctrl0 & CTRL0_SFTRST)) {
            s->ctrl0 |= CTRL0_CLKGATE;
        }
        return;
    }
    case SSP_CMD0:
        switch (off) {
        case 0:  s->cmd0 = value; break;
        case 4:  s->cmd0 |= value; break;
        case 8:  s->cmd0 &= ~value; break;
        case 12: s->cmd0 ^= value; break;
        }
        return;
    case SSP_CMD1:
        switch (off) {
        case 0:  s->cmd1 = value; break;
        case 4:  s->cmd1 |= value; break;
        case 8:  s->cmd1 &= ~value; break;
        case 12: s->cmd1 ^= value; break;
        }
        return;
    case SSP_XFER_SIZE:
        switch (off) {
        case 0:  s->xfer_size = value; break;
        case 4:  s->xfer_size |= value; break;
        case 8:  s->xfer_size &= ~value; break;
        case 12: s->xfer_size ^= value; break;
        }
        return;
    case SSP_BLOCK_SIZE:
        switch (off) {
        case 0:  s->block_size = value; break;
        case 4:  s->block_size |= value; break;
        case 8:  s->block_size &= ~value; break;
        case 12: s->block_size ^= value; break;
        }
        return;
    case SSP_COMPREF:
        switch (off) {
        case 0:  s->compref = value; break;
        case 4:  s->compref |= value; break;
        case 8:  s->compref &= ~value; break;
        case 12: s->compref ^= value; break;
        }
        return;
    case SSP_COMPMASK:
        switch (off) {
        case 0:  s->compmask = value; break;
        case 4:  s->compmask |= value; break;
        case 8:  s->compmask &= ~value; break;
        case 12: s->compmask ^= value; break;
        }
        return;
    case SSP_TIMING:
        s->timing = value;
        return;
    case SSP_CTRL1:
        qemu_log_mask(LOG_TRACE, "mxs-ssp: CTRL1 %s 0x%08x\n",
                      off == 0 ? "REG" : off == 4 ? "SET" :
                      off == 8 ? "CLR" : "TOG", (uint32_t)value);
        switch (off) {
        case 0:
            s->ctrl1 = value;
            break;
        case 4:
            s->ctrl1 |= value;
            break;
        case 8:
            s->ctrl1 &= ~value;
            break;
        case 12:
            s->ctrl1 ^= value;
            break;
        }
        mxs_ssp_irq_update(s);
        return;
    case SSP_DATA:
    {
        if (!s->data_dir_read && s->data_remaining >= 4) {
            uint8_t buf[4];
            stl_le_p(buf, value);
            sdbus_write_data(&s->sdbus, buf, 4);
            s->data_remaining -= 4;
            if (s->data_remaining == 0) {
                s->status &= ~STATUS_DATA_BUSY;
            }
        }
        return;
    }
    case SSP_STATUS:
        /* writing 1s clears the error/status bits */
        s->status &= ~value;
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-ssp: unimplemented write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, value);
        return;
    }
}

static const MemoryRegionOps mxs_ssp_ops = {
    .read = mxs_ssp_read,
    .write = mxs_ssp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mxs_ssp_reset(DeviceState *dev)
{
    MxsSspState *s = MXS_SSP(dev);

    s->ctrl0 = 0;
    s->cmd0 = 0;
    s->cmd1 = 0;
    s->xfer_size = 0;
    s->block_size = 0;
    s->compref = 0;
    s->compmask = 0;
    s->timing = 0;
    s->ctrl1 = 0;
    s->status = 0;
    s->ddr_ctrl = 0;
    memset(s->sdresp, 0, sizeof(s->sdresp));
    s->fifo_level = 0;
    s->data_remaining = 0;
    s->data_dir_read = false;
    s->version = 0x04000000;    /* imx28 SSP v4.0 magic-ish */
}

static void mxs_ssp_init(Object *obj)
{
    MxsSspState *s = MXS_SSP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_ssp_ops, s, "mxs-ssp", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj),
              "sd-bus");
}

static void mxs_ssp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_ssp_reset);
}

static const TypeInfo mxs_ssp_info = {
    .name = TYPE_MXS_SSP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsSspState),
    .instance_init = mxs_ssp_init,
    .class_init = mxs_ssp_class_init,
};

static void mxs_ssp_register(void)
{
    type_register_static(&mxs_ssp_info);
}
type_init(mxs_ssp_register)
