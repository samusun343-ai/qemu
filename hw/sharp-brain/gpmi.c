/*
 * i.MX28 GPMI (General Purpose Media Interface) NAND flash controller
 * + BCH ECC controller + a Micron MT29F2G08A-class NAND chip.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The GPMI is the i.MX28's NAND flash host (base 0x8000C000).  The
 * WinCE FMD/NDF1: driver drives it through the APBH DMA (channels
 * 4-11, GPMI0-7) with PIO descriptors that program the GPMI
 * registers and data descriptors that move command/address/data
 * bytes.  ECC transfers use NO_DMAXFER descriptors: the PIO words
 * program GPMI (CTRL0/ECCCTRL/ECCCOUNT/PAYLOAD/AUXILIARY) and the
 * GPMI itself bus-masters the page between system memory and the
 * NAND through the BCH.
 *
 * The BCH (base 0x8000A000) is modelled as a pass-through ECC: it
 * accepts the layout programming and reports every ECC operation as
 * successful (status 0x00 = OK, erased pages 0xFF).  Real BCH error
 * correction is a later step; the driver only needs the read to
 * succeed and the data to come back intact to proceed.
 *
 * The NAND chip is a Micron MT29F2G08ABADA-class 2 Gb part
 * (2048+128 byte pages, 64 pages/block, 2048 blocks, 1 LUN,
 * 256 MiB).  It answers Read ID (2C DA 90 95 06), the ONFI
 * parameter page (0xEC) and RESET / READ / PROGRAM / ERASE.
 * A valid FCB (fingerprint 0x20424346) is synthesised at block 0
 * (replicated to blocks 1-3, as kobs-ng does), because the FMD
 * driver reads the FCB from NAND block 0 via a BCH-ECC read as its
 * first access; without it the driver fails its first BCH read.
 *
 * Register map: i.MX28 Reference Manual ch. 9 (GPMI) / ch. 19
 * (BCH), mirrored by U-Boot regs-gpmi.h / regs-bch.h and Linux
 * gpmi-nand gpmi-regs.h / bch-regs.h.  ONFI layout: Linux
 * include/linux/mtd/onfi.h.  FCB/DBBT: Barebox imx-nand-bcb.h.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "brain.h"

/* ------------------------------------------------------------------ */
/* GPMI registers (base 0x8000C000, 0x10 stride, SET/CLR/TOG aliases) */

#define GPMI_CTRL0        0x000
#define GPMI_COMPARE      0x010
#define GPMI_ECCCTRL      0x020
#define GPMI_ECCCOUNT     0x030
#define GPMI_PAYLOAD      0x040
#define GPMI_AUXILIARY    0x050
#define GPMI_CTRL1        0x060
#define GPMI_TIMING0      0x070
#define GPMI_TIMING1      0x080
#define GPMI_TIMING2      0x090
#define GPMI_DATA         0x0a0
#define GPMI_STAT         0x0b0
#define GPMI_DEBUG        0x0c0
#define GPMI_VERSION      0x0d0

/* CTRL0 */
#define GPMI_CTRL0_SFTRST          (1u << 31)
#define GPMI_CTRL0_CLKGATE         (1u << 30)
#define GPMI_CTRL0_RUN             (1u << 29)
#define GPMI_CTRL0_DEV_IRQ_EN      (1u << 28)
#define GPMI_CTRL0_LOCK_CS         (1u << 27)
#define GPMI_CTRL0_UDMA            (1u << 26)
#define GPMI_CTRL0_COMMAND_MODE    (3u << 24)
#define GPMI_CTRL0_CMD_WRITE       (0u << 24)
#define GPMI_CTRL0_CMD_READ        (1u << 24)
#define GPMI_CTRL0_CMD_READ_COMPARE (2u << 24)
#define GPMI_CTRL0_CMD_WAIT_READY  (3u << 24)
#define GPMI_CTRL0_WORD_LENGTH     (1u << 23)
#define GPMI_CTRL0_CS              (7u << 20)
#define GPMI_CTRL0_ADDRESS         (3u << 17)
#define GPMI_CTRL0_ADDR_DATA       (0u << 17)
#define GPMI_CTRL0_ADDR_CLE        (1u << 17)
#define GPMI_CTRL0_ADDR_ALE        (2u << 17)
#define GPMI_CTRL0_ADDR_INCREMENT  (1u << 16)
#define GPMI_CTRL0_XFER_COUNT      0xffffu

/* ECCCTRL */
#define GPMI_ECCCTRL_HANDLE        (0xffffu << 16)
#define GPMI_ECCCTRL_ECC_CMD       (3u << 13)
#define GPMI_ECCCTRL_ECC_DECODE    (0u << 13)
#define GPMI_ECCCTRL_ECC_ENCODE    (1u << 13)
#define GPMI_ECCCTRL_ENABLE_ECC    (1u << 12)
#define GPMI_ECCCTRL_RANDOMIZER_EN (1u << 11)
#define GPMI_ECCCTRL_BUFFER_MASK   0x1ffu
#define GPMI_ECCCTRL_BCH_AUXONLY   0x100u
#define GPMI_ECCCTRL_BCH_PAGE      0x1ffu

/* CTRL1 */
#define GPMI_CTRL1_DECOUPLE_CS     (1u << 24)
#define GPMI_CTRL1_GANGED_RDYBUSY  (1u << 19)
#define GPMI_CTRL1_BCH_MODE        (1u << 18)
#define GPMI_CTRL1_DLL_ENABLE      (1u << 17)
#define GPMI_CTRL1_DEV_IRQ         (1u << 10)
#define GPMI_CTRL1_TIMEOUT_IRQ     (1u << 9)
#define GPMI_CTRL1_BURST_EN        (1u << 8)
#define GPMI_CTRL1_DEV_RESET       (1u << 3)
#define GPMI_CTRL1_ATA_IRQRDY_POL  (1u << 2)
#define GPMI_CTRL1_GPMI_MODE       (1u << 0)

/* STAT */
#define GPMI_STAT_READY_BUSY       (0xffu << 24)
#define GPMI_STAT_FIFO_EMPTY       (1u << 2)
#define GPMI_STAT_FIFO_FULL        (1u << 1)
#define GPMI_STAT_PRESENT          (1u << 0)

/* ------------------------------------------------------------------ */
/* BCH registers (base 0x8000A000) */

#define BCH_CTRL          0x000
#define BCH_STATUS0       0x010
#define BCH_MODE          0x020
#define BCH_ENCODEPTR     0x030
#define BCH_DATAPTR       0x040
#define BCH_METAPTR       0x050
#define BCH_LAYOUTSELECT  0x070
#define BCH_FLASH0LAYOUT0 0x080
#define BCH_FLASH0LAYOUT1 0x090
#define BCH_VERSION       0x160

#define BCH_CTRL_SFTRST         (1u << 31)
#define BCH_CTRL_CLKGATE        (1u << 30)
#define BCH_CTRL_COMPLETE_IRQ_EN (1u << 8)
#define BCH_CTRL_COMPLETE_IRQ   (1u << 0)

/* BCH per-chunk status (STATUS0 STATUS_BLK0 and aux-status bytes) */
#define BCH_STATUS_OK           0x00
#define BCH_STATUS_UNCORRECTABLE 0xfe
#define BCH_STATUS_ERASED       0xff

/* ------------------------------------------------------------------ */
/* NAND chip: Micron MT29F2G08ABADA-class, 2 Gb, 2048+128 page */

#define NAND_PAGE_DATA   2048
#define NAND_PAGE_SPARE  128
#define NAND_PAGE_TOTAL  (NAND_PAGE_DATA + NAND_PAGE_SPARE)
#define NAND_PAGES_BLOCK 64
#define NAND_BLOCKS      2048
#define NAND_TOTAL_BYTES ((uint64_t)NAND_PAGE_TOTAL * NAND_PAGES_BLOCK \
                          * NAND_BLOCKS)

/* NAND commands */
#define NAND_CMD_READ0      0x00
#define NAND_CMD_READ1      0x01
#define NAND_CMD_RNDOUT     0x05
#define NAND_CMD_PAGEPROG   0x10
#define NAND_CMD_READOOB    0x50
#define NAND_CMD_ERASE1     0x60
#define NAND_CMD_STATUS     0x70
#define NAND_CMD_SEQIN      0x80
#define NAND_CMD_READID     0x90
#define NAND_CMD_ERASE2     0xd0
#define NAND_CMD_PARAM      0xec
#define NAND_CMD_RNDIN      0x85
#define NAND_CMD_RESET      0xff
#define NAND_CMD_READ2      0x30

/* Read ID bytes (Micron MT29F2G08ABADA) */
static const uint8_t nand_id[5] = { 0x2c, 0xda, 0x90, 0x95, 0x06 };

/* ONFI parameter page (256 bytes, CRC-16 poly 0x8005 init 0x4F4E) */
static uint16_t onfi_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0x4f4e;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
        }
    }
    return crc;
}

static void nand_build_onfi(uint8_t *p)
{
    memset(p, 0, 256);
    memcpy(p + 0, "ONFI", 4);
    /* revision: ONFI 1.0 */
    p[4] = 0x02; p[5] = 0x00;
    /* features: none */
    /* manufacturer / model */
    memcpy(p + 32, "MICRON", 6);
    memcpy(p + 44, "MT29F2G08ABADA", 13);
    /* jedec id */
    p[64] = 0x00;
    /* byte per page (LE32 @80) */
    p[80] = 0x00; p[81] = 0x08; p[82] = 0x00; p[83] = 0x00;
    /* spare bytes per page (LE16 @84) */
    p[84] = 0x80; p[85] = 0x00;
    /* data bytes per partial page (LE32 @86) = 2048 */
    p[86] = 0x00; p[87] = 0x08; p[88] = 0x00; p[89] = 0x00;
    /* spare bytes per partial page (LE16 @90) = 128 */
    p[90] = 0x80; p[91] = 0x00;
    /* pages per block (LE32 @92) = 64 */
    p[92] = 0x40; p[93] = 0x00; p[94] = 0x00; p[95] = 0x00;
    /* blocks per LUN (LE32 @96) = 2048 */
    p[96] = 0x00; p[97] = 0x08; p[98] = 0x00; p[99] = 0x00;
    /* LUN count @100 */
    p[100] = 0x01;
    /* address cycles @101 = 5 (2 column + 3 row) */
    p[101] = 0x05;
    /* bits per cell @102 = 1 (SLC) */
    p[102] = 0x01;
    /* bad blocks per LUN (LE16 @103) = 20 */
    p[103] = 0x14; p[104] = 0x00;
    /* block endurance (LE16 @105) */
    p[105] = 0x01; p[106] = 0x00;
    /* guaranteed good blocks @107 = 1 */
    p[107] = 0x01;
    /* programs per page @110 = 4 */
    p[110] = 0x04;
    /* ECC bits @112 = 4 */
    p[112] = 0x04;
    /* SDR timing modes (LE16 @129): modes 0-5 */
    p[129] = 0x3f; p[130] = 0x00;
    /* t_prog (LE16 @133, us) = 700 */
    p[133] = 0xbc; p[134] = 0x02;
    /* t_bers (LE16 @135, ms) = 3000 */
    p[135] = 0xb8; p[136] = 0x0b;
    /* t_r (LE16 @137, us) = 25 */
    p[137] = 0x19; p[138] = 0x00;
    /* t_ccs (LE16 @139, ns) = 100 */
    p[139] = 0x64; p[140] = 0x00;
    uint16_t crc = onfi_crc16(p, 254);
    p[254] = crc & 0xff;
    p[255] = (crc >> 8) & 0xff;
}

/* FCB (Firmware Configuration Block, Barebox imx-nand-bcb.h) */
#define FCB_FINGERPRINT 0x20424346u /* 'FCB ' */
#define FCB_VERSION     0x01000000u

static void nand_build_fcb(uint8_t *page)
{
    uint32_t *w = (uint32_t *)page;
    memset(page, 0xff, NAND_PAGE_TOTAL);
    w[0] = 0;                       /* Checksum, fixed up below */
    w[1] = FCB_FINGERPRINT;         /* 0x04 */
    w[2] = FCB_VERSION;             /* 0x08 */
    /* 0x0C: DataSetup, DataHold, AddressSetup, DSAMPLE_TIME */
    page[0x0c] = 80;
    page[0x0d] = 60;
    page[0x0e] = 25;
    page[0x0f] = 6;
    w[4] = NAND_PAGE_DATA;          /* PageDataSize @0x10 */
    w[5] = NAND_PAGE_TOTAL;         /* TotalPageSize @0x14 (2176) */
    w[6] = 64;                      /* SectorsPerBlock @0x18 */
    /* ECC: 4-bit, 512-byte blocks, metadata 10 bytes */
    w[7] = 4;                       /* EccBlock0EccType @0x1C (level) */
    w[8] = 512;                     /* EccBlock0Size @0x20 */
    w[9] = 4;                       /* EccBlockNEccType @0x24 */
    w[10] = 512;                    /* EccBlockNSize @0x28 */
    w[11] = 10;                     /* MetadataSize @0x2C */
    w[12] = 3;                      /* NumEccBlocksPerPage @0x30 */
    w[13] = 0;                      /* EccBlock0EccLevel @0x34 */
    /* Firmware start pages (first firmware at block 8 = page 512) */
    w[14] = 512;                    /* Firmware1_startingPage @0x38 */
    w[15] = 512;                    /* Firmware2_startingPage @0x3C */
    w[16] = 52;                     /* PagesInFirmware1 @0x40 */
    w[17] = 52;                     /* PagesInFirmware2 @0x44 */
    w[18] = 0;                      /* DBBTSearchAreaStartAddress @0x48 */
    w[19] = 1999;                   /* BadBlockMarkerByte @0x4C */
    /* checksum: chosen so the 32-bit sum of all FCB words == 0 */
    {
        uint64_t sum = 0;
        for (int i = 1; i < 128; i++) {
            sum += w[i];
        }
        w[0] = (uint32_t)(0u - (uint32_t)sum);
    }
}

/* ------------------------------------------------------------------ */
/* GPMI + BCH + NAND state                                            */

#define TYPE_MXS_GPMI "mxs-gpmi"
/* (the MXS_GPMI instance checker + struct layout are declared in
 * brain.h, like the other devices in this directory) */

/* NAND command state machine */
enum {
    NAND_ST_IDLE = 0,     /* waiting for a command byte */
    NAND_ST_ADDR,         /* collecting address bytes */
    NAND_ST_DATA_IN,      /* collecting program data */
    NAND_ST_CONFIRM,      /* waiting for the confirm command */
    NAND_ST_READ,         /* read data ready in read_buf */
};

struct MxsGpmiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;         /* GPMI @ 0x8000C000 */
    MemoryRegion bch_iomem;     /* BCH  @ 0x8000A000 */
    qemu_irq irq;               /* GPMI device IRQ -> ICOLL 88 */
    qemu_irq bch_irq;           /* BCH complete IRQ -> ICOLL 41 */

    /* GPMI registers */
    uint32_t ctrl0, compare, eccctrl, ecccount, payload, auxiliary;
    uint32_t ctrl1, timing0, timing1, timing2;
    uint32_t stat, debug, version;

    /* BCH registers */
    uint32_t bch_ctrl, bch_status0, bch_mode;
    uint32_t bch_encodeptr, bch_dataptr, bch_metaptr;
    uint32_t bch_layoutselect;
    uint32_t bch_layout[4][2];  /* FLASHnLAYOUT0/1 */
    uint32_t bch_version;

    /* NAND chip (in-memory, erased = 0xFF) */
    uint8_t *nand;              /* NAND_TOTAL_BYTES */
    uint8_t onfi[256];

    /* NAND command state machine */
    int nand_state;
    uint8_t cmd;                /* current command byte */
    uint8_t addr[5];
    int addr_len;               /* address bytes collected */
    int addr_want;              /* address bytes expected */
    uint32_t col;               /* column (byte offset in page) */
    uint32_t row;               /* page number */
    uint8_t read_buf[NAND_PAGE_TOTAL];
    uint32_t read_len;
    uint32_t read_pos;
    uint8_t prog_buf[NAND_PAGE_TOTAL];
    uint32_t prog_len;
    bool prog_data;             /* in a program data phase */

    /* raw data-phase state (set up by the CTRL0 kick, consumed by
     * dma_data) */
    bool data_read_pending;     /* a read phase is armed */
    uint32_t data_remaining;
};

/* ------------------------------------------------------------------ */
/* NAND chip helpers                                                  */

static uint8_t *nand_page_ptr(MxsGpmiState *s, uint32_t page)
{
    uint64_t off = (uint64_t)page * NAND_PAGE_TOTAL;
    if (off >= NAND_TOTAL_BYTES) {
        return NULL;
    }
    return s->nand + off;
}

static bool nand_page_erased(const uint8_t *page)
{
    for (int i = 0; i < NAND_PAGE_TOTAL; i++) {
        if (page[i] != 0xff) {
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* BCH (pass-through ECC)                                             */

static void mxs_bch_irq_update(MxsGpmiState *s)
{
    bool fire = (s->bch_ctrl & BCH_CTRL_COMPLETE_IRQ) &&
                (s->bch_ctrl & BCH_CTRL_COMPLETE_IRQ_EN);
    qemu_set_irq(s->bch_irq, fire);
}

static void mxs_bch_complete(MxsGpmiState *s, uint8_t chunk_status)
{
    s->bch_ctrl |= BCH_CTRL_COMPLETE_IRQ;
    /* STATUS0: per-chunk status in STATUS_BLK0 (bits 8-15) */
    s->bch_status0 &= ~(0xffu << 8);
    s->bch_status0 |= (uint32_t)chunk_status << 8;
    mxs_bch_irq_update(s);
}

/* ------------------------------------------------------------------ */
/* GPMI IRQ                                                           */

static void mxs_gpmi_irq_update(MxsGpmiState *s)
{
    bool fire = (s->ctrl1 & GPMI_CTRL1_DEV_IRQ) &&
                (s->ctrl0 & GPMI_CTRL0_DEV_IRQ_EN);
    qemu_set_irq(s->irq, fire);
}

/* ------------------------------------------------------------------ */
/* NAND command execution                                             */

static void nand_set_read(MxsGpmiState *s, const uint8_t *data,
                          uint32_t len)
{
    memcpy(s->read_buf, data, len);
    s->read_len = len;
    s->read_pos = 0;
    s->nand_state = NAND_ST_READ;
    s->data_read_pending = true;
    s->data_remaining = len;
}

/* execute a command once its address bytes are collected (or for a
 * single-byte command, immediately) */
static void nand_exec(MxsGpmiState *s)
{
    switch (s->cmd) {
    case NAND_CMD_RESET:
        s->nand_state = NAND_ST_IDLE;
        s->read_len = s->read_pos = 0;
        s->data_read_pending = false;
        break;
    case NAND_CMD_READID: {
        /* address 0x00 = device ID (5 bytes), 0x20 = "ONFI" */
        if (s->addr_len >= 1 && s->addr[0] == 0x20) {
            nand_set_read(s, (const uint8_t *)"ONFI", 4);
        } else {
            nand_set_read(s, nand_id, sizeof(nand_id));
        }
        break;
    }
    case NAND_CMD_STATUS:
        /* bit 7 WP# (1 = not protected), bit 6 RDY (1 = ready) */
        nand_set_read(s, (const uint8_t *)"\xc0", 1);
        break;
    case NAND_CMD_PARAM:
        nand_set_read(s, s->onfi, sizeof(s->onfi));
        break;
    case NAND_CMD_READ0:
    case NAND_CMD_READ1:
    case NAND_CMD_READOOB:
        /* 0x30 confirm must precede the data phase; if the driver
         * sent the confirm already (or reads without one, as some
         * drivers do for the OOB), arm the page read now. */
        if (s->addr_len >= 5) {
            s->col = s->addr[0] | ((uint32_t)s->addr[1] << 8);
            s->row = s->addr[2] | ((uint32_t)s->addr[3] << 8) |
                     ((uint32_t)s->addr[4] << 16);
            s->nand_state = NAND_ST_CONFIRM;
        }
        break;
    case NAND_CMD_READ2:        /* 0x30 confirm: arm the page read */
        if (s->nand_state == NAND_ST_CONFIRM) {
            uint8_t *page = nand_page_ptr(s, s->row);
            if (!page) {
                static const uint8_t ff[NAND_PAGE_TOTAL] = {
                    [0 ... NAND_PAGE_TOTAL - 1] = 0xff
                };
                nand_set_read(s, ff, NAND_PAGE_TOTAL);
            } else {
                uint32_t off = s->col < NAND_PAGE_TOTAL ? s->col : 0;
                nand_set_read(s, page + off, NAND_PAGE_TOTAL - off);
            }
        }
        break;
    case NAND_CMD_SEQIN:
        if (s->addr_len >= 5) {
            s->col = s->addr[0] | ((uint32_t)s->addr[1] << 8);
            s->row = s->addr[2] | ((uint32_t)s->addr[3] << 8) |
                     ((uint32_t)s->addr[4] << 16);
            s->prog_len = 0;
            s->prog_data = true;
            s->nand_state = NAND_ST_DATA_IN;
        }
        break;
    case NAND_CMD_PAGEPROG:     /* 0x10 confirm: program the page */
        if (s->nand_state == NAND_ST_DATA_IN) {
            uint8_t *page = nand_page_ptr(s, s->row);
            if (page) {
                uint32_t off = s->col < NAND_PAGE_TOTAL ? s->col : 0;
                uint32_t n = s->prog_len;
                if (off + n > NAND_PAGE_TOTAL) {
                    n = NAND_PAGE_TOTAL - off;
                }
                /* NAND programming can only clear bits (1->0) */
                for (uint32_t i = 0; i < n; i++) {
                    page[off + i] &= s->prog_buf[i];
                }
            }
            s->prog_data = false;
            s->nand_state = NAND_ST_IDLE;
        }
        break;
    case NAND_CMD_ERASE1:
        if (s->addr_len >= 3) {
            s->row = s->addr[0] | ((uint32_t)s->addr[1] << 8) |
                     ((uint32_t)s->addr[2] << 16);
            s->nand_state = NAND_ST_CONFIRM;
        }
        break;
    case NAND_CMD_ERASE2:       /* 0xD0 confirm: erase the block */
        if (s->nand_state == NAND_ST_CONFIRM) {
            uint32_t block = s->row / NAND_PAGES_BLOCK;
            uint32_t first = block * NAND_PAGES_BLOCK;
            for (uint32_t p = 0; p < NAND_PAGES_BLOCK; p++) {
                uint8_t *page = nand_page_ptr(s, first + p);
                if (page) {
                    memset(page, 0xff, NAND_PAGE_TOTAL);
                }
            }
            s->nand_state = NAND_ST_IDLE;
        }
        break;
    default:
        s->nand_state = NAND_ST_IDLE;
        break;
    }
}

/* feed one command/address byte into the NAND state machine */
static void nand_feed_byte(MxsGpmiState *s, uint8_t b)
{
    switch (s->nand_state) {
    case NAND_ST_IDLE:
        s->cmd = b;
        s->addr_len = 0;
        s->prog_len = 0;
        switch (b) {
        case NAND_CMD_RESET:
        case NAND_CMD_STATUS:
            nand_exec(s);
            break;
        case NAND_CMD_READID:
        case NAND_CMD_PARAM:
            s->addr_want = 1;
            s->nand_state = NAND_ST_ADDR;
            break;
        case NAND_CMD_READ0:
        case NAND_CMD_READ1:
        case NAND_CMD_READOOB:
        case NAND_CMD_SEQIN:
            s->addr_want = 5;
            s->nand_state = NAND_ST_ADDR;
            break;
        case NAND_CMD_ERASE1:
            s->addr_want = 3;
            s->nand_state = NAND_ST_ADDR;
            break;
        case NAND_CMD_READ2:        /* confirm for a pending read */
        case NAND_CMD_PAGEPROG:     /* confirm for a pending program */
        case NAND_CMD_ERASE2:       /* confirm for a pending erase */
            nand_exec(s);
            break;
        default:
            break;
        }
        break;
    case NAND_ST_ADDR:
        if (s->addr_len < 5) {
            s->addr[s->addr_len++] = b;
        }
        if (s->addr_len >= s->addr_want) {
            /* READ ID / PARAM execute as soon as the address arrives;
             * READ / SEQIN / ERASE wait for their confirm command. */
            switch (s->cmd) {
            case NAND_CMD_READID:
            case NAND_CMD_PARAM:
                nand_exec(s);
                break;
            default:
                nand_exec(s);   /* derives col/row, enters CONFIRM */
                break;
            }
        }
        break;
    case NAND_ST_DATA_IN:
        /* program data bytes (collected until the 0x10 confirm) */
        if (s->prog_len < sizeof(s->prog_buf)) {
            s->prog_buf[s->prog_len++] = b;
        }
        break;
    case NAND_ST_CONFIRM:
        /* this byte is the confirm command (0x30 / 0x10 / 0xD0) */
        s->cmd = b;
        nand_exec(s);
        break;
    case NAND_ST_READ:
        /* a new command byte while a read is pending: restart */
        s->nand_state = NAND_ST_IDLE;
        nand_feed_byte(s, b);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* ECC transfer (driven by the PIO kick)                              */

static void gpmi_ecc_xfer(MxsGpmiState *s)
{
    bool encode = (s->eccctrl & GPMI_ECCCTRL_ECC_CMD) ==
                  GPMI_ECCCTRL_ECC_ENCODE;
    uint32_t count = s->ecccount & 0xffff;
    uint8_t *page;
    uint8_t status = BCH_STATUS_OK;

    if (count == 0) {
        count = NAND_PAGE_TOTAL;
    }
    if (count > NAND_PAGE_TOTAL) {
        count = NAND_PAGE_TOTAL;
    }

    page = nand_page_ptr(s, s->row);
    if (!page) {
        mxs_bch_complete(s, BCH_STATUS_UNCORRECTABLE);
        return;
    }

    if (!encode) {
        /* BCH decode (read page): NAND page -> PAYLOAD (data) and
         * AUXILIARY (OOB) guest buffers. */
        uint32_t data_len = count > NAND_PAGE_DATA ? NAND_PAGE_DATA
                                                   : count;
        uint32_t oob_len = count > NAND_PAGE_DATA
                               ? count - NAND_PAGE_DATA : 0;
        if (nand_page_erased(page)) {
            status = BCH_STATUS_ERASED;
        }
        if (data_len) {
            address_space_write(&address_space_memory, s->payload,
                                MEMTXATTRS_UNSPECIFIED, page, data_len);
        }
        if (oob_len) {
            address_space_write(&address_space_memory, s->auxiliary,
                                MEMTXATTRS_UNSPECIFIED,
                                page + NAND_PAGE_DATA, oob_len);
        }
        /* block-mark swap: the byte at the block-mark offset in the
         * data buffer is swapped with OOB[0] (the true bad-block
         * mark).  The block-mark byte offset for a 2K page with
         * 4-bit ECC is 2018 (see the research notes); swapping the
         * first OOB byte with data[2018] keeps the caller's OOB[0]
         * correct. */
        if (data_len > 2018 && oob_len > 0) {
            uint8_t tmp;
            MemTxResult r;
            r = address_space_read(&address_space_memory, s->payload + 2018,
                                   MEMTXATTRS_UNSPECIFIED, &tmp, 1);
            if (r == MEMTX_OK) {
                uint8_t oob0;
                r = address_space_read(&address_space_memory, s->auxiliary,
                                       MEMTXATTRS_UNSPECIFIED, &oob0, 1);
                if (r == MEMTX_OK) {
                    address_space_write(&address_space_memory,
                                        s->payload + 2018,
                                        MEMTXATTRS_UNSPECIFIED, &oob0, 1);
                    address_space_write(&address_space_memory,
                                        s->auxiliary,
                                        MEMTXATTRS_UNSPECIFIED, &tmp, 1);
                }
            }
        }
    } else {
        /* BCH encode (write page): PAYLOAD (data) and AUXILIARY
         * (OOB) guest buffers -> NAND page. */
        uint32_t data_len = count > NAND_PAGE_DATA ? NAND_PAGE_DATA
                                                   : count;
        uint32_t oob_len = count > NAND_PAGE_DATA
                               ? count - NAND_PAGE_DATA : 0;
        if (data_len) {
            address_space_read(&address_space_memory, s->payload,
                               MEMTXATTRS_UNSPECIFIED, page, data_len);
        }
        if (oob_len) {
            address_space_read(&address_space_memory, s->auxiliary,
                               MEMTXATTRS_UNSPECIFIED,
                               page + NAND_PAGE_DATA, oob_len);
        }
    }

    s->stat |= GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY;
    s->stat &= ~GPMI_STAT_FIFO_FULL;
    mxs_bch_complete(s, status);
}

/* ------------------------------------------------------------------ */
/* GPMI command kick (PIO word -1, or a CPU CTRL0 write with RUN)    */

static void gpmi_kick(MxsGpmiState *s)
{
    uint32_t mode = s->ctrl0 & GPMI_CTRL0_COMMAND_MODE;

    s->ctrl0 &= ~GPMI_CTRL0_RUN;

    switch (mode) {
    case GPMI_CTRL0_CMD_WAIT_READY:
        /* the NAND is always ready in this synchronous model */
        s->stat |= GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY |
                   (1u << 24);   /* READY_BUSY bit 24 (chip 0) */
        break;
    case GPMI_CTRL0_CMD_READ_COMPARE:
        /* used for status polling; the data phase (if any) is
         * handled by dma_data, and the compare itself is a no-op
         * here (the status byte read through the normal path is
         * already correct). */
        s->stat |= GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY;
        break;
    case GPMI_CTRL0_CMD_READ:
    case GPMI_CTRL0_CMD_WRITE:
    default:
        if (s->eccctrl & GPMI_ECCCTRL_ENABLE_ECC) {
            /* an ECC transfer: the whole page moves now, driven by
             * the PIO-programmed PAYLOAD/AUXILIARY pointers */
            gpmi_ecc_xfer(s);
        } else {
            /* a raw transfer: the byte stream is moved by the
             * following dma_data call (or has already been fed);
             * mark the controller ready */
            s->stat |= GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY;
        }
        break;
    }

    /* the command completed: raise the device IRQ if enabled */
    if (s->ctrl0 & GPMI_CTRL0_DEV_IRQ_EN) {
        s->ctrl1 |= GPMI_CTRL1_DEV_IRQ;
        mxs_gpmi_irq_update(s);
    }
}

/* ------------------------------------------------------------------ */
/* DMA client hooks (APBH channels 4-11 = GPMI0-7)                    */

int mxs_gpmi_dma_pio(MxsGpmiState *s, int word_idx, uint32_t value)
{
    if (word_idx < 0) {
        /* descriptor complete: execute the programmed command */
        gpmi_kick(s);
        return 1;
    }

    switch (word_idx) {
    case 0:
        s->ctrl0 = value & ~GPMI_CTRL0_RUN;
        break;
    case 1:
        s->compare = value;
        break;
    case 2:
        s->eccctrl = value;
        break;
    case 3:
        s->ecccount = value;
        break;
    case 4:
        s->payload = value;
        break;
    case 5:
        s->auxiliary = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-gpmi: dma pio word %d (0x%08x) ignored\n",
                      word_idx, value);
        break;
    }
    return 0;
}

int mxs_gpmi_dma_data(MxsGpmiState *s, bool to_device,
                      dma_addr_t bufaddr, uint32_t len)
{
    uint8_t *buf = g_malloc(len ? len : 1);
    MemTxResult res = MEMTX_OK;

    if (to_device) {
        /* memory -> peripheral: command / address / program data */
        res = address_space_read(&address_space_memory, bufaddr,
                                 MEMTXATTRS_UNSPECIFIED, buf, len);
        if (res == MEMTX_OK) {
            uint32_t mode = s->ctrl0 & GPMI_CTRL0_COMMAND_MODE;
            uint32_t addr = s->ctrl0 & GPMI_CTRL0_ADDRESS;
            if (mode == GPMI_CTRL0_CMD_WRITE &&
                addr == GPMI_CTRL0_ADDR_DATA &&
                s->nand_state == NAND_ST_DATA_IN) {
                /* program data bytes */
                for (uint32_t i = 0; i < len; i++) {
                    nand_feed_byte(s, buf[i]);
                }
            } else {
                /* command / address bytes */
                for (uint32_t i = 0; i < len; i++) {
                    nand_feed_byte(s, buf[i]);
                }
            }
        }
    } else {
        /* peripheral -> memory: read data (page / ID / ONFI / status) */
        uint32_t n = len;
        if (n > s->read_len - s->read_pos) {
            n = s->read_len - s->read_pos;
        }
        if (n) {
            memcpy(buf, s->read_buf + s->read_pos, n);
            s->read_pos += n;
            s->data_remaining = s->read_len - s->read_pos;
            if (s->read_pos >= s->read_len) {
                s->data_read_pending = false;
                s->nand_state = NAND_ST_IDLE;
            }
        }
        /* pad any shortfall with 0xFF (erased) so the guest never
         * sees stale data */
        if (n < len) {
            memset(buf + n, 0xff, len - n);
        }
        res = address_space_write(&address_space_memory, bufaddr,
                                  MEMTXATTRS_UNSPECIFIED, buf, len);
    }

    g_free(buf);
    return res == MEMTX_OK ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* CPU register access                                                */

static uint64_t mxs_gpmi_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsGpmiState *s = opaque;
    uint32_t reg = addr & ~0xf;

    switch (reg) {
    case GPMI_CTRL0:
        return s->ctrl0;
    case GPMI_COMPARE:
        return s->compare;
    case GPMI_ECCCTRL:
        return s->eccctrl;
    case GPMI_ECCCOUNT:
        return s->ecccount;
    case GPMI_PAYLOAD:
        return s->payload;
    case GPMI_AUXILIARY:
        return s->auxiliary;
    case GPMI_CTRL1:
        return s->ctrl1;
    case GPMI_TIMING0:
        return s->timing0;
    case GPMI_TIMING1:
        return s->timing1;
    case GPMI_TIMING2:
        return s->timing2;
    case GPMI_DATA:
        /* PIO data port: return the next read byte (if any) */
        if (s->read_pos < s->read_len) {
            return s->read_buf[s->read_pos++];
        }
        return 0xff;
    case GPMI_STAT:
        return s->stat | GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY |
               (1u << 24);   /* READY_BUSY chip 0 */
    case GPMI_DEBUG:
        return s->debug;
    case GPMI_VERSION:
        return s->version;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-gpmi: unimplemented read 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void mxs_gpmi_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    MxsGpmiState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;
    uint32_t *slot = NULL;

    switch (reg) {
    case GPMI_CTRL0:    slot = &s->ctrl0; break;
    case GPMI_COMPARE:  slot = &s->compare; break;
    case GPMI_ECCCTRL:  slot = &s->eccctrl; break;
    case GPMI_ECCCOUNT: slot = &s->ecccount; break;
    case GPMI_PAYLOAD:  slot = &s->payload; break;
    case GPMI_AUXILIARY: slot = &s->auxiliary; break;
    case GPMI_CTRL1:    slot = &s->ctrl1; break;
    case GPMI_TIMING0:  slot = &s->timing0; break;
    case GPMI_TIMING1:  slot = &s->timing1; break;
    case GPMI_TIMING2:  slot = &s->timing2; break;
    case GPMI_STAT:     slot = &s->stat; break;
    case GPMI_DEBUG:    slot = &s->debug; break;
    case GPMI_VERSION:  return;   /* read-only */
    case GPMI_DATA:
        /* PIO data port: a write is a command/address/data byte */
        nand_feed_byte(s, value & 0xff);
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-gpmi: unimplemented write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, value);
        return;
    }

    switch (off) {
    case 0:
        *slot = value;
        break;
    case 4:
        *slot |= value;
        break;
    case 8:
        *slot &= ~value;
        break;
    case 12:
        *slot ^= value;
        break;
    }

    /* SFTRST rising edge -> clock gates once (stmp handshake) */
    if (reg == GPMI_CTRL0) {
        if ((value & GPMI_CTRL0_SFTRST) && off == 0) {
            s->ctrl0 |= GPMI_CTRL0_CLKGATE;
        }
        if (value & GPMI_CTRL0_RUN) {
            gpmi_kick(s);
        }
    }
    if (reg == GPMI_CTRL1) {
        mxs_gpmi_irq_update(s);
    }
}

static const MemoryRegionOps mxs_gpmi_ops = {
    .read = mxs_gpmi_read,
    .write = mxs_gpmi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static uint64_t mxs_bch_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsGpmiState *s = opaque;
    uint32_t reg = addr & ~0xf;

    switch (reg) {
    case BCH_CTRL:
        return s->bch_ctrl;
    case BCH_STATUS0:
        return s->bch_status0;
    case BCH_MODE:
        return s->bch_mode;
    case BCH_ENCODEPTR:
        return s->bch_encodeptr;
    case BCH_DATAPTR:
        return s->bch_dataptr;
    case BCH_METAPTR:
        return s->bch_metaptr;
    case BCH_LAYOUTSELECT:
        return s->bch_layoutselect;
    case BCH_FLASH0LAYOUT0: case BCH_FLASH0LAYOUT0 + 0x20:
    case BCH_FLASH0LAYOUT0 + 0x40: case BCH_FLASH0LAYOUT0 + 0x60:
        return s->bch_layout[(reg - BCH_FLASH0LAYOUT0) / 0x20][0];
    case BCH_FLASH0LAYOUT1: case BCH_FLASH0LAYOUT1 + 0x20:
    case BCH_FLASH0LAYOUT1 + 0x40: case BCH_FLASH0LAYOUT1 + 0x60:
        return s->bch_layout[(reg - BCH_FLASH0LAYOUT1) / 0x20][1];
    case BCH_VERSION:
        return s->bch_version;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-bch: unimplemented read 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void mxs_bch_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    MxsGpmiState *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;
    uint32_t *slot = NULL;

    switch (reg) {
    case BCH_CTRL:        slot = &s->bch_ctrl; break;
    case BCH_MODE:        slot = &s->bch_mode; break;
    case BCH_ENCODEPTR:   slot = &s->bch_encodeptr; break;
    case BCH_DATAPTR:     slot = &s->bch_dataptr; break;
    case BCH_METAPTR:     slot = &s->bch_metaptr; break;
    case BCH_LAYOUTSELECT: slot = &s->bch_layoutselect; break;
    case BCH_FLASH0LAYOUT0: case BCH_FLASH0LAYOUT0 + 0x20:
    case BCH_FLASH0LAYOUT0 + 0x40: case BCH_FLASH0LAYOUT0 + 0x60:
        slot = &s->bch_layout[(reg - BCH_FLASH0LAYOUT0) / 0x20][0];
        break;
    case BCH_FLASH0LAYOUT1: case BCH_FLASH0LAYOUT1 + 0x20:
    case BCH_FLASH0LAYOUT1 + 0x40: case BCH_FLASH0LAYOUT1 + 0x60:
        slot = &s->bch_layout[(reg - BCH_FLASH0LAYOUT1) / 0x20][1];
        break;
    case BCH_STATUS0:
    case BCH_VERSION:
        return;   /* status / version are read-only */
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mxs-bch: unimplemented write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", addr, value);
        return;
    }

    switch (off) {
    case 0:
        *slot = value;
        break;
    case 4:
        *slot |= value;
        break;
    case 8:
        *slot &= ~value;
        break;
    case 12:
        *slot ^= value;
        break;
    }

    if (reg == BCH_CTRL) {
        /* writing COMPLETE_IRQ to CTRL_CLR clears it (the driver
         * polls COMPLETE_IRQ then clears it this way) */
        if (off == 8 && (value & BCH_CTRL_COMPLETE_IRQ)) {
            s->bch_ctrl &= ~BCH_CTRL_COMPLETE_IRQ;
        }
        /* BCH reset erratum (MX28): a full SFTRST breaks the block;
         * accept the "clear CLKGATE only" bring-up the Linux driver
         * uses, and treat a full reset as a normal reset here. */
        mxs_bch_irq_update(s);
    }
}

static const MemoryRegionOps mxs_bch_ops = {
    .read = mxs_bch_read,
    .write = mxs_bch_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* reset / init                                                       */

static void mxs_gpmi_reset(DeviceState *dev)
{
    MxsGpmiState *s = MXS_GPMI(dev);

    s->ctrl0 = s->compare = s->eccctrl = s->ecccount = 0;
    s->payload = s->auxiliary = 0;
    s->ctrl1 = s->timing0 = s->timing1 = s->timing2 = 0;
    s->stat = GPMI_STAT_PRESENT | GPMI_STAT_FIFO_EMPTY;
    s->debug = 0;
    s->bch_ctrl = s->bch_status0 = s->bch_mode = 0;
    s->bch_encodeptr = s->bch_dataptr = s->bch_metaptr = 0;
    s->bch_layoutselect = 0;
    memset(s->bch_layout, 0, sizeof(s->bch_layout));

    s->nand_state = NAND_ST_IDLE;
    s->cmd = 0;
    s->addr_len = s->addr_want = 0;
    s->col = s->row = 0;
    s->read_len = s->read_pos = 0;
    s->prog_len = 0;
    s->prog_data = false;
    s->data_read_pending = false;
    s->data_remaining = 0;

    mxs_gpmi_irq_update(s);
    mxs_bch_irq_update(s);
}

static void mxs_gpmi_init(Object *obj)
{
    MxsGpmiState *s = MXS_GPMI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_gpmi_ops, s,
                          "mxs-gpmi", 0x4000);
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->bch_iomem, obj, &mxs_bch_ops, s,
                          "mxs-bch", 0x2000);
    sysbus_init_mmio(sbd, &s->bch_iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->bch_irq);

    /* NAND backing store: erased (0xFF) */
    s->nand = g_malloc0(NAND_TOTAL_BYTES);
    memset(s->nand, 0xff, NAND_TOTAL_BYTES);
    nand_build_onfi(s->onfi);

    /* valid FCB at block 0, replicated to blocks 1-3 (kobs-ng) */
    for (int blk = 0; blk < 4; blk++) {
        uint8_t *page = nand_page_ptr(s, blk * NAND_PAGES_BLOCK);
        if (page) {
            nand_build_fcb(page);
        }
    }

    /* VERSION values: the driver was not observed reading them, and
     * the exact i.MX28 values were not verified from a primary
     * source -- report a plausible MAJOR/MINOR and flag as
     * unverified in the header comment. */
    s->version = (3u << 24) | (0u << 16) | 0u;      /* GPMI 3.0 */
    s->bch_version = (2u << 24) | (0u << 16) | 0u;  /* BCH 2.0 */
}

static void mxs_gpmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_gpmi_reset);
}

static const TypeInfo mxs_gpmi_info = {
    .name = TYPE_MXS_GPMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsGpmiState),
    .instance_init = mxs_gpmi_init,
    .class_init = mxs_gpmi_class_init,
};

static void mxs_gpmi_register(void)
{
    type_register_static(&mxs_gpmi_info);
}
type_init(mxs_gpmi_register)
