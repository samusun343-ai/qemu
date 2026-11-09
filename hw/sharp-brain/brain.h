/*
 * Sharp Brain (i.MX28) SoC definitions.
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */

#ifndef HW_SHARP_BRAIN_BRAIN_H
#define HW_SHARP_BRAIN_BRAIN_H

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "target/arm/cpu.h"
#include "hw/sd/sd.h"
#include "system/dma.h"
#include "qom/object.h"

/* ------------------------------------------------------------------ */
/* i.MX28 memory map                                                  */

#define IMX28_IRAM_BASE        0x00000000
#define IMX28_IRAM_SIZE        (128 * KiB)
#define IMX28_DRAM_BASE        0x40000000
#define IMX28_DRAM_SIZE        (128 * MiB)
#define IMX28_PERIPH_BASE      0x80000000

#define IMX28_ICOLL_BASE       0x80000000
#define IMX28_APBH_DMA_BASE    0x80004000
#define IMX28_SSP0_BASE        0x80010000
#define IMX28_SSP1_BASE        0x80012000
#define IMX28_SSP2_BASE        0x80014000
#define IMX28_SSP3_BASE        0x80016000
#define IMX28_PINCTRL_BASE     0x80018000
#define IMX28_DIGCTL_BASE      0x8001c000
#define IMX28_APBX_DMA_BASE    0x80024000
#define IMX28_LCDIF_BASE       0x80030000
#define IMX28_BOOTROM_MAILBOX  0x8003c000      /* SIMDBG block (unused) */
#define IMX28_CLKCTRL_BASE     0x80040000
#define IMX28_POWER_BASE       0x80044000
#define IMX28_LRADC_BASE       0x80050000
#define IMX28_TIMROT_BASE      0x80068000
#define IMX28_DUART_BASE       0x80074000
#define IMX28_EMI_BASE         0x800e0000
#define IMX28_OCOTP_BASE       0x8002c000

/* IRQ numbers (i.MX28 ICOLL, from linux-brain imx28.dtsi) */
#define IMX28_IRQ_GPIO0        127
#define IMX28_IRQ_GPIO1        126
#define IMX28_IRQ_GPIO2        125
#define IMX28_IRQ_GPIO3        124
#define IMX28_IRQ_GPIO4        123
#define IMX28_IRQ_LRADC_TOUCH  10
#define IMX28_IRQ_LRADC_THRESH0 14
#define IMX28_IRQ_LRADC_THRESH1 15
#define IMX28_IRQ_LRADC_CH(n)  (16 + (n))
#define IMX28_IRQ_LCDIF        38
#define IMX28_IRQ_DUART        47
#define IMX28_IRQ_TIMROT0      48
#define IMX28_IRQ_TIMROT1      49
#define IMX28_IRQ_TIMROT2      50
#define IMX28_IRQ_TIMROT3      51
#define IMX28_IRQ_APBH_SSP0    82
#define IMX28_IRQ_APBH_SSP1    83
#define IMX28_IRQ_APBH_SSP2    84
#define IMX28_IRQ_APBH_SSP3    85
#define IMX28_IRQ_APBH_GPMI    88
#define IMX28_IRQ_APBH_HSADC   87
#define IMX28_IRQ_APBH_LCDIF   86
#define IMX28_IRQ_SSP0         96
#define IMX28_IRQ_SSP1         97
#define IMX28_NUM_IRQS         128

/* ------------------------------------------------------------------ */
/* device types                                                       */

#define TYPE_IMX28 "imx28"
OBJECT_DECLARE_SIMPLE_TYPE(Imx28State, IMX28)

#define TYPE_MXS_ICOLL "mxs-icoll"

struct Imx28State {
    DeviceState parent_obj;

    ARMCPU cpu;
    DeviceState *icoll;

    DeviceState *apbh_dma;

    DeviceState *ssp[2];        /* SSP0 = eMMC, SSP1 = microSD */

    DeviceState *digctl;
    DeviceState *clkctrl;
    DeviceState *timrot;
    DeviceState *pinctrl;
    DeviceState *power;
    DeviceState *lradc;        /* real LRADC + touch (lradc.c) */
    DeviceState *ocotp;
    DeviceState *emi;          /* DDR controller stub */
    DeviceState *lcdif;
    DeviceState *i2c[2];        /* I2C0: sub-MCU bus, I2C1 */
    DeviceState *duart;         /* PL011 */

    MemoryRegion iram;
    BlockBackend *emmc_blk;     /* eMMC backend, used by the boot ROM */
    char *cpu_type;
};

/* ------------------------------------------------------------------ */
/* APBH DMA (hw/sharp-brain/apbh-dma.c)                               */

#define MXS_DMA_CHANNELS 16

#define TYPE_MXS_DMA "mxs-dma"
OBJECT_DECLARE_SIMPLE_TYPE(MxsDmaState, MXS_DMA)

struct MxsDmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[MXS_DMA_CHANNELS];

    /* register file */
    uint32_t ctrl0, ctrl1, ctrl2, channel_ctrl;
    uint32_t nxctmdar[MXS_DMA_CHANNELS];
    uint32_t sema[MXS_DMA_CHANNELS];
    uint32_t bar[MXS_DMA_CHANNELS];

    /* typed client pointers (same directory, shared header) */
    struct MxsSspState *ssp[4];         /* channels 0-3 */
    struct MxsLcdifPriv *lcdif;         /* channel 13 */
};

/* ------------------------------------------------------------------ */
/* SSP (hw/sharp-brain/ssp.c)                                         */

#define TYPE_MXS_SSP "mxs-ssp"
OBJECT_DECLARE_SIMPLE_TYPE(MxsSspState, MXS_SSP)

struct MxsSspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;

    uint32_t ctrl0, cmd0, cmd1, xfer_size, block_size;
    uint32_t compref, compmask, timing, ctrl1, status, ddr_ctrl;
    uint32_t sdresp[4];
    uint32_t version;

    /* data FIFO state */
    uint32_t fifo[32];
    uint32_t fifo_level;        /* words available */
    bool data_dir_read;
    uint32_t data_remaining;
};

/* called by the DMA engine (channel n = SSP n) */
int mxs_ssp_dma_pio(MxsSspState *s, int word_idx, uint32_t value);
int mxs_ssp_dma_data(MxsSspState *s, bool to_device,
                     dma_addr_t bufaddr, uint32_t len);

/* ------------------------------------------------------------------ */
/* LCDIF (hw/sharp-brain/lcdif.c)                                     */

#define TYPE_MXS_LCDIF "mxs-lcdif"
OBJECT_DECLARE_SIMPLE_TYPE(MxsLcdifPriv, MXS_LCDIF)

/* ILI9805 panel geometry (native portrait orientation) */
#define ILI9805_NATIVE_WIDTH   480
#define ILI9805_NATIVE_HEIGHT  854

struct MxsLcdifPriv {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QemuConsole *con;

    /* LCDIF registers */
    uint32_t ctrl, ctrl1, ctrl2;
    uint32_t transfer_count, cur_buf, next_buf, timing;
    uint32_t vdctrl[4], dvi[4], csc[4];

    /* CPU-mode transfer countdown */
    uint32_t cpu_pending;

    /* ILI9805 smart-panel state */
    uint16_t *gram;
    bool panel_on, write_active, dirty;
    uint8_t last_cmd, cmd_pos, mac, pixel_fmt;
    uint8_t cmd_bytes[8];
    uint32_t col_start, col_end, page_start, page_end;
    uint32_t gram_x, gram_y;

    /* console geometry (landscape view) */
    uint32_t width, height;
};

/* DMA channel 13 client */
int mxs_lcdif_dma_data(struct MxsLcdifPriv *s, bool to_device,
                       dma_addr_t bufaddr, uint32_t len);

/* ------------------------------------------------------------------ */
/* misc blocks                                                        */

#define TYPE_MXS_DIGCTL "mxs-digctl"
#define TYPE_MXS_CLKCTRL "mxs-clkctrl"
#define TYPE_MXS_TIMROT "mxs-timrot"
#define TYPE_MXS_PINCTRL "mxs-pinctrl"
#define TYPE_MXS_POWER "mxs-power"
#define TYPE_MXS_OCOTP "mxs-ocotp"
#define TYPE_MXS_EMI "mxs-emi"
#define TYPE_MXS_I2C "mxs-i2c"

/* ------------------------------------------------------------------ */
/* LRADC (hw/sharp-brain/lradc.c): ADC + 4-wire resistive touch        */

#define MXS_LRADC_NUM_CH 8

#define TYPE_MXS_LRADC "mxs-lradc"
OBJECT_DECLARE_SIMPLE_TYPE(MxsLradcState, MXS_LRADC)

/*
 * IRQ lines in device-tree order: touchscreen, thresh0, thresh1,
 * channel0..7, button0, button1.
 */
#define MXS_LRADC_NUM_IRQ 13

struct MxsLradcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irqs[MXS_LRADC_NUM_IRQ];

    uint32_t ctrl0, ctrl1, ctrl2, ctrl3, ctrl4;
    uint32_t status;
    uint32_t ch[MXS_LRADC_NUM_CH];
    uint32_t delay[4];
    uint32_t conversion;
    uint32_t threshold[2];

    /* touchscreen state */
    bool touch_down;
    int x, y;                    /* 0..max */
    uint32_t x_max, y_max;
};

/* public: inject external GPIO input levels (keyboard, card detect) */
void mxs_pinctrl_set_input(DeviceState *dev, int bank, uint32_t mask,
                           uint32_t value);

/* boot ROM emulation (hw/sharp-brain/bootrom.c) */
void brain_bootrom_setup(Imx28State *s, BlockBackend *blk);

#endif /* HW_SHARP_BRAIN_BRAIN_H */
