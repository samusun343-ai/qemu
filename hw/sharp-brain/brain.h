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
#define IMX28_BCH_BASE         0x8000a000
#define IMX28_GPMI_BASE        0x8000c000
#define IMX28_SSP0_BASE        0x80010000
#define IMX28_SSP1_BASE        0x80012000
#define IMX28_SSP2_BASE        0x80014000
#define IMX28_SSP3_BASE        0x80016000
#define IMX28_PINCTRL_BASE     0x80018000
#define IMX28_DIGCTL_BASE      0x8001c000
#define IMX28_APBX_DMA_BASE    0x80024000
#define IMX28_OCOTP_BASE       0x8002c000
#define IMX28_LCDIF_BASE       0x80030000
#define IMX28_BOOTROM_MAILBOX  0x8003c000      /* SIMDBG block (unused) */
#define IMX28_CLKCTRL_BASE     0x80040000
#define IMX28_POWER_BASE       0x80044000
#define IMX28_LRADC_BASE       0x80050000
#define IMX28_RTC_BASE         0x80056000
#define IMX28_I2C0_BASE        0x80058000
#define IMX28_I2C1_BASE        0x8005a000
#define IMX28_PWM_BASE         0x80064000
#define IMX28_TIMROT_BASE      0x80068000
#define IMX28_DUART_BASE       0x80074000
#define IMX28_USBPHY0_BASE     0x8007c000
#define IMX28_USBPHY1_BASE     0x8007e000
#define IMX28_USB0_BASE        0x80080000
#define IMX28_EMI_BASE         0x800e0000

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
#define IMX28_IRQ_LRADC_BTN0   24
#define IMX28_IRQ_LRADC_BTN1   25
#define IMX28_IRQ_RTC          29
#define IMX28_IRQ_LCDIF        38
#define IMX28_IRQ_BCH          41
#define IMX28_IRQ_DUART        47
#define IMX28_IRQ_TIMROT0      48
#define IMX28_IRQ_TIMROT1      49
#define IMX28_IRQ_TIMROT2      50
#define IMX28_IRQ_TIMROT3      51
#define IMX28_IRQ_APBH_SSP0    82
#define IMX28_IRQ_APBH_SSP1    83
#define IMX28_IRQ_APBH_SSP2    84
#define IMX28_IRQ_APBH_SSP3    85
#define IMX28_IRQ_APBH_LCDIF   86
#define IMX28_IRQ_APBH_HSADC   87
#define IMX28_IRQ_APBH_GPMI    88
#define IMX28_IRQ_USB0         93  /* imx28.dtsi usb0 (usb1 = 92) */
#define IMX28_IRQ_SSP0         96
#define IMX28_IRQ_SSP1         97
#define IMX28_IRQ_I2C1         110
#define IMX28_IRQ_I2C0         111
#define IMX28_NUM_IRQS         128

/* blocks modelled as plain stmp register files with the SFTRST/CLKGATE
 * handshake (misc.c TYPE_MXS_STMP): AUART0-4, SAIF0/1, SPDIF,
 * AUDIOOUT/IN, DCP, PXP */
#define IMX28_NUM_STMP_BLOCKS 12

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
    DeviceState *apbx_dma;

    DeviceState *ssp[4];        /* SSP0 = eMMC, SSP1 = microSD,
                                 * SSP2 = SPI sub-LCD / sub-MCU, SSP3 */
    DeviceState *gpmi;          /* GPMI NAND controller + BCH ECC */

    DeviceState *digctl;
    DeviceState *clkctrl;
    DeviceState *timrot;
    DeviceState *pinctrl;
    DeviceState *power;
    DeviceState *lradc;        /* real LRADC + touch (lradc.c) */
    DeviceState *ocotp;
    DeviceState *emi;          /* DDR controller stub */
    DeviceState *rtc;          /* stmp3xxx RTC block */
    DeviceState *pwm;          /* PWM block w/ channel-present bits */
    DeviceState *usbphy[2];    /* USB PHY0/1 */
    DeviceState *usb_otg;      /* USB0 OTG controller stub */
    DeviceState *stmp[IMX28_NUM_STMP_BLOCKS]; /* AUART/SAIF/SPDIF/... */
    DeviceState *lcdif;
    DeviceState *i2c[2];        /* I2C0: sub-MCU bus, I2C1 */
    DeviceState *duart;         /* PL011 */

    MemoryRegion iram;
    uint8_t *iram_buf;          /* backing store for the IRAM regions */
    /*
     * SIMDBG block (0x8003c004..0x80040000) as persistent storage:
     * the EBOOT suspend/resume path parks scratch state there across
     * the power-cycle reset (see imx28.c).
     */
    MemoryRegion simdbg;
    /*
     * MMIO "mirror" of the IRAM at PA 0xa8000000 (the WinCE uncached
     * alias of IRAM interpreted as a physical address).  The DVFS
     * clock-change trampolines run at the uncached VA and toggle the
     * MMU; real hardware keeps executing the next (already
     * prefetched) instruction from the pipeline, while QEMU re-fetches
     * it under the new MMU state - i.e. at PA 0xa8003xxx.  The mirror
     * makes those fetches succeed, and its read handler redirects the
     * trampoline's return-jump target from the PA-identity address to
     * the uncached VA, where instruction fetches work with the MMU
     * both on (IRAM 1MB section) and off (this mirror).
     */
    MemoryRegion iram_mirror;
    BlockBackend *emmc_blk;     /* eMMC backend, used by the boot ROM */
    char *cpu_type;
};

/* ------------------------------------------------------------------ */
/* APBH/APBX DMA (hw/sharp-brain/apbh-dma.c)                          */

#define MXS_DMA_CHANNELS 16

/*
 * APBX channel map (linux imx28.dtsi dma-apbx node):
 *   0,1   auart4 rx/tx   2 spdif   3 (none)
 *   4,5   saif0/1         6,7 i2c0/1
 *   8..15 auart0..3 rx/tx
 */
#define MXS_APBX_CH_I2C0 6
#define MXS_APBX_CH_I2C1 7

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

    bool is_apbx;                /* APBX channel map and clients */

    /* typed client pointers (same directory, shared header) */
    struct MxsSspState *ssp[4];         /* APBH channels 0-3 */
    struct MxsGpmiState *gpmi;          /* APBH channels 4-11 */
    struct MxsLcdifPriv *lcdif;         /* APBH channel 13 */
    struct MxsI2cState *i2c[2];         /* APBX channels 6-7 */
};

/* ------------------------------------------------------------------ */
/* SSP (hw/sharp-brain/ssp.c)                                         */

#define TYPE_MXS_SSP "mxs-ssp"
OBJECT_DECLARE_SIMPLE_TYPE(MxsSspState, MXS_SSP)

#define SSP_FIFO_WORDS 32

struct MxsSspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    uint8_t id;                 /* 0..3, for trace messages */

    uint32_t ctrl0, cmd0, cmd1, xfer_size, block_size;
    uint32_t compref, compmask, timing, ctrl1, status, ddr_ctrl, dll_ctrl;
    uint32_t sdresp[4];
    uint32_t version;

    /* data FIFO state */
    uint32_t fifo[SSP_FIFO_WORDS];
    uint32_t fifo_level;        /* words available */
    bool data_dir_read;
    uint32_t data_remaining;
};

/* called by the DMA engine (channel n = SSP n) */
int mxs_ssp_dma_pio(MxsSspState *s, int word_idx, uint32_t value);
int mxs_ssp_dma_data(MxsSspState *s, bool to_device,
                     dma_addr_t bufaddr, uint32_t len);

/* ------------------------------------------------------------------ */
/* GPMI NAND controller + BCH ECC (hw/sharp-brain/gpmi.c)             */

#define TYPE_MXS_GPMI "mxs-gpmi"
OBJECT_DECLARE_SIMPLE_TYPE(MxsGpmiState, MXS_GPMI)

/* called by the DMA engine (APBH channels 4-11 = GPMI0-7) */
int mxs_gpmi_dma_pio(MxsGpmiState *s, int word_idx, uint32_t value);
int mxs_gpmi_dma_data(MxsGpmiState *s, bool to_device,
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

    /* periodic VSYNC (panel refresh): the real LCDIF raises a VSYNC
     * edge each frame (panel timing), and the guest display driver
     * waits for it before pushing the next frame.  Without a periodic
     * VSYNC the driver stalls after the initial pushes (it clears the
     * VSYNC bit and waits for a new edge that only a transfer
     * completion would set). */
    QEMUTimer *vsync_timer;

    /* Last master-mode frame (for the VSYNC dirty re-push): the
     * shell draws desktop content into the framebuffer directly and
     * the driver only pushes on present, so a frame can be drawn
     * after the driver's last push and never reach the panel.  The
     * model tracks the FULL framebuffer (from the full-frame push)
     * and, on VSYNC, re-checks it; if it changed since it was
     * pushed, the full frame is re-pushed so the console tracks the
     * framebuffer.  (Tracking only the last pushed region is not
     * enough: the last push is often the small menu-bar strip while
     * the icons land in the main area.) */
    hwaddr full_frame_buf;
    uint32_t full_frame_crc;
    int vsync_repush_counter;

    /* debug: remaining DATA writes to log after a RAMWR */
    int dbg_data_log;
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
#define TYPE_MXS_RTC "mxs-rtc"
#define TYPE_MXS_USBPHY "mxs-usbphy"
#define TYPE_MXS_PWM "mxs-pwm"
#define TYPE_MXS_STMP "mxs-stmp"
#define TYPE_MXS_PXP "mxs-pxp"
#define TYPE_MXS_USB_OTG "mxs-usb-otg"
#define TYPE_BU26154 "bu26154"

/* ROHM BU26154 audio codec: I2C slave address 0x1A (7-bit, SAD pin
 * = "L"; 0x1B if SAD = "H").  Datasheet TSZ22111 Rev.002.  The
 * codec lives on I2C0 (brainux imx28-brain.dtsi codec node under
 * i2c@80058000). */
#define BU26154_I2C_ADDR 0x1a

/* ------------------------------------------------------------------ */
/* I2C (hw/sharp-brain/i2c.c)                                         */

OBJECT_DECLARE_SIMPLE_TYPE(MxsI2cState, MXS_I2C)

struct MxsI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    I2CBus *bus;

    uint32_t ctrl0, ctrl1, stat;
    uint32_t timing0, timing1, timing2;
    uint32_t queuectrl, queueend, queuecmd, queuedata;
    uint32_t data[4];

    /* a RUN programmed through a DMA PIO descriptor waits for the
     * data descriptor before the synchronous transfer executes */
    bool run_pending;
};

/* DMA client hooks (channels 6/7 of the APBX DMA) */
void mxs_i2c_dma_pio(MxsI2cState *s, int word_idx, uint32_t value);
void mxs_i2c_dma_data(MxsI2cState *s, bool to_device,
                      dma_addr_t bufaddr, uint32_t len);

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

    /* microSD slot card-detect level (LRADC channel 5) */
    bool sd_card_present;
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

/* public: re-arm the Brain keyboard column IRQs after the guest
 * KBD driver tears them down (see pinctrl.c). */
void mxs_pinctrl_rearm_kbd_irq(DeviceState *dev);

/*
 * Public: observe GPIO output changes.
 *
 * The matrix keyboard derives the column levels from the row drive
 * state in the pinctrl DOUT/DOE registers, so it needs to be
 * notified whenever the guest re-drives the row pins (GPIO4).  The
 * callback receives the new "effective output" value (DOUT & DOE)
 * of the bank that changed.
 */
typedef void MxsGpioOutNotify(void *opaque, int bank, uint32_t out);

void mxs_pinctrl_set_output_hook(DeviceState *dev, MxsGpioOutNotify fn,
                                  void *opaque);
uint32_t mxs_pinctrl_get_output(DeviceState *dev, int bank);
uint32_t mxs_pinctrl_get_doe(DeviceState *dev, int bank);

/* ------------------------------------------------------------------ */
/* GPIO matrix keyboard (hw/sharp-brain/brain-kbd.c)                  */

#define BRAIN_KBD_ROWS 8
#define BRAIN_KBD_COLS 7

#define TYPE_BRAIN_KBD "brain-kbd"
OBJECT_DECLARE_SIMPLE_TYPE(BrainKbdState, BRAIN_KBD)

/*
 * Host key -> matrix contact mapping.  Tables are indexed by linux
 * keycode (KEY_CNT entries); the QEMU input core delivers host key
 * events as linux keycodes.
 *
 * The tables only list keys that exist on the Brain; the entries
 * for all other host keys are zero-filled, so flags carries the
 * validity of the mapping: flags == 0 means "unmapped" and the
 * entry is ignored.  (Row 0, column 0 is a real key -- V -- so an
 * in-band row/col sentinel could not express "unmapped" here.)
 *
 * flags:
 *   BRAIN_KEY_DIRECT - key closes the (row, col) contact
 *   BRAIN_KEY_SYMBOL - key additionally holds the "symbols" (記号)
 *                      modifier contact while down, reproducing the
 *                      blue-legend layer of the real keyboard
 */
#define BRAIN_KEY_DIRECT (1 << 0)
#define BRAIN_KEY_SYMBOL (1 << 1)

typedef struct BrainKeyDef {
    int8_t row;                 /* valid only when flags != 0 */
    int8_t col;
    uint8_t flags;
} BrainKeyDef;

typedef struct BrainKbdLayout {
    const char *name;
    int symbol_row, symbol_col; /* "symbols" modifier contact        */
    const BrainKeyDef *direct;  /* [KEY_CNT] host keys        */
    const BrainKeyDef *combo;   /* [KEY_CNT] Ctrl+key combos  */
} BrainKbdLayout;

const BrainKbdLayout *brain_kbd_find_layout(const char *name);

struct BrainKbdState {
    SysBusDevice parent_obj;

    DeviceState *pinctrl;       /* link to the i.MX28 PINCTRL */
    char *layout_name;          /* board layout property */
    const BrainKbdLayout *layout;

    uint32_t row_out;           /* GPIO4 effective output (DOUT&DOE) */
    uint32_t row_doe;           /* GPIO4 output-enable: rows the guest drives */
    uint64_t pressed;           /* bitmask: bit = row * BRAIN_KBD_COLS + col */
    unsigned int ctrl_held;     /* host Ctrl keys currently down */
};

/* boot ROM emulation (hw/sharp-brain/bootrom.c) */
void brain_bootrom_setup(Imx28State *s, BlockBackend *blk);

#endif /* HW_SHARP_BRAIN_BRAIN_H */
