/*
 * i.MX28 misc blocks: DIGCTL, CLKCTRL, POWER, OCOTP, EMI, USB PHY,
 * PWM and PXP, plus generic stmp register files.
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
 *   - PXP: register-set blits executed synchronously on the
 *     HW_PXP_NEXT write (see pxp_process)
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "brain.h"

typedef struct MxsMiscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;                /* unused output line (stmp blocks) */
    uint32_t regs[0x2000 / 16];
    uint8_t kind;               /* which block */
    uint8_t ctrl_idx;           /* register slot holding SFTRST/CLKGATE */
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
#define KIND_USBPHY   5
#define KIND_PWM      6
#define KIND_STMP     7
#define KIND_PXP      8

/* DIGCTL register indexes (offset / 16) */
#define DIGCTL_MICROSECONDS  0x0c
#define DIGCTL_CHIPID        0x31

/* POWER register indexes */
#define POWER_STS            0x0c
#define POWER_BATTMONITOR    0x0e
#define POWER_RESET          0x10

/*
 * HW_POWER_RESET (+0x100), from u-boot regs-power-mx28.h:
 *
 *   bits 31:16  UNLOCK field, key 0x3e77
 *   bit  2      FASTFALL_PSWITCH_OFF
 *   bit  1      PWD_OFF - power off and stay off
 *   bit  0      PWD     - power-down request
 *
 * The WinCE suspend trampoline (copied to IRAM by VMC_PowerDown)
 * writes UNLOCK + PWD as the final step of suspend-to-RAM.  On real
 * hardware with 5V present the chip powers down and then powers
 * itself back on automatically (NXP: "The MX28 does not power off if
 * you have 5V"); the ROM re-runs the boot chain and EBOOT's
 * CheckResumeInfo finds the kernel's saved resume state still in
 * (self-refreshed) DRAM, jumping straight to the resume vector.
 *
 * POWER_STS models 5V as present, so model PWD as that automatic
 * power cycle: request a guest reset, which re-runs the boot ROM /
 * EBOOT / resume chain.  Guest DRAM is not cleared on reset, exactly
 * like the self-refreshed DRAM on hardware.  PWD_OFF powers the
 * device off for real, which for a VM means shutting the emulator
 * down.
 */
#define POWER_RESET_UNLOCK_KEY       0x3e770000u
#define POWER_RESET_FASTFALL_PSWITCH_OFF  (1u << 2)
#define POWER_RESET_PWD_OFF          (1u << 1)
#define POWER_RESET_PWD              (1u << 0)

/*
 * HW_CLKCTRL_RESET (+0x1e0): global chip-reset register (Linux
 * mach-mxs uses it for reboot: MXS_CLKCTRL_RESET_CHIP = bit 1).
 */
#define CLKCTRL_RESET         0x1e
#define CLKCTRL_RESET_CHIP    (1u << 1)

/*
 * Battery voltage: BATTVAL field (bits 31:16) is in 8 mV steps.
 * Report a healthy Li-ion pack (~3.7 V) so the boot loader's
 * battery-fatal check passes.
 */
#define POWER_BATTMONITOR_BATTVAL_MV   3700

/* EMI register indexes */
#define EMI_DEBUG            0x0e
#define EMI_DEBUG_DLL_LOCK   (1u << 20)

/* PXP register slots (offset / 16) and bits (common_regspxp.h) */
#define PXP_STAT_SLOT        0x01        /* HW_PXP_STAT  @ 0x10 */
#define PXP_NEXT_SLOT        0x10        /* HW_PXP_NEXT  @ 0x100 */
#define PXP_STAT_IRQ         (1u << 0)
#define PXP_STAT_NEXT_IRQ    (1u << 3)
#define PXP_NEXT_ENABLED     (1u << 0)

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

/*
 * PXP data path. The guest's display driver programs the PXP by writing
 * a register set to RAM and pointing HW_PXP_NEXT at it. The register set
 * (observed live, boottest163, 14 identical blits; full dump in
 * boottest204/run.log) is a packed 32-bit-word struct:
 *   [+0x000] CTRL (0x84147), [+0x004] OUT_BUF (dest PA),
 *   [+0x010] PS_BUF (source PA), [+0x028] PS_SCALE = 0x10001000 (1.0).
 * The driver copies the 800x480 landscape GDI surface to a 480x800
 * portrait combine buffer, which the guest's copy loop then reads with
 * stride 1600 (hardcoded `add r4, #1600`) and writes to the fb with
 * stride 960. So the combine must be written with stride 1600 and
 * rotated 90 deg CW (combine[y][x] = surface(479-x, y)) — this is
 * what the uploaded PXP model does, and it makes the fb an exact
 * rotation of the surface (boottest205: fb = CW(surface), 384000/384000)
 * and the console show the setup screen clean and upright. (CCW
 * produces an upside-down console, boottest204; stride 960 makes the
 * copy loop downscale 800->480, boottest200.) The register set carries
 * no output-pitch field (set+0x08 is 0), so the stride is taken from
 * the copy loop / uploaded model, not the register set. RGB565. The
 * RECT is ignored (a full-surface copy is always correct; the RECT only
 * selects which tiles are dirty).
 */
static bool pxp_read_guest(hwaddr buf, uint8_t *out, uint32_t len)
{
    uint32_t off = 0;

    while (off < len) {
        MemoryRegionSection sec = memory_region_find(get_system_memory(),
                                                     buf + off,
                                                     len - off);
        if (!sec.mr || !memory_region_is_ram(sec.mr) || sec.size == 0) {
            return false;
        }
        if (sec.size > len - off) {
            sec.size = len - off;
        }
        uint8_t *host = memory_region_get_ram_ptr(sec.mr);
        memcpy(out + off, host + sec.offset_within_region, sec.size);
        off += sec.size;
    }
    return true;
}

/*
 * Host pointer for a guest-RAM range, or NULL if it is not a
 * single RAM region.
 */
static uint8_t *pxp_host_ptr(hwaddr pa, uint32_t len)
{
    MemoryRegionSection sec = memory_region_find(get_system_memory(), pa,
                                                 len);
    if (!sec.mr || !memory_region_is_ram(sec.mr) || sec.size < len) {
        return NULL;
    }
    return memory_region_get_ram_ptr(sec.mr) + sec.offset_within_region;
}

/*
 * Packed YUV 4:2:2 -> RGB565, the PXP's CSC for S0_FORMAT YUV422
 * register sets (HW_PXP_CTRL bits 15:12; i.MX23/28 PXP: 0x8 =
 * YUV422, 0xa = the UYVY byte order the Brain driver uses).
 * Byte order in memory is [U, Y0, V, Y1] for 0xa and [Y0, U, Y1, V]
 * for 0x8; both pixels of a pair share the pair's chroma. BT.601
 * full-range coefficients -- with these, converting the guest's
 * UYVY combine frame reproduces its RGB surface pixel-exactly for
 * 92% of pixels (rest: surface edits after the last transfer).
 */
static uint16_t pxp_yuv422_to_rgb565(const uint8_t *row, int x,
                                     bool uyvy)
{
    const uint8_t *pair = row + (x & ~1) * 2;
    int y, u, v, r, g, b;

    if (uyvy) {
        u = pair[0];
        y = pair[1 + (x & 1) * 2];
        v = pair[2];
    } else {
        y = pair[(x & 1) * 2];
        u = pair[1];
        v = pair[3];
    }
    r = y + (((v - 128) * 359) >> 8);                    /* 1.402 */
    g = y - (((u - 128) * 88) >> 8) - (((v - 128) * 183) >> 8);
    b = y + (((u - 128) * 454) >> 8);                    /* 1.772 */
    r = CLAMP(r, 0, 255);
    g = CLAMP(g, 0, 255);
    b = CLAMP(b, 0, 255);
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static void pxp_process(hwaddr next)
{
    uint8_t set[0x100];
    uint32_t ctrl, scale, src_pa, dst_pa;
    uint8_t *src, *dst;
    int x, y;

    if (!pxp_read_guest(next, set, sizeof(set))) {
        return;
    }
    memcpy(&ctrl, set + 0x000, 4);
    memcpy(&dst_pa, set + 0x004, 4);
    memcpy(&src_pa, set + 0x010, 4);
    memcpy(&scale, set + 0x028, 4);
    if (!dst_pa || !src_pa) {
        return;
    }
    if (ctrl != 0x00084147 || scale != 0x10001000) {
        /*
         * A different operation than the usual surface->combine
         * rotation: observed when ATR CALL starts, where the driver
         * programs a combine->framebuffer composite (PS_BUF =
         * combine, OUT_BUF = LCD scanout buffer, CTRL 0xca147,
         * PS_SCALE 0x0aaa0aaa). The fixed rotation above writes
         * 800 rows at stride 1600 (1.28 MB) wherever OUT_BUF points;
         * aimed at the 768 KB framebuffer that overruns into the
         * kernel image and the guest dies in an abort loop.
         *
         * The set carries its geometry in two packed rect words at
         * +0x01c/+0x024 whose four byte fields are pixel counts in
         * units of 8 (decoded from the stopped-guest snapshot in
         * p7-atr-scale-2026-11-15.md):
         *   +0x01c: out offset (x, y), PS size (w, h)
         *   +0x024: PS offset (x, y), out size (w, h)
         * For ATR: out at (80, 0), PS 480x320, out 720x480. PS_SCALE
         * holds the input/output ratio in 4.12 fixed point per axis
         * (0x0aaa = 2/3), i.e. the output is the input upscaled by
         * 3/2 -- which is exactly what the rect words say (480x320
         * -> 720x480) and matches the buffer contents: the combine
         * holds a packed 480x320 landscape frame (stride 960) with
         * the app's user-data UI, not a 720x480 one. Two overlay
         * groups follow the same rect packing:
         *   +0x040 buf, +0x044 rect, +0x048 OL0PARAM
         *   +0x050 buf, +0x054 rect, +0x058 OL1PARAM
         * (the set is the documented PXP_NEXT register set; the
         * overlay parameter word carries ALPHA/FORMAT/ENABLE bits,
         * cf. the Freescale PXP drivers' pxp_set_olparam()).  For
         * ATR, +0x050 is an 80x480 sidebar strip (volume /
         * example / clear touchkey labels, read verbatim from its
         * buffer) at (0, 0) and +0x040 an 80x144 alarm tile at
         * (0, 336).  Both PARAM words carry 0x40 (the FORMAT
         * field) and differ in bit 0 (ENABLE): the strip is always
         * enabled (0x41); the tile's enable is set only while an
         * alarm sounds (verified live in p15: a firing countdown
         * timer flips it to 0x41), so bit 0 of the PARAM word is
         * the composite gate.
         * Overlays are copied verbatim (no alpha model).
         *
         * Rects are in the device's landscape coordinates; the
         * framebuffer itself is the portrait panel image (480 px
         * rows at stride 960, main area = rows 0..799), and the
         * panel transform is fb[row = landscape x][479 - landscape
         * y] (verified pixel-exact against a live screendump), so
         * each landscape pixel is written through that mapping.
         * The sidebar rows 800..853 of the frame are not touched.
         */
        uint32_t w01c, w024;
        int out_x, out_y, in_w, in_h, in_x, in_y, out_w, out_h;

        memcpy(&w01c, set + 0x01c, 4);
        memcpy(&w024, set + 0x024, 4);
        out_x = ((w01c >> 24) & 0xff) * 8;
        out_y = ((w01c >> 16) & 0xff) * 8;
        in_w  = ((w01c >>  8) & 0xff) * 8;
        in_h  = ( w01c        & 0xff) * 8;
        in_x  = ((w024 >> 24) & 0xff) * 8;
        in_y  = ((w024 >> 16) & 0xff) * 8;
        out_w = ((w024 >>  8) & 0xff) * 8;
        out_h = ( w024        & 0xff) * 8;
        if (in_w > 0 && in_h > 0 && out_w > 0 && out_h > 0 &&
            in_x + in_w <= 800 && in_y + in_h <= 800) {
            src = pxp_host_ptr(src_pa, (in_y + in_h) * in_w * 2);
            dst = pxp_host_ptr(dst_pa, 800 * 480 * 2);
            if (src && dst) {
                uint32_t scx = scale & 0xffff, scy = scale >> 16;
                int s0_format = (ctrl >> 12) & 0xf;
                bool ps_yuv = (s0_format == 0x8 || s0_format == 0xa);
                int ox, oy, g;

                /*
                 * PS: nearest-neighbour scale into the out rect.
                 * A YUV422 S0 (CTRL 0xca147 for ATR CALL) is run
                 * through the CSC on the way: the guest stores the
                 * app frame in the combine as packed UYVY and the
                 * real PXP converts it back to RGB565 (the SCALE
                 * bit is only defined for YUV sources). Copying it
                 * verbatim, as this branch used to, displays the
                 * UYVY bytes as RGB565 -- rainbow banding in every
                 * textured area (p9-atr-color-2026-11-15.md).
                 * RGB565 S0 sets (0x4) keep the verbatim path.
                 */
                for (oy = 0; oy < out_h; oy++) {
                    int ly = out_y + oy;
                    int sy = in_y + (int)((oy * scy) >> 12);
                    const uint16_t *srow =
                        (const uint16_t *)(src + sy * in_w * 2);
                    if (ly < 0 || ly >= 480 || sy >= in_y + in_h) {
                        continue;
                    }
                    for (ox = 0; ox < out_w; ox++) {
                        int lx = out_x + ox;
                        int sx = in_x + (int)((ox * scx) >> 12);
                        uint16_t *dpx;

                        if (lx < 0 || lx >= 800 || sx >= in_x + in_w) {
                            continue;
                        }
                        dpx = (uint16_t *)(dst + lx * 960 +
                                           (479 - ly) * 2);
                        if (ps_yuv) {
                            *dpx = pxp_yuv422_to_rgb565(
                                (const uint8_t *)srow, sx,
                                s0_format == 0xa);
                        } else {
                            *dpx = srow[sx];
                        }
                    }
                }
                /*
                 * Overlay groups at +0x040 (OL0) / +0x050 (OL1).
                 * OL0 is composited last (on top): the guest leaves
                 * the strip's (OL1) rows under the alarm tile's rect
                 * blank and sets OL0's enable only while an alarm is
                 * sounding (observed live: the countdown timer firing
                 * flips OL0PARAM 0x40 -> 0x41), so OL0 must win the
                 * overlap for the tile to be visible at all.
                 */
                for (g = 1; g >= 0; g--) {
                    uint32_t obuf, orect, ogate;
                    const uint8_t *osrc;
                    int gx, gy, gw, gh, row;

                    memcpy(&obuf, set + 0x040 + g * 0x10, 4);
                    memcpy(&orect, set + 0x044 + g * 0x10, 4);
                    memcpy(&ogate, set + 0x048 + g * 0x10, 4);
                    if (!obuf || !(ogate & 1)) {
                        continue;
                    }
                    gx = ((orect >> 24) & 0xff) * 8;
                    gy = ((orect >> 16) & 0xff) * 8;
                    gw = ((orect >>  8) & 0xff) * 8;
                    gh = ( orect        & 0xff) * 8;
                    if (gw <= 0 || gh <= 0) {
                        continue;
                    }
                    osrc = pxp_host_ptr(obuf, gw * gh * 2);
                    if (!osrc) {
                        continue;
                    }
                    for (row = 0; row < gh; row++) {
                        int ly = gy + row;
                        const uint16_t *srow =
                            (const uint16_t *)(osrc + row * gw * 2);
                        int col;

                        if (ly < 0 || ly >= 480) {
                            continue;
                        }
                        for (col = 0; col < gw; col++) {
                            int lx = gx + col;
                            uint16_t *dpx;

                            if (lx < 0 || lx >= 800) {
                                continue;
                            }
                            dpx = (uint16_t *)(dst + lx * 960 +
                                               (479 - ly) * 2);
                            *dpx = srow[col];
                        }
                    }
                }
                BRAIN_LOG("pxp: blit src=0x%08x dst=0x%08x "
                         "(scaled composite %dx%d -> %dx%d @(%d,%d), "
                         "ctrl=0x%08x scale=0x%08x)\n",
                         src_pa, dst_pa, in_w, in_h, out_w, out_h,
                         out_x, out_y, ctrl, scale);
                return;
            }
        }
        /*
         * Fallback for sets whose geometry does not decode: the
         * bounded repack from a750cd0e, which at least cannot
         * overrun the framebuffer.
         */
        src = pxp_host_ptr(src_pa, 800 * 800 * 2);
        dst = pxp_host_ptr(dst_pa, 800 * 480 * 2);
        if (!src || !dst) {
            return;
        }
        for (y = 0; y < 800; y++) {
            memcpy(dst + y * 960, src + y * 1600, 960);
        }
        BRAIN_LOG("pxp: blit src=0x%08x dst=0x%08x "
                 "(combine -> fb repack, ctrl=0x%08x scale=0x%08x)\n",
                 src_pa, dst_pa, ctrl, scale);
        return;
    }
    src = pxp_host_ptr(src_pa, 800 * 480 * 2);
    /*
     * The dest stride is 1600, not the nominal 960: the guest's
     * copy loop reads the combine with stride 1600, so the dest
     * region is 800 rows x 1600 B and the rotation is CW (see the
     * pxp_process header comment above for the full story).
     */
    dst = pxp_host_ptr(dst_pa, 800 * 800 * 2);
    if (!src || !dst) {
        return;
    }
    /* 90 deg CW: dest(row y, col x) = src(row 479-x, col y). */
    for (y = 0; y < 800; y++) {
        uint16_t *drow = (uint16_t *)(dst + y * 1600);
        for (x = 0; x < 480; x++) {
            uint16_t *srow = (uint16_t *)(src + (479 - x) * 1600);
            drow[x] = srow[y];
        }
    }
    BRAIN_LOG("pxp: blit src=0x%08x dst=0x%08x "
             "(800x480 -> 480x800 CW, stride 1600)\n",
             src_pa, dst_pa);
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
     * guest clears CLKGATE itself after releasing SFTRST.  The USB
     * PHY block carries its CTRL register at offset 0x30.
     */
    if (idx == s->ctrl_idx) {
        if (!(old & (1u << 31)) && (s->regs[s->ctrl_idx] & (1u << 31))) {
            s->regs[s->ctrl_idx] |= 1u << 30;
        }
    }

    /*
     * POWER_RESET / CLKCTRL_RESET: chip reset and power-down requests
     * (see the POWER_RESET comment above).
     */
    if (s->kind == KIND_POWER && idx == POWER_RESET) {
        uint32_t v = s->regs[POWER_RESET];

        if ((v & 0xffff0000u) == POWER_RESET_UNLOCK_KEY) {
            if (v & POWER_RESET_PWD) {
                /* 5V present: power down + automatic power-on = reset */
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            } else if (v & POWER_RESET_PWD_OFF) {
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            }
        }
    }
    if (s->kind == KIND_CLKCTRL && idx == CLKCTRL_RESET) {
        if (s->regs[CLKCTRL_RESET] & CLKCTRL_RESET_CHIP) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
    }

    /*
     * PXP: writing a register-set pointer to NEXT starts a process.
     * Copy the pixel data first, then report completion (STAT.IRQ |
     * NEXT_IRQ set, IRQ line raised until the guest's ISR clears the
     * status bits): the guest's driver waits on the completion IRQ,
     * so the copy must be done before it is raised. The driver writes
     * the raw physical address; ENABLED is a hardware "buffer pending"
     * indication, not part of the write.
     */
    if (s->kind == KIND_PXP) {
        if (idx == PXP_NEXT_SLOT && off == 0 && value != 0) {
            pxp_process((hwaddr)value);
            s->regs[PXP_STAT_SLOT] |= PXP_STAT_IRQ | PXP_STAT_NEXT_IRQ;
            qemu_set_irq(s->irq, true);
        }
        if (idx == PXP_STAT_SLOT) {
            qemu_set_irq(s->irq, !!(s->regs[PXP_STAT_SLOT] &
                                    (PXP_STAT_IRQ | PXP_STAT_NEXT_IRQ)));
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
    if (s->kind == KIND_PWM) {
        /* PWM0..7_PRESENT bits so PwmInitialize's channel scan passes */
        s->regs[0] = 0x3fc00000;
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
    } else if (strcmp(t, TYPE_MXS_USBPHY) == 0) {
        /* USBPHY: PWD@0x0, TX@0x10, RX@0x20, CTRL@0x30 */
        s->kind = KIND_USBPHY;
        s->ctrl_idx = 3;
    } else if (strcmp(t, TYPE_MXS_PWM) == 0) {
        s->kind = KIND_PWM;
    } else if (strcmp(t, TYPE_MXS_STMP) == 0) {
        s->kind = KIND_STMP;
    } else if (strcmp(t, TYPE_MXS_PXP) == 0) {
        s->kind = KIND_PXP;
    } else {
        s->kind = KIND_EMI;
    }

    memory_region_init_io(&s->iomem, obj, &mxs_misc_ops, s,
                          "mxs-misc", 0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
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
        TYPE_MXS_EMI, TYPE_MXS_USBPHY, TYPE_MXS_PWM, TYPE_MXS_STMP,
        TYPE_MXS_PXP,
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
