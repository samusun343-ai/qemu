/*
 * mxs LCDIF emulation with ILI9805 smart-panel model (MPU 8080 mode).
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The SHARP Brain drives its LCD through the LCDIF in system (MPU)
 * mode: single-word accesses send commands/parameters to the panel
 * (DATA_SELECT=0/1), and full frames are pushed by DMA from NEXT_BUF
 * after a "memory write" (0x2C) command.  The panel is natively
 * 480x854 portrait; the Brain mounts it rotated and uses the MAC
 * register (MX/MY/MV/GS/BGR) for the transform, so the emulated GRAM
 * stores native-orientation pixels and the console shows the rotated
 * (landscape) view.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "brain.h"

#define HW_LCDIF_CTRL             0x000
#define HW_LCDIF_CTRL1            0x010
#define HW_LCDIF_CTRL2            0x020
#define HW_LCDIF_TRANSFER_COUNT   0x030
#define HW_LCDIF_CUR_BUF          0x040
#define HW_LCDIF_NEXT_BUF         0x050
#define HW_LCDIF_TIMING           0x060
#define HW_LCDIF_VDCTRL(n)        (0x070 + (n) * 0x10)
#define HW_LCDIF_DVICTRL(n)       (0x0c0 + (n) * 0x10)
#define HW_LCDIF_CSC_COEFF(n)     (0x110 + (n) * 0x10)
#define HW_LCDIF_DATA             0x180
#define HW_LCDIF_VERSION          0x190

#define BM_CTRL_SFTRST        (1u << 31)
#define BM_CTRL_CLKGATE       (1u << 30)
#define BM_CTRL_RUN           (1u << 0)
#define BM_CTRL_DATA_SELECT   (1u << 16)
#define BM_CTRL_MASTER        (1u << 5)

#define BM_CTRL1_RESET        (1u << 30)
#define BP_CTRL1_PACKING      16
#define BM_CTRL1_PACKING      (0xfu << 16)
#define BM_CTRL1_CUR_FRAME_DONE_IRQ     (1u << 9)
#define BM_CTRL1_CUR_FRAME_DONE_IRQ_EN  (1u << 13)
#define BM_CTRL1_VSYNC_EDGE_IRQ         (1u << 8)
#define BM_CTRL1_VSYNC_EDGE_IRQ_EN      (1u << 12)

/* ILI9805 commands used by the Brain firmware */
#define ILI9805_MAC           0x36
#define ILI9805_PIXFMT        0x3A
#define ILI9805_CASET         0x2A
#define ILI9805_PASET         0x2B
#define ILI9805_RAMWR         0x2C
#define ILI9805_SLPOUT        0x11
#define ILI9805_DISPON        0x29

#define MAC_GS   (1u << 0)
#define MAC_MX   (1u << 6)
#define MAC_MY   (1u << 7)
#define MAC_MV   (1u << 5)
#define MAC_BGR  (1u << 3)

/* ------------------------------------------------------------------ */
/* ILI9805 panel                                                      */
/* ------------------------------------------------------------------ */

static void panel_invalidate(MxsLcdifPriv *s)
{
    if (s->con) {
        qemu_console_update(s->con, 0, 0, s->width, s->height);
        s->dirty = false;
    }
}

/*
 * Advance the GRAM write position.
 *
 * The address counter always walks the CASET (column) coordinate first
 * and wraps into the PASET (page) coordinate - for the Brain firmware
 * this holds in both MAC modes (verified against the WinCE display
 * driver's push pattern, see the geometry comment above
 * lcdif_gfx_update).  MV only changes how the panel scans the GRAM out
 * to the glass, which the console side models separately.
 */
static void panel_advance(MxsLcdifPriv *s)
{
    if (s->gram_x < s->col_end) {
        s->gram_x++;
    } else {
        s->gram_x = s->col_start;
        if (s->gram_y < s->page_end) {
            s->gram_y++;
        } else {
            s->gram_y = s->page_start;
        }
    }
}

/* write one RGB565 pixel into the GRAM at the current position */
static void panel_pixel(MxsLcdifPriv *s, uint16_t rgb565)
{

    uint32_t x = s->gram_x, y = s->gram_y;

    /* apply MY/MX mirror around the panel edges */
    if (s->mac & MAC_MY) {
        y = ILI9805_NATIVE_HEIGHT - 1 - y;
    }
    if (s->mac & MAC_MX) {
        x = ILI9805_NATIVE_WIDTH - 1 - x;
    }

    if (x < ILI9805_NATIVE_WIDTH && y < ILI9805_NATIVE_HEIGHT) {
        s->gram[y * ILI9805_NATIVE_WIDTH + x] = rgb565;
    }
    s->dirty = true;
    panel_advance(s);
}

/* handle one command/data halfword arriving over the 16-bit bus */
static void panel_word(MxsLcdifPriv *s, uint32_t word, bool is_data)
{
    uint16_t val16 = word & 0xffff;

    if (s->write_active && is_data) {
        /* inside RAMWR stream: pixel data */
        if (s->mac & MAC_BGR) {
            uint16_t r = val16 >> 11, g = (val16 >> 5) & 0x3f, b = val16 & 0x1f;
            val16 = (b << 11) | (g << 5) | r;
        }
        panel_pixel(s, val16);
        return;
    }

    if (!is_data) {
        /* command */
        s->last_cmd = val16 & 0xff;
        s->cmd_pos = 0;
        s->write_active = false;
        qemu_log_mask(LOG_TRACE,
                      "brain-panel: CMD 0x%02x (mac=0x%02x win x[%u..%u] "
                      "y[%u..%u] pos %u,%u)\n", s->last_cmd, s->mac,
                      s->col_start, s->col_end, s->page_start, s->page_end,
                      s->gram_x, s->gram_y);
        switch (s->last_cmd) {
        case ILI9805_SLPOUT:
            s->panel_on = true;
            break;
        case ILI9805_DISPON:
            s->panel_on = true;
            panel_invalidate(s);
            break;
        case ILI9805_RAMWR:
            s->write_active = true;
            /* reset to window start */
            s->gram_x = s->col_start;
            s->gram_y = s->page_start;
            break;
        default:
            break;
        }
        return;
    }

    /* parameter data for the current command */
    switch (s->last_cmd) {
    case ILI9805_MAC:
        s->mac = val16 & 0xff;
        qemu_log_mask(LOG_TRACE, "brain-panel: MAC = 0x%02x\n", s->mac);
        break;
    case ILI9805_PIXFMT:
        s->pixel_fmt = val16 & 0xff;
        qemu_log_mask(LOG_TRACE, "brain-panel: PIXFMT = 0x%02x\n", s->pixel_fmt);
        break;
    case ILI9805_CASET:
        if (s->cmd_pos < 4) {
            s->cmd_bytes[s->cmd_pos++] = val16 & 0xff;
        }
        if (s->cmd_pos == 4) {
            s->col_start = (s->cmd_bytes[0] << 8) | s->cmd_bytes[1];
            s->col_end   = (s->cmd_bytes[2] << 8) | s->cmd_bytes[3];
            s->gram_x = s->col_start;
            qemu_log_mask(LOG_TRACE, "brain-panel: CASET %u..%u\n",
                          s->col_start, s->col_end);
        }
        break;
    case ILI9805_PASET:
        if (s->cmd_pos < 4) {
            s->cmd_bytes[s->cmd_pos++] = val16 & 0xff;
        }
        if (s->cmd_pos == 4) {
            s->page_start = (s->cmd_bytes[0] << 8) | s->cmd_bytes[1];
            s->page_end   = (s->cmd_bytes[2] << 8) | s->cmd_bytes[3];
            s->gram_y = s->page_start;
            qemu_log_mask(LOG_TRACE, "brain-panel: PASET %u..%u\n",
                          s->page_start, s->page_end);
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* LCDIF transfer engine                                              */
/* ------------------------------------------------------------------ */

static void lcdif_update_irq(MxsLcdifPriv *s)
{
    bool done = (s->ctrl1 & BM_CTRL1_CUR_FRAME_DONE_IRQ) &&
                (s->ctrl1 & BM_CTRL1_CUR_FRAME_DONE_IRQ_EN);
    bool vsync = (s->ctrl1 & BM_CTRL1_VSYNC_EDGE_IRQ) &&
                 (s->ctrl1 & BM_CTRL1_VSYNC_EDGE_IRQ_EN);
    qemu_set_irq(s->irq, done || vsync);
}

/* Periodic VSYNC edge: the panel refreshes at ~60 Hz and the LCDIF
 * raises a VSYNC edge each frame.  The guest display driver clears
 * the VSYNC status bit (CTRL1 CLR alias) and waits for the next edge
 * before pushing a frame, so a VSYNC that only fires on transfer
 * completion deadlocks the driver after the initial pushes.  Re-arm
 * every 16.667 ms (QEMU_CLOCK_VIRTUAL: follows guest virtual time). */
#define LCDIF_VSYNC_PERIOD_NS (16666667ULL)
#define LCDIF_REPUSH_VSYNCS 30   /* re-check the framebuffer every 0.5 s */
static void lcdif_repush_if_dirty(MxsLcdifPriv *s);
static void lcdif_vsync_tick(void *opaque)
{
    MxsLcdifPriv *s = opaque;

    s->ctrl1 |= BM_CTRL1_VSYNC_EDGE_IRQ;
    lcdif_update_irq(s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  LCDIF_VSYNC_PERIOD_NS);
    if (s->full_frame_buf) {
        lcdif_repush_if_dirty(s);
    }
}

/* FNV-1a checksum over a host buffer (fast framebuffer change check) */
static uint32_t lcdif_crc_host(const uint8_t *p, size_t len)
{
    uint32_t h = 0x811c9dc5u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 0x01000193u;
    }
    return h;
}

/* checksum of a guest-memory region, read in 4 KiB chunks */
static uint32_t lcdif_crc_guest(hwaddr buf, uint32_t len)
{
    uint32_t h = 0x811c9dc5u;
    uint8_t tmp[4096];
    uint32_t off = 0;

    while (off < len) {
        uint32_t n = len - off < sizeof(tmp) ? len - off : sizeof(tmp);
        if (address_space_read(&address_space_memory, buf + off,
                               MEMTXATTRS_UNSPECIFIED, tmp, n) != MEMTX_OK) {
            break;
        }
        for (uint32_t i = 0; i < n; i++) {
            h ^= tmp[i];
            h *= 0x01000193u;
        }
        off += n;
    }
    return h;
}

/*
 * VSYNC dirty re-push: the shell draws desktop content into the
 * framebuffer directly and the driver only pushes on present, so a
 * frame can be drawn after the driver's last push and never reach
 * the panel (the desktop icons sit in the framebuffer while the
 * console still shows the earlier, black frame).  Every 30 VSYNCs,
 * if the LCDIF is idle and the full framebuffer has changed since
 * it was last pushed, re-stream the full frame into the GRAM
 * (internal only: does not touch the guest-visible RUN/IRQ state,
 * which the driver left parked).  This keeps the console tracking
 * the framebuffer the way a continuously-scanning display would.
 * The full frame is used (not the last pushed region) because the
 * last push is often the small menu-bar strip while the content
 * lands in the main area.
 */
static void lcdif_repush_if_dirty(MxsLcdifPriv *s)
{
    uint32_t total = ILI9805_NATIVE_WIDTH * ILI9805_NATIVE_HEIGHT;
    uint32_t crc;
    uint8_t *mem;

    if (!s->panel_on || (s->ctrl & BM_CTRL_RUN)) {
        return;
    }
    if (++s->vsync_repush_counter < LCDIF_REPUSH_VSYNCS) {
        return;
    }
    s->vsync_repush_counter = 0;
    crc = lcdif_crc_guest(s->full_frame_buf, total * 2);
    if (crc == s->full_frame_crc) {
        return;
    }
    mem = g_malloc(total * 2 + 4);
    if (address_space_read(&address_space_memory, s->full_frame_buf,
                           MEMTXATTRS_UNSPECIFIED, mem,
                           total * 2) != MEMTX_OK) {
        g_free(mem);
        return;
    }
    /* re-stream the full frame through the panel as a RAMWR stream */
    s->col_start = 0;
    s->col_end = ILI9805_NATIVE_WIDTH - 1;
    s->page_start = 0;
    s->page_end = ILI9805_NATIVE_HEIGHT - 1;
    s->gram_x = s->col_start;
    s->gram_y = s->page_start;
    s->write_active = true;
    for (uint32_t i = 0; i < total; i++) {
        panel_word(s, lduw_le_p(&mem[i * 2]), true);
    }
    g_free(mem);
    s->full_frame_crc = crc;
    if (s->dirty) {
        panel_invalidate(s);
    }
    qemu_log_mask(LOG_TRACE,
                  "lcdif: VSYNC re-push full frame buf=0x%" HWADDR_PRIx
                  " (framebuffer changed)\n",
                  s->full_frame_buf);
}

/* a CPU/DMA transfer sequence completed */
static void lcdif_transfer_done(MxsLcdifPriv *s)
{
    s->ctrl &= ~BM_CTRL_RUN;
    s->ctrl1 |= BM_CTRL1_CUR_FRAME_DONE_IRQ | BM_CTRL1_VSYNC_EDGE_IRQ;
    if (s->dirty) {
        panel_invalidate(s);
    }
    lcdif_update_irq(s);
}

static uint32_t lcdif_count(MxsLcdifPriv *s)
{
    uint32_t h = (s->transfer_count >> 16) & 0xffff;
    uint32_t v = s->transfer_count & 0xffff;

    return h * v;
}

/*
 * Master-mode frame push: stream words from the buffer through the
 * 8080 bus into the panel (RAMWR stream).  Each 32-bit memory word
 * carries two 16-bit bus transfers.
 */
static void lcdif_run_master(MxsLcdifPriv *s)
{
    uint32_t total = lcdif_count(s);
    bool is_data = s->ctrl & BM_CTRL_DATA_SELECT;
    hwaddr buf = s->cur_buf;
    uint32_t done = 0;
    uint8_t *mem;

    qemu_log_mask(LOG_TRACE,
                  "brain-lcdif: master run: count=%ux%u total=%u "
                  "cur_buf=0x%08x next_buf=0x%08x data=%d ctrl=0x%08x\n",
                  (s->transfer_count >> 16) & 0xffff,
                  s->transfer_count & 0xffff, total, (uint32_t)buf,
                  s->next_buf, is_data, s->ctrl);

    if (total == 0) {
        lcdif_transfer_done(s);
        return;
    }

    mem = g_malloc(total * 2 + 4);
    if (address_space_read(&address_space_memory, buf,
                           MEMTXATTRS_UNSPECIFIED, mem,
                           total * 2) != MEMTX_OK) {
        g_free(mem);
        lcdif_transfer_done(s);
        return;
    }

    while (done < total) {
        uint16_t bus_word = lduw_le_p(&mem[done * 2]);
        panel_word(s, bus_word, is_data);
        done++;
    }

    /* remember the full framebuffer for the VSYNC dirty re-push
     * (only a full-frame push establishes the framebuffer base) */
    if (total == (uint32_t)(ILI9805_NATIVE_WIDTH *
                            ILI9805_NATIVE_HEIGHT)) {
        s->full_frame_buf = buf;
        s->full_frame_crc = lcdif_crc_host(mem, total * 2);
        s->vsync_repush_counter = 0;
    }
    g_free(mem);

    /* swap buffers like the hardware */
    s->cur_buf = s->next_buf;
    lcdif_transfer_done(s);
}

/* RUN set (register write) starts a transfer */
static void lcdif_run(MxsLcdifPriv *s)
{
    if (s->ctrl & BM_CTRL_MASTER) {
        lcdif_run_master(s);
    } else {
        /*
         * System (CPU) mode: the CPU feeds the panel through the DATA
         * register; count down transfers and finish when the count is
         * reached.  A zero count completes immediately.
         */
        s->cpu_pending = lcdif_count(s);
        if (s->cpu_pending == 0) {
            lcdif_transfer_done(s);
        }
    }
}

/* one word written to the DATA register in system mode */
static void lcdif_cpu_data(MxsLcdifPriv *s, uint32_t val)
{
    bool is_data = s->ctrl & BM_CTRL_DATA_SELECT;

    panel_word(s, val & 0xffff, is_data);

    if (s->ctrl & BM_CTRL_RUN) {
        if (s->cpu_pending > 1) {
            s->cpu_pending--;
        } else {
            s->cpu_pending = 0;
            lcdif_transfer_done(s);
        }
    }
}

/* ------------------------------------------------------------------ */
/* registers                                                          */
/* ------------------------------------------------------------------ */

static uint64_t lcdif_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsLcdifPriv *s = opaque;
    uint32_t reg = addr & ~0xf;

    switch (reg) {
    case HW_LCDIF_CTRL:
        return s->ctrl;
    case HW_LCDIF_CTRL1:
        return s->ctrl1;
    case HW_LCDIF_CTRL2:
        return s->ctrl2;
    case HW_LCDIF_TRANSFER_COUNT:
        return s->transfer_count;
    case HW_LCDIF_CUR_BUF:
        return s->cur_buf;
    case HW_LCDIF_NEXT_BUF:
        return s->next_buf;
    case HW_LCDIF_TIMING:
        return s->timing;
    case HW_LCDIF_VERSION:
        return 0x010000;
    default:
        if (reg >= HW_LCDIF_VDCTRL(0) && reg < HW_LCDIF_VDCTRL(4)) {
            return s->vdctrl[(reg - HW_LCDIF_VDCTRL(0)) / 0x10];
        }
        if (reg >= HW_LCDIF_DVICTRL(0) && reg < HW_LCDIF_DVICTRL(4)) {
            return s->dvi[(reg - HW_LCDIF_DVICTRL(0)) / 0x10];
        }
        if (reg >= HW_LCDIF_CSC_COEFF(0) && reg < HW_LCDIF_CSC_COEFF(0) + 0x40) {
            return s->csc[(reg - HW_LCDIF_CSC_COEFF(0)) / 0x10];
        }
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "mxs-lcdif: unimplemented read 0x%" HWADDR_PRIx "\n",
                  addr);
    return 0;
}

static void lcdif_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    MxsLcdifPriv *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

#define STMP_REG(field)                                         \
    do {                                                        \
        switch (off) {                                          \
        case 0:  s->field = val; break;                         \
        case 4:  s->field |= val; break;                         \
        case 8:  s->field &= ~val; break;                        \
        case 12: s->field ^= val; break;                         \
        }                                                        \
    } while (0)

    switch (reg) {
    case HW_LCDIF_CTRL:
    {
        uint32_t old = s->ctrl;

        STMP_REG(ctrl);
        /*
         * SFTRST rising edge -> the block enters reset and its clock
         * gates once (boot loaders poll for CLKGATE reading back set
         * before releasing the block).  One-shot, so an explicit
         * CLKGATE clear while SFTRST is still asserted sticks.
         */
        if (!(old & BM_CTRL_SFTRST) && (s->ctrl & BM_CTRL_SFTRST)) {
            s->ctrl |= BM_CTRL_CLKGATE;
        }
        if ((s->ctrl & BM_CTRL_RUN) && !(old & BM_CTRL_RUN)) {
            lcdif_run(s);
        }
        return;
    }
    case HW_LCDIF_CTRL1:
    {
        STMP_REG(ctrl1);
        /* IRQ status bits are cleared through the CLR alias */
        if (off == 8) {
            s->ctrl1 &= ~(val & (BM_CTRL1_CUR_FRAME_DONE_IRQ |
                                 BM_CTRL1_VSYNC_EDGE_IRQ));
        }
        lcdif_update_irq(s);
        return;
    }
    case HW_LCDIF_CTRL2:
        STMP_REG(ctrl2);
        return;
    case HW_LCDIF_TRANSFER_COUNT:
        STMP_REG(transfer_count);
        return;
    case HW_LCDIF_CUR_BUF:
        STMP_REG(cur_buf);
        return;
    case HW_LCDIF_NEXT_BUF:
        STMP_REG(next_buf);
        return;
    case HW_LCDIF_TIMING:
        STMP_REG(timing);
        return;
    case HW_LCDIF_DATA:
        lcdif_cpu_data(s, val);
        return;
    default:
        if (reg >= HW_LCDIF_VDCTRL(0) && reg < HW_LCDIF_VDCTRL(4)) {
            s->vdctrl[(reg - HW_LCDIF_VDCTRL(0)) / 0x10] = val;
            return;
        }
        if (reg >= HW_LCDIF_DVICTRL(0) && reg < HW_LCDIF_DVICTRL(4)) {
            s->dvi[(reg - HW_LCDIF_DVICTRL(0)) / 0x10] = val;
            return;
        }
        if (reg >= HW_LCDIF_CSC_COEFF(0) && reg < HW_LCDIF_CSC_COEFF(0) + 0x40) {
            s->csc[(reg - HW_LCDIF_CSC_COEFF(0)) / 0x10] = val;
            return;
        }
        break;
    }
#undef STMP_REG

    qemu_log_mask(LOG_UNIMP,
                  "mxs-lcdif: unimplemented write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, val);
}

static const MemoryRegionOps lcdif_ops = {
    .read = lcdif_read,
    .write = lcdif_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* graphics console                                                   */
/* ------------------------------------------------------------------ */

/*
 * The console shows the device (landscape) view.
 *
 * Geometry (reverse engineered from the WinCE display driver's actual
 * register traffic, see README):
 *
 *   - the framebuffer is a portrait 480(x=CASET) x 854(y=PASET) image
 *     streamed row-major: the address counter walks CASET first, so one
 *     framebuffer row is one display column of the landscape UI;
 *   - the panel glass is 480x854 and is mounted rotated 90 degrees in
 *     the Brain case, so the panel's Y axis (854) is the device's
 *     horizontal axis;
 *   - with the firmware's MAC = MX|MY|MV (0xd0) the GRAM is written
 *     mirrored on both native axes (panel_pixel);
 *   - the mounting rotation maps device (x, y) to GRAM (853-x, y),
 *     i.e. console(x, y) = GRAM[853-x][y]: the portrait GRAM rotates
 *     counter-clockwise into the landscape console.  This direction
 *     is VERIFIED, not derived: the EBOOT splash reads upright with
 *     it and the menu bar sits on the right edge (x = 800..853).
 *     The opposite choice (GRAM[853-x][479-y]) shows the screen
 *     upside down.  The transform is constant - it only depends on
 *     how the panel is glued into the case.
 */
static bool lcdif_gfx_update(void *opaque)
{
    MxsLcdifPriv *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int stride = surface_stride(surface);
    uint8_t *dest = surface_data(surface);
    int w = surface_width(surface);
    int h = surface_height(surface);

    if (surface_bits_per_pixel(surface) != 32) {
        return true;
    }

    for (int y = 0; y < h; y++) {
        uint32_t *row = (uint32_t *)(dest + y * stride);
        for (int x = 0; x < w; x++) {
            int nx = y;
            int ny = ILI9805_NATIVE_HEIGHT - 1 - x;
            uint16_t rgb = (nx >= 0 && ny >= 0) ? s->gram[ny * ILI9805_NATIVE_WIDTH + nx] : 0;
            int r = ((rgb >> 11) & 0x1f) << 3;
            int g = ((rgb >> 5) & 0x3f) << 2;
            int b = (rgb & 0x1f) << 3;
            row[x] = rgb_to_pixel32(r, g, b);
        }
    }
    qemu_console_update(s->con, 0, 0, w, h);
    return true;
}

static void lcdif_invalidate(void *opaque)
{
    /* full redraw on the next gfx_update */
}

static const GraphicHwOps lcdif_gfx_ops = {
    .invalidate = lcdif_invalidate,
    .gfx_update = lcdif_gfx_update,
};

/* ------------------------------------------------------------------ */
/* APBH DMA channel 13 client hook                                    */
/* ------------------------------------------------------------------ */

int mxs_lcdif_dma_data(MxsLcdifPriv *s, bool to_device,
                       dma_addr_t bufaddr, uint32_t len)
{
    bool is_data = s->ctrl & BM_CTRL_DATA_SELECT;
    uint8_t *mem;

    qemu_log_mask(LOG_TRACE,
                  "brain-lcdif: dma %s len=%u at 0x%08x data=%d ctrl=0x%08x "
                  "xfer=0x%08x\n", to_device ? "push" : "pull", len,
                  (uint32_t)bufaddr, is_data, s->ctrl, s->transfer_count);

    if (!to_device) {
        return 0;
    }

    /* memory -> panel data push (smart LCD mode) */
    mem = g_malloc(len);
    if (address_space_read(&address_space_memory, bufaddr,
                           MEMTXATTRS_UNSPECIFIED, mem, len) != MEMTX_OK) {
        g_free(mem);
        return -1;
    }
    for (uint32_t i = 0; i + 1 < len; i += 2) {
        panel_word(s, lduw_le_p(&mem[i]), is_data);
    }
    if (s->dirty) {
        panel_invalidate(s);
    }
    g_free(mem);
    return 0;
}

/* ------------------------------------------------------------------ */

static void lcdif_reset(DeviceState *dev)
{
    MxsLcdifPriv *s = MXS_LCDIF(dev);


    s->ctrl = 0;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->transfer_count = 0;
    s->cur_buf = s->next_buf = 0;
    s->timing = 0;
    memset(s->vdctrl, 0, sizeof(s->vdctrl));
    memset(s->dvi, 0, sizeof(s->dvi));
    memset(s->csc, 0, sizeof(s->csc));

    s->cpu_pending = 0;
    s->panel_on = false;
    s->last_cmd = 0;
    s->cmd_pos = 0;
    s->mac = 0;
    s->col_start = s->col_end = 0;
    s->page_start = s->page_end = 0;
    s->gram_x = s->gram_y = 0;
    s->pixel_fmt = 0x55;   /* 16bpp */
    s->write_active = false;
    s->full_frame_buf = 0;
    s->full_frame_crc = 0;
    s->vsync_repush_counter = 0;
    memset(s->gram, 0, sizeof(uint16_t) * ILI9805_NATIVE_WIDTH *
           ILI9805_NATIVE_HEIGHT);
    s->dirty = false;

    qemu_set_irq(s->irq, false);
}

static void lcdif_init(Object *obj)
{
    MxsLcdifPriv *s = MXS_LCDIF(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->gram = g_malloc0(sizeof(uint16_t) * ILI9805_NATIVE_WIDTH *
                        ILI9805_NATIVE_HEIGHT);

    memory_region_init_io(&s->iomem, OBJECT(s), &lcdif_ops, s,
                          "mxs-lcdif", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void lcdif_realize(DeviceState *dev, Error **errp)
{
    MxsLcdifPriv *s = MXS_LCDIF(dev);

    if (s->width == 0) {
        s->width = 854;
    }
    if (s->height == 0) {
        s->height = 480;
    }
    s->con = qemu_graphic_console_create(dev, 0, &lcdif_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);

    /* start the periodic VSYNC (panel refresh) */
    s->vsync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcdif_vsync_tick, s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  LCDIF_VSYNC_PERIOD_NS);
}

static const Property lcdif_properties[] = {
    DEFINE_PROP_UINT32("width", MxsLcdifPriv, width, 854),
    DEFINE_PROP_UINT32("height", MxsLcdifPriv, height, 480),
};

static void lcdif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "mxs LCDIF with ILI9805 smart panel";
    dc->realize = lcdif_realize;
    device_class_set_legacy_reset(dc, lcdif_reset);
    device_class_set_props(dc, lcdif_properties);
}

static const TypeInfo lcdif_info = {
    .name = TYPE_MXS_LCDIF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsLcdifPriv),
    .instance_init = lcdif_init,
    .class_init = lcdif_class_init,
};

static void lcdif_register(void)
{
    type_register_static(&lcdif_info);
}
type_init(lcdif_register)
