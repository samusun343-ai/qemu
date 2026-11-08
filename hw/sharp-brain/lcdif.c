/*
 * i.MX28 LCDIF emulation (basic display path).
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Bring-up display model: 16bpp RGB565 framebuffer refreshed from the
 * CUR_BUF address programmed by the guest (the same path the u-boot
 * mxsfb driver uses).  The Sharp smart-LCD protocol used by the
 * Windows CE "brainlcd" driver is refined once the guest behaviour is
 * captured.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/display/framebuffer.h"
#include "brain.h"

#define LCDIF_CTRL           0x000
#define LCDIF_CTRL1          0x010
#define LCDIF_CTRL2          0x020
#define LCDIF_TRANSFER_COUNT 0x030
#define LCDIF_CUR_BUF        0x040
#define LCDIF_NEXT_BUF       0x050
#define LCDIF_TIMING         0x060
#define LCDIF_VDCTRL(n)      (0x070 + (n) * 0x10)
#define LCDIF_DVICTRL(n)     (0x0c0 + (n) * 0x10)
#define LCDIF_DATA           0x180
#define LCDIF_VERSION        0x190

#define CTRL_SFTRST          (1u << 31)
#define CTRL_CLKGATE         (1u << 30)
#define CTRL_RUN             (1u << 28)
#define CTRL_DATA_FORMAT_16  (1u << 3)
#define CTRL_LCDIF_EN        (1u << 0)

#define REG_IDX(off)         ((off) / 0x10)

static void lcdif_invalidate(void *opaque)
{
    MxsLcdifPriv *s = opaque;

    s->invalidate = true;
}

static void lcdif_draw_line16(void *opaque, uint8_t *dst, const uint8_t *src,
                              int width, int deststep)
{
    uint16_t rgb565;
    uint8_t r, g, b;

    while (width--) {
        rgb565 = lduw_le_p(src);
        r = ((rgb565 >> 11) & 0x1f) << 3;
        g = ((rgb565 >>  5) & 0x3f) << 2;
        b = ((rgb565 >>  0) & 0x1f) << 3;
        src += 2;
        *(uint32_t *)dst = rgb_to_pixel32(r, g, b);
        dst += 4;
    }
}

static bool lcdif_gfx_update(void *opaque)
{
    MxsLcdifPriv *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int first, last;
    uint32_t ctrl = s->regs[REG_IDX(LCDIF_CTRL)];
    uint32_t tc = s->regs[REG_IDX(LCDIF_TRANSFER_COUNT)];
    uint32_t bufaddr = s->regs[REG_IDX(LCDIF_CUR_BUF)];
    uint32_t width = tc >> 16, height = tc & 0xffff;

    if (surface == NULL) {
        return true;
    }
    if (!(ctrl & CTRL_RUN) || !(ctrl & CTRL_LCDIF_EN)) {
        return true;
    }
    if (width == 0 || height == 0) {
        return true;
    }
    if (width != s->width || height != s->height) {
        s->width = width;
        s->height = height;
        qemu_console_resize(s->con, width, height);
        s->invalidate = true;
    }

    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection,
                                          get_system_memory(),
                                          bufaddr, height, width * 2);
        s->invalidate = false;
    }

    framebuffer_update_display(surface, &s->fbsection,
                               width, height,
                               width * 2, width * 4, 4,
                               0, lcdif_draw_line16, NULL,
                               &first, &last);

    if (first >= 0) {
        qemu_console_update(s->con, 0, first, width, last - first + 1);
    }
    return true;
}

static const GraphicHwOps lcdif_gfx_ops = {
    .invalidate = lcdif_invalidate,
    .gfx_update = lcdif_gfx_update,
};

/* DMA channel 13 client hook */
int mxs_lcdif_dma_data(MxsLcdifPriv *s, bool to_device,
                       dma_addr_t bufaddr, uint32_t len)
{
    /*
     * memory -> panel data push (smart LCD mode).  Remember the pushed
     * buffer as the current framebuffer so the periodic refresh can
     * display it; refine once the Sharp panel protocol is understood.
     */
    if (to_device) {
        s->regs[REG_IDX(LCDIF_CUR_BUF)] = bufaddr;
        if (s->height) {
            s->regs[REG_IDX(LCDIF_TRANSFER_COUNT)] =
                ((len / 2 / s->height) << 16) | s->height;
        }
        s->invalidate = true;
    }
    return 0;
}

static uint64_t lcdif_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsLcdifPriv *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;

    if (off != 0 || reg >= 0x2000) {
        return 0;
    }
    if (reg == LCDIF_VERSION) {
        return (2 << 24);       /* v2 (i.MX28) */
    }
    return s->regs[REG_IDX(reg)];
}

static void lcdif_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    MxsLcdifPriv *s = opaque;
    uint32_t reg = addr & ~0xf;
    uint32_t off = addr & 0xf;
    uint32_t idx;

    if (reg >= 0x2000) {
        return;
    }
    idx = REG_IDX(reg);

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

    if (reg == LCDIF_CTRL || reg == LCDIF_CUR_BUF ||
        reg == LCDIF_NEXT_BUF) {
        s->invalidate = true;
    }
}

static const MemoryRegionOps lcdif_reg_ops = {
    .read = lcdif_read,
    .write = lcdif_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void lcdif_reset(DeviceState *dev)
{
    MxsLcdifPriv *s = MXS_LCDIF(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->invalidate = true;
    s->width = 0;
    s->height = 0;
}

static void lcdif_init(Object *obj)
{
    MxsLcdifPriv *s = MXS_LCDIF(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lcdif_reg_ops, s,
                          "mxs-lcdif", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void lcdif_realize(DeviceState *dev, Error **errp)
{
    MxsLcdifPriv *s = MXS_LCDIF(dev);

    s->con = qemu_graphic_console_create(dev, 0, &lcdif_gfx_ops, s);
}

static void lcdif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lcdif_realize;
    device_class_set_legacy_reset(dc, lcdif_reset);
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
