/*
 * NXP i.MX28 SoC (used by the SHARP Brain machines).
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Emulates the i.MX283: ARM926EJ-S core, 128KB IRAM at 0x0, and the
 * APBH/APBX peripheral set at 0x80000000 that the Sharp boot chain
 * (boot ROM -> SB stream -> stage0 -> EBOOT -> Windows CE) touches.
 *
 * Blocks that are not modelled yet (GPMI/NAND, DCP, AUART, I2C, USB,
 * CAN, SAIF, SPDIF, RTC, PWM, ...) are instantiated as unimplemented
 * devices so accesses log but do not fault.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/char/pl011.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "target/arm/cpu-qom.h"
#include "brain.h"

static void imx28_icoll_connect(Imx28State *s, DeviceState *dev, int irq)
{
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in_named(DEVICE(s->icoll), "irqs",
                                              irq));
}

/* sink for DMA channels that have no interrupt on the real chip */
static void imx28_unused_irq(void *opaque, int n, int level)
{
}

/*
 * Catch-all "unimplemented" blocks: register layout, sizes and gaps
 * follow NXP's i.MX28 memory map (u-boot regs-base.h).
 */
static const struct {
    const char *name;
    hwaddr base;
    uint64_t size;
} imx28_unimplemented[] = {
    { "imx28-hsadc",    0x80002000, 0x2000 },
    { "imx28-perfmon",  0x80006000, 0x2000 },
    { "imx28-bch",      0x8000a000, 0x2000 },
    { "imx28-gpmi",     0x8000c000, 0x4000 },
    { "imx28-etm",      0x80022000, 0x2000 },
    { "imx28-axi-ahb0", 0x8002e000, 0x2000 },
    { "imx28-can0",     0x80032000, 0x2000 },
    { "imx28-can1",     0x80034000, 0x2000 },
    { "imx28-simdbg",   0x8003c004, 0x3ffc }, /* bootrom mailbox at +0 */
    { "imx28-usb0",     0x80080000, 0x10000 },
    { "imx28-usb1",     0x80090000, 0x10000 },
    { "imx28-dflpt",    0x800c0000, 0x2000 },
    { "imx28-enet0",    0x800f0000, 0x4000 },
    { "imx28-enet1",    0x800f4000, 0x4000 },
};

/*
 * Additional stmp peripheral blocks modelled as plain register files
 * with the SFTRST/CLKGATE reset handshake (misc.c, TYPE_MXS_STMP).
 * Drivers probe these during cspddk/device init; without the handshake
 * their "set SFTRST, poll CLKGATE" loops never terminate.
 */
static const struct {
    const char *type;
    const char *name;
    hwaddr base;
    int irq;                    /* ICOLL line, -1 = none */
} imx28_stmp_blocks[IMX28_NUM_STMP_BLOCKS] = {
    { TYPE_MXS_STMP, "auart0",   0x8006a000, 112 },
    { TYPE_MXS_STMP, "auart1",   0x8006c000, 113 },
    { TYPE_MXS_STMP, "auart2",   0x8006e000, 114 },
    { TYPE_MXS_STMP, "auart3",   0x80070000, 115 },
    { TYPE_MXS_STMP, "auart4",   0x80072000, 116 },
    { TYPE_MXS_STMP, "saif0",    0x80042000, 59 },
    { TYPE_MXS_STMP, "saif1",    0x80046000, 58 },
    { TYPE_MXS_STMP, "spdif",    0x80054000, 45 },
    { TYPE_MXS_STMP, "audioout", 0x80048000, -1 },
    { TYPE_MXS_STMP, "audioin",  0x8004c000, -1 },
    { TYPE_MXS_STMP, "dcp",      0x80028000, 53 },
    { TYPE_MXS_PXP,  "pxp",      0x8002a000, 39 },
};

static void imx28_realize(DeviceState *dev, Error **errp)
{
    Imx28State *s = IMX28(dev);

    /* CPU */
    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }

    /* IRAM */
    memory_region_init_ram(&s->iram, OBJECT(dev), "imx28.iram",
                           IMX28_IRAM_SIZE, errp);
    memory_region_add_subregion(get_system_memory(), IMX28_IRAM_BASE,
                                &s->iram);

    /* ICOLL */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->icoll), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->icoll), 0, IMX28_ICOLL_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->icoll), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));

    /* APBH DMA */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->apbh_dma), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->apbh_dma), 0, IMX28_APBH_DMA_BASE);
    for (int i = 0; i < MXS_DMA_CHANNELS; i++) {
        static const int dma_irq_map[MXS_DMA_CHANNELS] = {
            82, 83, 84, 85,     /* SSP0-3 */
            88, 88, 88, 88,     /* GPMI0-3 */
            88, 88, 88, 88,     /* GPMI4-7 */
            87,                 /* HSADC */
            86,                 /* LCDIF */
            0, 0,
        };
        sysbus_connect_irq(SYS_BUS_DEVICE(s->apbh_dma), i,
                           qdev_get_gpio_in_named(DEVICE(s->icoll), "irqs",
                                                  dma_irq_map[i]));
    }

    /*
     * APBX DMA (0x80024000): same CCW engine, APBX channel map
     * (linux imx28.dtsi): 0/1 auart4 rx/tx, 2 spdif, 3 none,
     * 4/5 saif0/1, 6/7 i2c0/1, 8..15 auart0..3 rx/tx.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->apbx_dma), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->apbx_dma), 0, IMX28_APBX_DMA_BASE);
    {
        static const int apbx_irq_map[MXS_DMA_CHANNELS] = {
            78, 79, 66, -1,
            80, 81, 68, 69,
            70, 71, 72, 73,
            74, 75, 76, 77,
        };
        qemu_irq unused = qemu_allocate_irq(imx28_unused_irq, NULL, 0);

        for (int i = 0; i < MXS_DMA_CHANNELS; i++) {
            qemu_irq line = (apbx_irq_map[i] < 0)
                ? unused
                : qdev_get_gpio_in_named(DEVICE(s->icoll), "irqs",
                                         apbx_irq_map[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(s->apbx_dma), i, line);
        }
    }

    /* SSP0 (eMMC), SSP1 (microSD), SSP2 (SPI sub-LCD), SSP3 */
    for (int i = 0; i < 4; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(s->ssp[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(s->ssp[i]), 0,
                        IMX28_SSP0_BASE + i * 0x2000);
        imx28_icoll_connect(s, s->ssp[i], IMX28_IRQ_SSP0 + i);
    }

    /* DIGCTL / CLKCTRL / POWER / OCOTP / PINCTRL / TIMROT / EMI / RTC */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->rtc), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->digctl), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->clkctrl), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->power), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->ocotp), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->emi), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->pinctrl), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->timrot), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->lcdif), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->digctl), 0, IMX28_DIGCTL_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->clkctrl), 0, IMX28_CLKCTRL_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->power), 0, IMX28_POWER_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->ocotp), 0, IMX28_OCOTP_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->emi), 0, IMX28_EMI_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->rtc), 0, 0x80056000);
    imx28_icoll_connect(s, s->rtc, 29);   /* alarm / onemsec IRQ */
    sysbus_mmio_map(SYS_BUS_DEVICE(s->pinctrl), 0, IMX28_PINCTRL_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->timrot), 0, IMX28_TIMROT_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lcdif), 0, IMX28_LCDIF_BASE);

    /* PWM (backlight) and the USB PHYs: register files with the
     * SFTRST/CLKGATE handshake; the USB driver resets the PHYs
     * during cspddk init and polls CTRL.C LKGATE (at 0x30) */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->pwm), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->usbphy[0]), errp) ||
        !sysbus_realize(SYS_BUS_DEVICE(s->usbphy[1]), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->pwm), 0, 0x80064000);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->usbphy[0]), 0, 0x8007c000);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->usbphy[1]), 0, 0x8007e000);

    /* remaining stmp blocks (AUART/SAIF/SPDIF/AUDIO/DCP/PXP) */
    for (int i = 0; i < IMX28_NUM_STMP_BLOCKS; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(s->stmp[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(s->stmp[i]), 0,
                        imx28_stmp_blocks[i].base);
        if (imx28_stmp_blocks[i].irq >= 0) {
            sysbus_connect_irq(SYS_BUS_DEVICE(s->stmp[i]), 0,
                               qdev_get_gpio_in_named(DEVICE(s->icoll),
                                                      "irqs",
                                                      imx28_stmp_blocks[i].irq));
        }
    }

    /* I2C0 / I2C1 */
    for (int i = 0; i < 2; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(s->i2c[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(s->i2c[i]), 0, 0x80058000 + i * 0x2000);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c[i]), 0,
                           qdev_get_gpio_in_named(DEVICE(s->icoll), "irqs",
                                                  111 - i));
    }

    /* LRADC (ADC + 4-wire touch) */
    if (!sysbus_realize(SYS_BUS_DEVICE(s->lradc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lradc), 0, IMX28_LRADC_BASE);
    {
        /* touch, thresh0/1, channel0..7, button0/1 */
        static const int lradc_irqs[MXS_LRADC_NUM_IRQ] = {
            IMX28_IRQ_LRADC_TOUCH,
            IMX28_IRQ_LRADC_THRESH0, IMX28_IRQ_LRADC_THRESH1,
            IMX28_IRQ_LRADC_CH(0), IMX28_IRQ_LRADC_CH(1),
            IMX28_IRQ_LRADC_CH(2), IMX28_IRQ_LRADC_CH(3),
            IMX28_IRQ_LRADC_CH(4), IMX28_IRQ_LRADC_CH(5),
            IMX28_IRQ_LRADC_CH(6), IMX28_IRQ_LRADC_CH(7),
            24, 25,
        };
        for (int i = 0; i < MXS_LRADC_NUM_IRQ; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(s->lradc), i,
                               qdev_get_gpio_in_named(DEVICE(s->icoll),
                                                      "irqs",
                                                      lradc_irqs[i]));
        }
    }

    /* TIMROT irqs 48-51 */
    for (int i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->timrot), i,
                           qdev_get_gpio_in_named(DEVICE(s->icoll), "irqs",
                                                  IMX28_IRQ_TIMROT0 + i));
    }
    /* GPIO bank irqs: GPIO0..4 = ICOLL 127..123 */
    {
        static const int gpio_irqs[5] = {
            IMX28_IRQ_GPIO0, IMX28_IRQ_GPIO1, IMX28_IRQ_GPIO2,
            IMX28_IRQ_GPIO3, IMX28_IRQ_GPIO4,
        };
        for (int i = 0; i < 5; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(s->pinctrl), i,
                               qdev_get_gpio_in_named(DEVICE(s->icoll),
                                                      "irqs", gpio_irqs[i]));
        }
    }
    /* LCDIF irq */
    imx28_icoll_connect(s, s->lcdif, IMX28_IRQ_LCDIF);

    /* DUART = PL011 */
    qdev_prop_set_chr(DEVICE(s->duart), "chardev", serial_hd(0));
    if (!sysbus_realize(SYS_BUS_DEVICE(s->duart), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(s->duart), 0, IMX28_DUART_BASE);
    imx28_icoll_connect(s, s->duart, IMX28_IRQ_DUART);

    /* unimplemented catch-all blocks */
    for (unsigned i = 0; i < ARRAY_SIZE(imx28_unimplemented); i++) {
        create_unimplemented_device(imx28_unimplemented[i].name,
                                    imx28_unimplemented[i].base,
                                    imx28_unimplemented[i].size);
    }

    /* DMA <-> SSP/LCDIF/I2C wiring */
    {
        MxsDmaState *dma = MXS_DMA(s->apbh_dma);

        dma->ssp[0] = MXS_SSP(s->ssp[0]);
        dma->ssp[1] = MXS_SSP(s->ssp[1]);
        dma->ssp[2] = MXS_SSP(s->ssp[2]);
        dma->ssp[3] = MXS_SSP(s->ssp[3]);
        dma->lcdif = MXS_LCDIF(s->lcdif);
    }
    {
        MxsDmaState *dma = MXS_DMA(s->apbx_dma);

        dma->i2c[0] = MXS_I2C(s->i2c[0]);
        dma->i2c[1] = MXS_I2C(s->i2c[1]);
    }
}

static void imx28_init(Object *obj)
{
    Imx28State *s = IMX28(obj);

    object_initialize_child(obj, "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("arm926"));

    s->icoll = qdev_new(TYPE_MXS_ICOLL);
    object_property_add_child(obj, "icoll", OBJECT(s->icoll));

    s->apbh_dma = qdev_new(TYPE_MXS_DMA);
    object_property_add_child(obj, "apbh-dma", OBJECT(s->apbh_dma));

    s->apbx_dma = qdev_new(TYPE_MXS_DMA);
    qdev_prop_set_bit(s->apbx_dma, "apbx", true);
    object_property_add_child(obj, "apbx-dma", OBJECT(s->apbx_dma));

    for (int i = 0; i < 4; i++) {
        s->ssp[i] = qdev_new(TYPE_MXS_SSP);
        object_property_add_child(obj, g_strdup_printf("ssp%d", i),
                                  OBJECT(s->ssp[i]));
    }

    s->digctl = qdev_new(TYPE_MXS_DIGCTL);
    object_property_add_child(obj, "digctl", OBJECT(s->digctl));
    s->clkctrl = qdev_new(TYPE_MXS_CLKCTRL);
    object_property_add_child(obj, "clkctrl", OBJECT(s->clkctrl));
    s->timrot = qdev_new(TYPE_MXS_TIMROT);
    object_property_add_child(obj, "timrot", OBJECT(s->timrot));
    s->pinctrl = qdev_new(TYPE_MXS_PINCTRL);
    object_property_add_child(obj, "pinctrl", OBJECT(s->pinctrl));
    s->power = qdev_new(TYPE_MXS_POWER);
    object_property_add_child(obj, "power", OBJECT(s->power));
    s->lradc = qdev_new(TYPE_MXS_LRADC);
    object_property_add_child(obj, "lradc", OBJECT(s->lradc));
    s->ocotp = qdev_new(TYPE_MXS_OCOTP);
    object_property_add_child(obj, "ocotp", OBJECT(s->ocotp));
    s->emi = qdev_new(TYPE_MXS_EMI);
    object_property_add_child(obj, "emi", OBJECT(s->emi));
    s->rtc = qdev_new(TYPE_MXS_RTC);
    object_property_add_child(obj, "rtc", OBJECT(s->rtc));

    s->pwm = qdev_new(TYPE_MXS_PWM);
    object_property_add_child(obj, "pwm", OBJECT(s->pwm));
    for (int i = 0; i < 2; i++) {
        s->usbphy[i] = qdev_new(TYPE_MXS_USBPHY);
        object_property_add_child(obj, g_strdup_printf("usbphy%d", i),
                                  OBJECT(s->usbphy[i]));
    }
    for (int i = 0; i < IMX28_NUM_STMP_BLOCKS; i++) {
        s->stmp[i] = qdev_new(imx28_stmp_blocks[i].type);
        object_property_add_child(obj,
                                  g_strdup_printf("%s", imx28_stmp_blocks[i].name),
                                  OBJECT(s->stmp[i]));
    }

    s->lcdif = qdev_new(TYPE_MXS_LCDIF);
    object_property_add_child(obj, "lcdif", OBJECT(s->lcdif));

    for (int i = 0; i < 2; i++) {
        s->i2c[i] = qdev_new(TYPE_MXS_I2C);
        object_property_add_child(obj, g_strdup_printf("i2c%d", i),
                                  OBJECT(s->i2c[i]));
    }

    s->duart = qdev_new(TYPE_PL011);
    object_property_add_child(obj, "duart", OBJECT(s->duart));
}

static void imx28_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = imx28_realize;
    dc->user_creatable = false;
}

static const TypeInfo imx28_info = {
    .name = TYPE_IMX28,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Imx28State),
    .instance_init = imx28_init,
    .class_init = imx28_class_init,
};

static void imx28_register(void)
{
    type_register_static(&imx28_info);
}
type_init(imx28_register)
