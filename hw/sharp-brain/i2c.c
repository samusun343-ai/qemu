/*
 * i.MX28 I2C controller emulation (PIO master mode + APBX DMA hooks).
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * HW_I2C, bases 0x80058000 (I2C0) / 0x8005a000 (I2C1).
 *
 * Master transfers triggered by CTRL0.RUN execute synchronously:
 * transmit bytes are taken from the DATA0-3 registers, received
 * bytes land there.  A transfer to a slave that does not answer
 * (e.g. the RTC / battery sub-MCU of the real machine, which is
 * not modelled) reports NO_SLAVE_ACK, which the boot loader
 * handles gracefully.
 *
 * When the APBX DMA drives the controller, a PIO descriptor programs
 * CTRL0/CTRL1 (RUN arms but does not start the transfer); the
 * following data descriptor moves the payload and executes it.
 *
 * Registers use the stmp 16-byte layout (REG/SET/CLR/TOG).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "system/dma.h"
#include "brain.h"

#define HW_I2C_CTRL0       0x000
#define HW_I2C_TIMING0     0x010
#define HW_I2C_CTRL1       0x020
#define HW_I2C_STAT        0x030
#define HW_I2C_TIMING1     0x040
#define HW_I2C_TIMING2     0x050
#define HW_I2C_QUEUECTRL   0x060
#define HW_I2C_QUEUEEND    0x070
#define HW_I2C_QUEUECMD    0x080
#define HW_I2C_QUEUEDATA   0x090
#define HW_I2C_DATA(n)     (0x0a0 + (n) * 0x10)

#define BM_CTRL0_SFTRST      (1u << 31)
#define BM_CTRL0_CLKGATE     (1u << 30)
#define BM_CTRL0_RUN         (1u << 29)
#define BM_CTRL0_PREACK      (1u << 27)
#define BM_CTRL0_ACKNOWLEDGE (1u << 26)
#define BM_CTRL0_NAK_ON_LAST (1u << 25)
#define BM_CTRL0_MASTER_MODE (1u << 17)
#define BM_CTRL0_DIRECTION   (1u << 16)   /* 1 = read */
#define BM_CTRL0_XFER_COUNT  0x0000ffffu

#define BM_CTRL1_IRQ_EN      0x0000ff00u
#define BM_CTRL1_IRQ         0x000000feu

#define BM_STAT_MASTER_PRESENT     (1u << 31)
#define BM_STAT_SLAVE_PRESENT      (1u << 30)
#define BM_STAT_GOT_A_NAK          (1u << 28)
#define BM_STAT_BUS_BUSY           (1u << 11)
#define BM_STAT_DATA_ENGINE_BUSY   (1u << 9)
#define BM_STAT_DATA_ENGINE_CMPLT_IRQ_SUMMARY (1u << 6)
#define BM_STAT_NO_SLAVE_ACK_IRQ_SUMMARY      (1u << 5)

static void i2c_update_irq(MxsI2cState *s)
{
    uint32_t en = (s->ctrl1 & BM_CTRL1_IRQ_EN) >> 8;
    uint32_t st = s->ctrl1 & BM_CTRL1_IRQ;

    qemu_set_irq(s->irq, !!(en & st));
}

/* DATA0-3 hold the transfer bytes MSB-first, four per register */
static void i2c_data_pack(MxsI2cState *s, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        s->data[i / 4] |= buf[i] << (24 - 8 * (i % 4));
    }
}

static void i2c_data_unpack(MxsI2cState *s, uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = (s->data[i / 4] >> (24 - 8 * (i % 4))) & 0xff;
    }
}

static void i2c_run(MxsI2cState *s)
{
    uint32_t count = s->ctrl0 & BM_CTRL0_XFER_COUNT;
    bool read = s->ctrl0 & BM_CTRL0_DIRECTION;
    uint8_t buf[16];
    int ret = 0;

    s->run_pending = false;

    if (!(s->ctrl0 & BM_CTRL0_MASTER_MODE) || count == 0) {
        goto done;
    }

    if (read) {
        if (count > sizeof(buf)) {
            count = sizeof(buf);
        }
        /*
         * Master-mode address: the first DATA byte is the 8-bit
         * address byte on the wire (Linux i2c-mxs puts
         * i2c_8bit_addr_from_msg() first into the data stream;
         * CTRL1.SLAVE_ADDRESS_BYTE is the address this block
         * responds to in *slave* mode, not the master address).
         * QEMU's i2c core matches slaves by 7-bit address (all
         * in-tree masters pass addr >> 1), so shift it down here.
         * The old code took the address from CTRL1 and passed the
         * 8-bit byte straight through, so no slave could ever
         * match (masked only because no slave is attached yet).
         */
        ret = i2c_start_recv(s->bus, (s->data[0] >> 25) & 0x7f);
        if (ret < 0) {
            s->stat |= BM_STAT_NO_SLAVE_ACK_IRQ_SUMMARY;
            goto done;
        }
        for (uint32_t i = 0; i < count; i++) {
            buf[i] = i2c_recv(s->bus);
        }
        i2c_end_transfer(s->bus);
        /* scatter the received bytes over DATA0..3 (MSB first) */
        i2c_data_pack(s, buf, count);
    } else {
        /* gather transmit bytes from DATA0..3 */
        if (count > sizeof(buf)) {
            count = sizeof(buf);
        }
        i2c_data_unpack(s, buf, count);
        if (count == 0) {
            goto done;
        }
        /* buf[0] is the 8-bit address byte; QEMU matches 7-bit */
        ret = i2c_start_send(s->bus, buf[0] >> 1);
        if (ret < 0) {
            s->stat |= BM_STAT_NO_SLAVE_ACK_IRQ_SUMMARY;
            goto done;
        }
        for (uint32_t i = 1; i < count; i++) {
            if (i2c_send(s->bus, buf[i]) < 0) {
                s->stat |= BM_STAT_GOT_A_NAK;
                break;
            }
        }
        i2c_end_transfer(s->bus);
    }

done:
    s->stat |= BM_STAT_DATA_ENGINE_CMPLT_IRQ_SUMMARY;
    s->ctrl0 &= ~BM_CTRL0_RUN;
    i2c_update_irq(s);
}

static uint64_t i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsI2cState *s = opaque;
    uint32_t reg = addr & ~0xf;

    switch (reg) {
    case HW_I2C_CTRL0:
        return s->ctrl0;
    case HW_I2C_TIMING0:
        return s->timing0;
    case HW_I2C_CTRL1:
        return s->ctrl1;
    case HW_I2C_STAT:
        return s->stat | BM_STAT_MASTER_PRESENT;
    case HW_I2C_TIMING1:
        return s->timing1;
    case HW_I2C_TIMING2:
        return s->timing2;
    case HW_I2C_QUEUECTRL:
        return s->queuectrl;
    case HW_I2C_QUEUEEND:
        return s->queueend;
    case HW_I2C_QUEUECMD:
        return s->queuecmd;
    case HW_I2C_QUEUEDATA:
        return s->queuedata;
    default:
        if (reg >= HW_I2C_DATA(0) && reg < HW_I2C_DATA(4)) {
            return s->data[(reg - HW_I2C_DATA(0)) / 0x10];
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "mxs-i2c: unimplemented read 0x%" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

/* REG/SET/CLR/TOG alias write, shared by all registers below */
static void i2c_alias_write(uint32_t *reg, uint32_t off, uint64_t val)
{
    switch (off) {
    case 0:  *reg = val; break;
    case 4:  *reg |= val; break;
    case 8:  *reg &= ~val; break;
    case 12: *reg ^= val; break;
    }
}

/*
 * One stmp register write.  off selects the REG/SET/CLR/TOG alias.
 * from_dma: RUN only arms the transfer (the APBX data descriptor
 * executes it); otherwise RUN executes immediately.
 */
static void i2c_write_one(MxsI2cState *s, uint32_t reg, uint32_t off,
                          uint64_t val, bool from_dma)
{
    switch (reg) {
    case HW_I2C_CTRL0:
    {
        uint32_t old = s->ctrl0;

        i2c_alias_write(&s->ctrl0, off, val);
        /* SFTRST rising edge -> clock gates once */
        if (!(old & BM_CTRL0_SFTRST) && (s->ctrl0 & BM_CTRL0_SFTRST)) {
            s->ctrl0 |= BM_CTRL0_CLKGATE;
        }
        if ((s->ctrl0 & BM_CTRL0_RUN) && !(old & BM_CTRL0_RUN)) {
            if (from_dma) {
                s->run_pending = true;
            } else {
                i2c_run(s);
            }
        }
        return;
    }
    case HW_I2C_TIMING0:
        i2c_alias_write(&s->timing0, off, val);
        return;
    case HW_I2C_CTRL1:
        i2c_alias_write(&s->ctrl1, off, val);
        i2c_update_irq(s);
        return;
    case HW_I2C_STAT:
        /* write-1-clear through every alias */
        s->stat &= ~val;
        return;
    case HW_I2C_TIMING1:
        i2c_alias_write(&s->timing1, off, val);
        return;
    case HW_I2C_TIMING2:
        i2c_alias_write(&s->timing2, off, val);
        return;
    case HW_I2C_QUEUECTRL:
        i2c_alias_write(&s->queuectrl, off, val);
        return;
    case HW_I2C_QUEUEEND:
        i2c_alias_write(&s->queueend, off, val);
        return;
    case HW_I2C_QUEUECMD:
        i2c_alias_write(&s->queuecmd, off, val);
        return;
    case HW_I2C_QUEUEDATA:
        i2c_alias_write(&s->queuedata, off, val);
        return;
    default:
        if (reg >= HW_I2C_DATA(0) && reg < HW_I2C_DATA(4)) {
            i2c_alias_write(&s->data[(reg - HW_I2C_DATA(0)) / 0x10],
                            off, val);
            return;
        }
        break;
    }

    qemu_log_mask(LOG_UNIMP, "mxs-i2c: unimplemented write 0x%08x"
                  " = 0x%" PRIx64 "\n", reg, val);
}

static void i2c_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MxsI2cState *s = opaque;

    i2c_write_one(s, addr & ~0xfu, addr & 0xfu, val, false);
}

/* ------------------------------------------------------------------ */
/* APBX DMA client hooks (channels 6 = I2C0, 7 = I2C1)                 */
/* ------------------------------------------------------------------ */

void mxs_i2c_dma_pio(MxsI2cState *s, int word_idx, uint32_t value)
{
    if (word_idx < 0) {
        /* end of the PIO word sequence; nothing to kick here */
        return;
    }
    /* DMA PIO words land in consecutive 16-byte-spaced registers */
    i2c_write_one(s, word_idx * 0x10, 0, value, true);
}

void mxs_i2c_dma_data(MxsI2cState *s, bool to_device,
                      dma_addr_t bufaddr, uint32_t len)
{
    uint8_t buf[16];

    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }

    if (to_device) {
        /* READ descriptor: memory -> controller DATA0-3, then the
         * armed transfer transmits them */
        memset(buf, 0, sizeof(buf));
        address_space_read(&address_space_memory, bufaddr,
                           MEMTXATTRS_UNSPECIFIED, buf, len);
        i2c_data_pack(s, buf, len);
        if (s->run_pending) {
            i2c_run(s);
        }
    } else {
        /* WRITE descriptor: controller -> memory.  Execute a pending
         * receive first so DATA0-3 hold the payload. */
        if (s->run_pending) {
            i2c_run(s);
        }
        memset(buf, 0, sizeof(buf));
        i2c_data_unpack(s, buf, sizeof(buf));
        address_space_write(&address_space_memory, bufaddr,
                            MEMTXATTRS_UNSPECIFIED, buf, len);
    }
}

/* ------------------------------------------------------------------ */

static const MemoryRegionOps i2c_ops = {
    .read = i2c_read,
    .write = i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void i2c_reset(DeviceState *dev)
{
    MxsI2cState *s = MXS_I2C(dev);

    s->ctrl0 = 0;
    s->ctrl1 = 0;
    s->stat = 0;
    s->timing0 = s->timing1 = s->timing2 = 0;
    s->queuectrl = s->queueend = s->queuecmd = s->queuedata = 0;
    memset(s->data, 0, sizeof(s->data));
    s->run_pending = false;
    qemu_set_irq(s->irq, 0);
}

static void i2c_init(Object *obj)
{
    MxsI2cState *s = MXS_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &i2c_ops, s,
                          "mxs-i2c", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(s), NULL);
}

static void i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "i.MX28 I2C controller";
    device_class_set_legacy_reset(dc, i2c_reset);
}

static const TypeInfo i2c_info = {
    .name = TYPE_MXS_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsI2cState),
    .instance_init = i2c_init,
    .class_init = i2c_class_init,
};

static void i2c_register(void)
{
    type_register_static(&i2c_info);
}
type_init(i2c_register)
