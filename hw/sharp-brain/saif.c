/*
 * i.MX28 SAIF (Synchronous Audio Interface) emulation.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Each SAIF is an I2S port fed by one APBX DMA channel (SAIF0 = TX
 * on channel 4, SAIF1 = RX on channel 5 on this SoC).  The codec
 * side (BU26154 register programming: sample rate, format, power)
 * is on I2C (bu26154.c); the PCM byte stream arrives here.
 *
 * Guest behaviour (observed live from the WinCE WaveDev driver,
 * wavedev2_BU26154.dll + the i.MX BSP SAIF driver):
 *
 *   - At machine init the CPU programs both SAIFs directly:
 *     SFTRST/CLKGATE handshake, SAIF0 CTRL = 0x801, SAIF1 CTRL =
 *     0x806, then SET of CTRL bit 0.  Real-hardware dumps from the
 *     NXP forum show playback CTRL = 0x08000801, STAT = 0x80000050,
 *     VERSION = 0x01010000; SAIF0 is the I2S master (it supplies
 *     MCLK/BCLK/LRCLK to the codec), SAIF1 the slave for record.
 *   - Playback data moves only through the APBX DMA: descriptor
 *     chains of 2048-byte buffers, stereo s16 LE at 44.1 kHz (the
 *     rate the driver programs into the codec's SR register).
 *     The guest never polls STAT/DATA (zero such accesses in a
 *     full boot + playback trace), so no completion IRQ timing is
 *     observable; the DMA engine's own completion IRQ drives the
 *     driver's buffer recycling.
 *
 * Model:
 *
 *   - STMP register layout (REG/SET/CLR/TOG aliases, SFTRST rising
 *     edge sets CLKGATE, as in misc.c): CTRL 0x00, STAT 0x10 (BUSY
 *     reads 1 while running), DATA 0x20 (CPU writes feed the TX
 *     FIFO), VERSION 0x30 = 0x01010000.
 *   - TX bytes (DMA or CPU) queue in a FIFO and drain to the QEMU
 *     audio backend at 44.1 kHz stereo s16 via a pacing timer and
 *     the backend's avail callback.  The DMA chain is circular
 *     (two double-buffered descriptors), so the DMA engine only
 *     primes it; this device's timer resumes it one buffer per
 *     buffer-duration (mxs_dma_pace), which spreads the driver's
 *     completion IRQs and refills over time like hardware.
 *     Overflow is dropped, not back-pressured.
 *   - RX (record) DMA reads back silence; the guest's record path
 *     is not exercised by any bundled application found so far.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/audio.h"
#include "qemu/fifo8.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "brain.h"

#define SAIF_CTRL    0x00
#define SAIF_STAT    0x10
#define SAIF_DATA    0x20
#define SAIF_VERSION 0x30

#define SAIF_CTRL_SFTRST  (1u << 31)
#define SAIF_CTRL_CLKGATE (1u << 30)
#define SAIF_CTRL_RUN     (1u << 0)

#define SAIF_STAT_BUSY    (1u << 31)

#define SAIF_VERSION_VALUE 0x01010000

#define SAIF_FIFO_CAPACITY (1u << 18)   /* 256 KiB, ~1.5 s of audio */
#define SAIF_SAMPLE_RATE   44100
#define SAIF_FRAME_BYTES   4            /* stereo s16 */

/* one DMA buffer (2048 bytes = 512 frames) per pacing tick */
#define SAIF_PACE_NS (NANOSECONDS_PER_SECOND * 512 / SAIF_SAMPLE_RATE)

static void saif_write_one(MxsSaifState *s, hwaddr addr, uint32_t value);

static bool saif_running(MxsSaifState *s)
{
    return (s->ctrl & SAIF_CTRL_RUN) &&
           !(s->ctrl & (SAIF_CTRL_SFTRST | SAIF_CTRL_CLKGATE));
}

static void saif_flush(MxsSaifState *s)
{
    uint8_t buf[4096];

    if (!s->voice || !saif_running(s)) {
        return;
    }
    while (!fifo8_is_empty(&s->tx_fifo)) {
        uint32_t n = fifo8_num_used(&s->tx_fifo);
        const uint8_t *p = fifo8_peek_bufptr(&s->tx_fifo,
                                             MIN(n, sizeof(buf)), &n);
        size_t written;

        if (n == 0) {
            break;
        }
        memcpy(buf, p, n);
        written = audio_be_write(s->audio_be, s->voice, buf, n);
        fifo8_drop(&s->tx_fifo, (uint32_t)MIN(written, (size_t)n));
        if (written < n) {
            break;   /* backend full: keep the rest queued */
        }
    }
}

static void saif_audio_cb(void *opaque, int avail)
{
    saif_flush(opaque);
}

static void saif_timer_cb(void *opaque)
{
    MxsSaifState *s = opaque;

    if (saif_running(s)) {
        /*
         * Pace the circular DMA chain at the rate hardware would
         * consume it: one buffer per tick, so the driver's
         * completion IRQs (and its buffer refills) are spread over
         * time instead of arriving in one burst at stream start.
         */
        if (s->dma) {
            mxs_dma_pace(s->dma, s->dma_ch, 1);
        }
        saif_flush(s);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  SAIF_PACE_NS);
    }
}

static void saif_maybe_start(MxsSaifState *s)
{
    if (!saif_running(s)) {
        return;
    }
    if (s->voice) {
        audio_be_set_active_out(s->audio_be, s->voice, true);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              SAIF_PACE_NS);
}

static void saif_push_tx(MxsSaifState *s, const uint8_t *data, uint32_t len)
{
    uint32_t space = fifo8_num_free(&s->tx_fifo);

    if (len > space) {
        s->dropped += len - space;
        len = space;
    }
    if (len) {
        fifo8_push_all(&s->tx_fifo, data, len);
    }
    saif_flush(s);
}

/* ------------------------------------------------------------------ */
/* DMA client hooks (APBX channels 4/5)                                */

void mxs_saif_dma_pio(MxsSaifState *s, int word_idx, uint32_t value)
{
    if (word_idx < 0) {
        return;
    }
    /* DMA PIO words land in consecutive 16-byte-spaced registers */
    saif_write_one(s, word_idx * 0x10, value);
}

void mxs_saif_dma_data(MxsSaifState *s, bool to_device,
                       dma_addr_t bufaddr, uint32_t len)
{
    g_autofree uint8_t *buf = g_malloc(len ? len : 1);

    if (to_device) {
        /* playback: guest memory -> SAIF TX FIFO */
        if (len) {
            address_space_read(&address_space_memory, bufaddr,
                               MEMTXATTRS_UNSPECIFIED, buf, len);
            saif_push_tx(s, buf, len);
        }
    } else {
        /* record: SAIF RX -> guest memory (silence) */
        if (len) {
            memset(buf, 0, len);
            address_space_write(&address_space_memory, bufaddr,
                                MEMTXATTRS_UNSPECIFIED, buf, len);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Register access                                                     */

static void saif_apply_write(MxsSaifState *s, uint32_t reg,
                             uint32_t mode, uint32_t value)
{
    uint32_t *slot;
    uint32_t old;

    switch (reg) {
    case SAIF_CTRL:
        slot = &s->ctrl;
        break;
    case SAIF_STAT:
        slot = &s->stat;
        break;
    case SAIF_DATA:
        if (mode == 0) {
            uint8_t b[4];

            b[0] = value;
            b[1] = value >> 8;
            b[2] = value >> 16;
            b[3] = value >> 24;
            saif_push_tx(s, b, 4);
        }
        return;
    default:
        return;   /* VERSION and undefined offsets: writes ignored */
    }

    old = *slot;
    switch (mode) {
    case 0: *slot = value; break;
    case 1: *slot |= value; break;
    case 2: *slot &= ~value; break;
    case 3: *slot ^= value; break;
    }

    if (reg == SAIF_CTRL) {
        /* STMP handshake: SFTRST rising edge gates the clock */
        if (!(old & SAIF_CTRL_SFTRST) && (s->ctrl & SAIF_CTRL_SFTRST)) {
            s->ctrl |= SAIF_CTRL_CLKGATE;
        }
        saif_maybe_start(s);
    }
}

static void saif_write_one(MxsSaifState *s, hwaddr addr, uint32_t value)
{
    saif_apply_write(s, addr & ~0xf, (addr >> 2) & 3, value);
}

static uint64_t saif_read(void *opaque, hwaddr addr, unsigned size)
{
    MxsSaifState *s = opaque;

    switch (addr & ~0xf) {
    case SAIF_CTRL:
        return s->ctrl;
    case SAIF_STAT:
        return s->stat | (saif_running(s) ? SAIF_STAT_BUSY : 0);
    case SAIF_VERSION:
        return SAIF_VERSION_VALUE;
    default:
        return 0;
    }
}

static void saif_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    saif_write_one(opaque, addr, (uint32_t)value);
}

static const MemoryRegionOps saif_ops = {
    .read = saif_read,
    .write = saif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */

static void saif_reset(DeviceState *dev)
{
    MxsSaifState *s = MXS_SAIF(dev);

    s->ctrl = SAIF_CTRL_SFTRST | SAIF_CTRL_CLKGATE;
    s->stat = 0;
    s->dropped = 0;
    fifo8_reset(&s->tx_fifo);
    if (s->voice) {
        audio_be_set_active_out(s->audio_be, s->voice, false);
    }
}

static void saif_realize(DeviceState *dev, Error **errp)
{
    MxsSaifState *s = MXS_SAIF(dev);
    struct audsettings as;

    if (!audio_be_check(&s->audio_be, errp)) {
        return;
    }

    as.freq = SAIF_SAMPLE_RATE;
    as.nchannels = 2;
    as.fmt = AUDIO_FORMAT_S16;
    as.big_endian = false;

    s->voice = audio_be_open_out(s->audio_be, NULL, "mxs-saif", s,
                                 saif_audio_cb, &as);
    if (!s->voice) {
        error_setg(errp, "mxs-saif: no audio voice");
        return;
    }
}

static void saif_init(Object *obj)
{
    MxsSaifState *s = MXS_SAIF(obj);

    memory_region_init_io(&s->iomem, obj, &saif_ops, s, "mxs-saif", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    fifo8_create(&s->tx_fifo, SAIF_FIFO_CAPACITY);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, saif_timer_cb, s);
}

static const Property saif_properties[] = {
    DEFINE_AUDIO_PROPERTIES(MxsSaifState, audio_be),
};

static void saif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "i.MX28 SAIF (I2S audio interface)";
    device_class_set_legacy_reset(dc, saif_reset);
    dc->realize = saif_realize;
    device_class_set_props(dc, saif_properties);
}

static const TypeInfo saif_info = {
    .name = TYPE_MXS_SAIF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MxsSaifState),
    .instance_init = saif_init,
    .class_init = saif_class_init,
};

static void saif_register(void)
{
    type_register_static(&saif_info);
}
type_init(saif_register)
