/*
 * ROHM BU26154MUV audio codec: I2C presence / ACK stub.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The SHARP Brain's audio codec is the ROHM BU26154MUV ("Monaural
 * Audio CODEC with Touch Panel Interface", 24-bit, VQFN040V6060;
 * applications include electronic dictionaries).  The brainux DTS
 * stubbed the sound node as an SGTL5000 because the real codec was
 * unknown to them; the BU26154 is the correct part (user-confirmed).
 *
 * This is a presence/ACK stub, NOT a full codec:
 *
 *   - I2C slave at 7-bit address 0x1A (datasheet: SAD pin = "L" ->
 *     "0011010"; SAD = "H" would be 0x1B).  Attached to I2C0, where
 *     the board's audio codec lives (brainux imx28-brain.dtsi puts
 *     the codec node under i2c@80058000; the board wiring is the
 *     same, only the part number in the DTS is a wrong stub).
 *   - ACKs its address and ACKs register writes; register reads
 *     return the stored byte (0x00 at reset).  This lets the WinCE
 *     WaveDev driver (wavedev2_BU26154.dll) complete its "Initialize
 *     Audio Codec" probe instead of NAKing every transfer.
 *   - Protocol (datasheet Fig. 24/25): the first byte of a write
 *     transaction is the internal register INDEX; following bytes
 *     are data written to INDEX, INDEX+1, ... (continuous write,
 *     auto-increment).  A read transaction returns the byte at the
 *     INDEX set by the preceding pointer write, then auto-increments.
 *     Unwritten registers read 0x00 (the datasheet's reserved bits
 *     also read 0).  The codec has three register maps selected by
 *     the MAPCON register (INDEX 0x1c); this stub accepts MAPCON
 *     writes but keeps a single flat register file across maps.
 *   - No DSP/EQ/ALC/touch processing and no SAIF (I2S) audio path
 *     yet: the guest makes zero I2C transfers at boot, so the stub
 *     is unexercised until WaveDev runs (after the home screen).
 *
 * Datasheet: analysis/rohm/BU26154MUV.pdf (ROHM TSZ22111 Rev.002).
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "brain.h"

/* register INDEX map (datasheet "Register map", MAP0): 0x00, 0x01,
 * 0x0c-0x0f, 0x10-0x15, 0x1c/0x1d (MAPCON), 0x20-0x27, 0x2c-0x2f,
 * 0x30/0x31, 0x3a/0x3b, 0x3e/0x3f, 0x46-0x4f, 0x58-0x5d, 0x60/0x61...
 * The whole INDEX space is one byte, so a flat 128-entry file
 * covers every defined register. */
#define BU26154_NREGS  0x80

typedef struct Bu26154State {
    I2CSlave parent_obj;

    uint8_t regs[BU26154_NREGS];
    uint8_t ptr;        /* current register INDEX */
    bool have_ptr;      /* true once the INDEX byte arrived */
} Bu26154State;

OBJECT_DECLARE_SIMPLE_TYPE(Bu26154State, BU26154)

static int bu26154_event(I2CSlave *i2c, enum i2c_event event)
{
    Bu26154State *s = BU26154(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* new write transaction: the first byte is the INDEX */
        s->have_ptr = false;
        break;
    case I2C_START_RECV:
    case I2C_FINISH:
    case I2C_NACK:
    default:
        break;
    }
    return 0;
}

static int bu26154_send(I2CSlave *i2c, uint8_t data)
{
    Bu26154State *s = BU26154(i2c);

    if (!s->have_ptr) {
        /* first byte: register INDEX */
        s->ptr = data & (BU26154_NREGS - 1);
        s->have_ptr = true;
        return 0;   /* ACK */
    }

    /* data byte: write to INDEX, then auto-increment */
    s->regs[s->ptr] = data;
    s->ptr = (s->ptr + 1) & (BU26154_NREGS - 1);
    return 0;       /* ACK */
}

static uint8_t bu26154_recv(I2CSlave *i2c)
{
    Bu26154State *s = BU26154(i2c);
    uint8_t v = s->regs[s->ptr];

    /* continuous read: auto-increment */
    s->ptr = (s->ptr + 1) & (BU26154_NREGS - 1);
    return v;
}

static void bu26154_reset(DeviceState *dev)
{
    Bu26154State *s = BU26154(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->have_ptr = false;
}

static void bu26154_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->desc = "ROHM BU26154 audio codec (I2C presence stub)";
    device_class_set_legacy_reset(dc, bu26154_reset);
    sc->event = bu26154_event;
    sc->recv = bu26154_recv;
    sc->send = bu26154_send;
}

static const TypeInfo bu26154_info = {
    .name = TYPE_BU26154,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Bu26154State),
    .class_init = bu26154_class_init,
};

static void bu26154_register(void)
{
    type_register_static(&bu26154_info);
}
type_init(bu26154_register)
