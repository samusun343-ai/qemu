/*
 * ROHM BU26154MUV audio codec: I2C register model.
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
 * Model scope:
 *
 *   - I2C slave at 7-bit address 0x1A (datasheet: SAD pin = "L" ->
 *     "0011010"; SAD = "H" would be 0x1B), on I2C0.
 *   - Register file per the datasheet (TSZ22111 Rev.001/002):
 *     the INDEX space is byte-wide pairs - even INDEX = read,
 *     odd INDEX = write of the same register - and three register
 *     maps are selected by the MAPCON register (canonical INDEX
 *     0x1c, present in every map).  Reset values follow the
 *     datasheet's "(Initial)" column for the defined registers.
 *     A write of SOFTRST (MAP0 INDEX 0x10 bit 0) restores them.
 *   - Protocol (datasheet "2 wire serial interface"): the first
 *     byte of a write transaction is the register INDEX; following
 *     bytes are data for INDEX, INDEX+2, ... (continuous write,
 *     auto-increment over register pairs).  A read transaction
 *     returns the byte at the INDEX set by the preceding pointer
 *     write, then auto-increments the same way.
 *   - The guest's only consumer is wavedev2_BU26154.dll, which
 *     programs the full init sequence (SR = 44.1 kHz, clock/PLL,
 *     SAI format, DAC/SP power, EQ coefficient block, ~150 writes)
 *     over APBX-DMA I2C at first playback and reads back the
 *     digital-volume function register (MAP0 INDEX 0x68).  All of
 *     that state is modelled; the PCM path itself is the SoC's
 *     SAIF (saif.c) - the codec's DSP/EQ/ALC are not simulated, so
 *     samples pass through unmodified, as they would with the
 *     guest's flat EQ settings.
 *   - The codec's touch-panel interface (MAP1 INDEX 0x60-0x64) is
 *     stored but inert: no guest driver implements its protocol
 *     (verified by NK-wide string search in the touch phase).
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "brain.h"

#define BU26154_NMAPS  3
#define BU26154_NREGS  0x100   /* canonical (even) INDEX space */

#define BU26154_REG_MAPCON   0x1c
#define BU26154_REG_SOFTRST  0x10   /* MAP0 */

typedef struct Bu26154State {
    I2CSlave parent_obj;

    uint8_t regs[BU26154_NMAPS][BU26154_NREGS];
    uint8_t mapcon;     /* MAPCON register (shared by all maps) */
    uint8_t ptr;        /* current register INDEX (raw, with R/W) */
    bool have_ptr;      /* true once the INDEX byte arrived */
} Bu26154State;

OBJECT_DECLARE_SIMPLE_TYPE(Bu26154State, BU26154)

/*
 * Datasheet reset values for the defined registers ("(Initial)"
 * column of the register map).  Everything not listed resets to
 * 0x00.  Register addresses are the canonical (even) INDEX.
 */
static const struct {
    uint8_t map;
    uint8_t reg;
    uint8_t val;
} bu26154_defaults[] = {
    /* MAP0 */
    { 0, 0x26, 0x04 },  /* Speaker Amplifier Power Management */
    { 0, 0x2c, 0x01 },  /* TSDEN */
    { 0, 0x3a, 0x0a },  /* AVVOL */
    { 0, 0x3e, 0xff },  /* PDATT (playback digital attenuator) */
    { 0, 0x5a, 0x80 },  /* MINVOL */
    { 0, 0x60, 0xc0 },  /* SAI Transmitter Control */
    { 0, 0x62, 0xc0 },  /* SAI Receiver Control */
    { 0, 0x66, 0x01 },  /* DSP Filter Function Enable (HPF1EN) */
    { 0, 0x6c, 0xff },  /* RDVOL */
    { 0, 0x70, 0xff },  /* Effect VOL */
    { 0, 0x72, 0x20 },  /* RALCVOL */
    { 0, 0x74, 0xe7 },  /* EQGAIN0 */
    { 0, 0x76, 0xe7 },  /* EQGAIN1 */
    { 0, 0x78, 0xe7 },  /* EQGAIN2 */
    { 0, 0x7a, 0xe7 },  /* EQGAIN3 */
    { 0, 0x7c, 0xe7 },  /* EQGAIN4 */
    { 0, 0xb2, 0x02 },  /* RALCATK */
    { 0, 0xb4, 0x03 },  /* RALCDCY */
    { 0, 0xb8, 0x17 },  /* RALCLVL */
    { 0, 0xbc, 0x22 },  /* Record ALC Saturation Detect */
    { 0, 0xc0, 0x04 },  /* PALCATK */
    { 0, 0xc2, 0x05 },  /* PALCDCY */
    { 0, 0xc4, 0x1b },  /* PALCLVL */
    { 0, 0xc8, 0x20 },  /* PALCVOL */
    { 0, 0xcc, 0x11 },  /* ALC Fast Release */
    { 0, 0xce, 0x11 },  /* Playback Limiter Fast Release */
    { 0, 0xe8, 0x01 },  /* MIC Select (MIN1EN) */
    /* MAP1 */
    { 1, 0x28, 0x01 },  /* SCGAIN */
    { 1, 0x60, 0x78 },  /* Touch ADC Control */
    { 1, 0xde, 0xd3 },  /* NGMINGAIN */
    { 1, 0xe0, 0x12 },  /* NGTH */
    { 1, 0xe2, 0x02 },  /* NGTHHYS */
    { 1, 0xe4, 0x14 },  /* NGSLOPE */
    { 1, 0xe6, 0x02 },  /* NGGAINSTEP */
    { 1, 0xe8, 0x10 },  /* Noise Gate Time Setting */
    { 1, 0xea, 0x15 },  /* NGFDOUT / NGFDIN */
    /* MAP2 */
    { 2, 0x00, 0x01 },  /* EXMODE */
    { 2, 0x04, 0x26 },  /* Audio Analog Control2 */
    { 2, 0x12, 0x01 },  /* AREFI1EN */
    { 2, 0x66, 0x01 },  /* Filter Function Enable B (HPF1ENB) */
    { 2, 0x70, 0xff },  /* Effect VOLB */
    { 2, 0x72, 0xff },  /* PDATTB */
    { 2, 0x74, 0xe7 },  /* EQGAIN0B */
    { 2, 0x76, 0xe7 },  /* EQGAIN1B */
    { 2, 0x78, 0xe7 },  /* EQGAIN2B */
    { 2, 0x7a, 0xe7 },  /* EQGAIN3B */
    { 2, 0x7c, 0xe7 },  /* EQGAIN4B */
};

static void bu26154_load_defaults(Bu26154State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned i = 0; i < ARRAY_SIZE(bu26154_defaults); i++) {
        s->regs[bu26154_defaults[i].map][bu26154_defaults[i].reg] =
            bu26154_defaults[i].val;
    }
    s->mapcon = 0;
}

static uint8_t bu26154_reg_read(Bu26154State *s, uint8_t reg)
{
    if (reg == BU26154_REG_MAPCON) {
        return s->mapcon;
    }
    return s->regs[s->mapcon % BU26154_NMAPS][reg];
}

static void bu26154_reg_write(Bu26154State *s, uint8_t reg, uint8_t val)
{
    if (reg == BU26154_REG_MAPCON) {
        s->mapcon = val & 0x03;
        return;
    }
    if (s->mapcon == 0 && reg == BU26154_REG_SOFTRST && (val & 1)) {
        /* software reset: restore defaults; the bit self-clears */
        bu26154_load_defaults(s);
        return;
    }
    s->regs[s->mapcon % BU26154_NMAPS][reg] = val;
}

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
        /* first byte: register INDEX (odd = write, even = read) */
        s->ptr = data;
        s->have_ptr = true;
        return 0;   /* ACK */
    }

    /* data byte: write the register, auto-increment to the next
     * register pair (the R/W bit stays in the low bit) */
    bu26154_reg_write(s, s->ptr & ~1, data);
    s->ptr += 2;
    return 0;       /* ACK */
}

static uint8_t bu26154_recv(I2CSlave *i2c)
{
    Bu26154State *s = BU26154(i2c);
    uint8_t v = bu26154_reg_read(s, s->ptr & ~1);

    /* continuous read: auto-increment to the next register pair */
    s->ptr += 2;
    return v;
}

static void bu26154_reset(DeviceState *dev)
{
    Bu26154State *s = BU26154(dev);

    bu26154_load_defaults(s);
    s->ptr = 0;
    s->have_ptr = false;
}

static void bu26154_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->desc = "ROHM BU26154 audio codec";
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
