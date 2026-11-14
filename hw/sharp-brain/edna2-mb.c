/*
 * SHARP Brain EDNA2 sub-MCU mailbox.
 *
 * Copyright (c) 2026 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * The Brain board carries an EDNA2 sub-MCU next to the i.MX28.  The
 * host CPU talks to it through a 4 KiB shared register/RAM window at
 * physical 0x400ea000 (inside the DRAM aperture, carved out with
 * priority over DRAM).  Most of the window is plain shared RAM: EBOOT
 * and the WinCE EDNA2 host driver exchange data blocks through it
 * (observed blocks around +0x400, +0x800, +0xc00, +0xe00).
 *
 * The command channel, reverse-engineered from the reference QEMU's
 * BRAIN_MBTRACE log of a full boot (2026-11-14):
 *
 *   +0x03c  doorbell: host writes 1 to kick, 0 to clear afterwards
 *   +0x0e8  command byte latch (cmd 0x00 seen; written right after
 *           the doorbell kick)
 *   +0x404  completion flag: the MCU posts 0xffffffff when the
 *           command finishes (the host also pre-arms it to
 *           0xffffffff)
 *
 * The MCU model in the reference answers cmd 0x00 after a 3000 us
 * busy period by posting the completion flag and filling the
 * response block at +0x818..+0x823 with six 0x0100 halfwords.  That
 * block is what the guest's VMCopy/touchkey path consumes: without
 * an MCU behind the mailbox the window is plain DRAM, the response
 * never appears, and the guest logs
 * "touchkey_point_calibration() FAILED!" and never enables the
 * touch panel.
 *
 * Only cmd 0x00 has been observed on the wire so far; other command
 * bytes get the same busy period and completion flag but no
 * response block until evidence shows otherwise.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "migration/vmstate.h"
#include "brain.h"

#define EDNA2_MB_SIZE        0x1000
#define REG_DOORBELL         0x03c
#define REG_CMD              0x0e8
#define REG_DONE             0x404
#define REG_RESP             0x818
#define RESP_HALFWORDS       6
#define RESP_VALUE           0x0100
#define CMD_BUSY_US          3000

struct BrainEdna2MbState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint8_t regs[EDNA2_MB_SIZE];

    QEMUTimer *busy_timer;
    bool doorbell_armed;
    bool cmd_fresh;
    bool busy;
    uint8_t cmd;
};

void brain_edna2_mb_post_touchkey(BrainEdna2MbState *s, uint32_t code)
{
    /*
     * Sidebar touchkey events are delivered by the sub-MCU as event
     * codes in the +0x404 slot, which the guest's TouchSub thread
     * polls.  (The same slot doubles as the command completion flag;
     * the reference model posts touchkey codes there too.)
     */
    memcpy(&s->regs[REG_DONE], &code, sizeof(code));
}

static void edna2_mb_cmd_start(BrainEdna2MbState *s)
{
    s->busy = true;
    s->cmd_fresh = false;
    timer_mod(s->busy_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              CMD_BUSY_US * 1000);
}

static void edna2_mb_busy_cb(void *opaque)
{
    BrainEdna2MbState *s = opaque;
    uint32_t done = 0xffffffffu;

    s->busy = false;
    s->doorbell_armed = false;

    if (s->cmd == 0x00) {
        int i;
        for (i = 0; i < RESP_HALFWORDS; i++) {
            uint16_t v = RESP_VALUE;
            memcpy(&s->regs[REG_RESP + 2 * i], &v, sizeof(v));
        }
    }
    memcpy(&s->regs[REG_DONE], &done, sizeof(done));
}

static uint64_t edna2_mb_read(void *opaque, hwaddr addr, unsigned size)
{
    BrainEdna2MbState *s = opaque;
    uint64_t v = 0;

    if (addr + size <= EDNA2_MB_SIZE) {
        memcpy(&v, &s->regs[addr], size);
    }
    return v;
}

static void edna2_mb_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
    BrainEdna2MbState *s = opaque;
    hwaddr i;

    if (addr + size > EDNA2_MB_SIZE) {
        return;
    }
    memcpy(&s->regs[addr], &value, size);

    for (i = addr; i < addr + size; i++) {
        if (i == REG_DOORBELL) {
            if (s->regs[REG_DOORBELL] & 1) {
                if (!s->busy && s->cmd_fresh) {
                    edna2_mb_cmd_start(s);
                } else if (!s->busy) {
                    s->doorbell_armed = true;
                }
            } else {
                s->doorbell_armed = false;
            }
        }
        if (i == REG_CMD && !s->busy) {
            s->cmd = s->regs[REG_CMD];
            if (s->doorbell_armed) {
                edna2_mb_cmd_start(s);
            } else {
                s->cmd_fresh = true;
            }
        }
    }
}

static const MemoryRegionOps edna2_mb_ops = {
    .read = edna2_mb_read,
    .write = edna2_mb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = true,
    },
};

static void edna2_mb_reset(DeviceState *dev)
{
    BrainEdna2MbState *s = BRAIN_EDNA2_MB(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->doorbell_armed = false;
    s->cmd_fresh = false;
    s->busy = false;
    s->cmd = 0;
    timer_del(s->busy_timer);
}

static const VMStateDescription vmstate_edna2_mb = {
    .name = "brain-edna2-mb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(regs, BrainEdna2MbState),
        VMSTATE_BOOL(doorbell_armed, BrainEdna2MbState),
        VMSTATE_BOOL(cmd_fresh, BrainEdna2MbState),
        VMSTATE_BOOL(busy, BrainEdna2MbState),
        VMSTATE_UINT8(cmd, BrainEdna2MbState),
        VMSTATE_END_OF_LIST()
    },
};

static void edna2_mb_realize(DeviceState *dev, Error **errp)
{
    BrainEdna2MbState *s = BRAIN_EDNA2_MB(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &edna2_mb_ops, s,
                          TYPE_BRAIN_EDNA2_MB, EDNA2_MB_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
    s->busy_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, edna2_mb_busy_cb, s);
}

static void edna2_mb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = edna2_mb_realize;
    dc->vmsd = &vmstate_edna2_mb;
    device_class_set_legacy_reset(dc, edna2_mb_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo edna2_mb_info = {
    .name = TYPE_BRAIN_EDNA2_MB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BrainEdna2MbState),
    .class_init = edna2_mb_class_init,
};

static void edna2_mb_register_types(void)
{
    type_register_static(&edna2_mb_info);
}
type_init(edna2_mb_register_types)
