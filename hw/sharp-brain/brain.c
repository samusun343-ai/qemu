/*
 * SHARP Brain machine family (PW-SH6 platform, i.MX28).
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * Machine naming follows the Brain product lineages:
 *   pw-sh6  - the SH6-generation platform (854x480 WVGA+, GPIO
 *             matrix keyboard, LRADC 4-wire touch, eMMC + microSD)
 *   pw-aj2  - alias: PW-AJ2 shares the PW-SH6 hardware platform
 *             (its EBOOT identifies as "SHARP ED-AA2 V030 / ED-SH6")
 *   sharp-brain - generic alias for the default machine
 *
 * Storage wiring:
 *   -drive if=sd,index=0,...  -> on-board eMMC (SSP0, 8-bit, TYPE_EMMC)
 *   -drive if=sd,index=1,...  -> microSD slot (SSP1, 4-bit, TYPE_SD_CARD)
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/machines-qom.h"
#include "hw/core/boards.h"
#include "qemu/error-report.h"
#include "system/qtest.h"
#include "qemu/cutils.h"
#include "ui/console.h"
#include "brain.h"

typedef struct BrainMachineState {
    Imx28State soc;
    DeviceState *kbd;
} BrainMachineState;

static void brain_init(MachineState *machine)
{
    BrainMachineState *s = g_new0(BrainMachineState, 1);
    DriveInfo *di;
    BlockBackend *blk;
    BusState *bus;
    DeviceState *carddev;

    if (machine->ram_size != IMX28_DRAM_SIZE) {
        char *sz = size_to_str(IMX28_DRAM_SIZE);
        error_report("The Brain machines require exactly %s of RAM "
                     "(-m %s)", sz, sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_IMX28);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    /* DRAM at 0x40000000 */
    memory_region_add_subregion(get_system_memory(), IMX28_DRAM_BASE,
                                machine->ram);

    /* eMMC on SSP0 */
    di = drive_get(IF_SD, 0, 0);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    if (blk) {
        bus = qdev_get_child_bus(s->soc.ssp[0], "sd-bus");
        carddev = qdev_new(TYPE_EMMC);
        qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
        qdev_realize_and_unref(carddev, bus, &error_fatal);
    }

    /* microSD on SSP1 */
    di = drive_get(IF_SD, 0, 1);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    if (blk) {
        bus = qdev_get_child_bus(s->soc.ssp[1], "sd-bus");
        carddev = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
        qdev_realize_and_unref(carddev, bus, &error_fatal);
    }

    /* GPIO matrix keyboard (PW-SH6 layout; per-lineage board data) */
    s->kbd = qdev_new(TYPE_BRAIN_KBD);
    qdev_prop_set_string(s->kbd, "layout", "pw-sh6");
    object_property_set_link(OBJECT(s->kbd), "pinctrl",
                             OBJECT(s->soc.pinctrl), &error_fatal);
    object_property_add_child(OBJECT(machine), "keyboard", OBJECT(s->kbd));
    qdev_realize_and_unref(s->kbd, NULL, &error_fatal);

    /* virtual boot ROM (i.MX28 ROM emulation, driven by the eMMC) */
    if (!qtest_enabled()) {
        di = drive_get(IF_SD, 0, 0);
        if (di) {
            brain_bootrom_setup(&s->soc, blk_by_legacy_dinfo(di));
        } else {
            warn_report("no eMMC image: use -drive if=sd,index=0,"
                        "file=emmc.img to boot the real firmware");
        }
    }
}

static void brain_machine_init(MachineClass *mc)
{
    mc->desc = "SHARP Brain PW-SH6 / PW-AJ2 (i.MX28, Windows CE)";
    mc->init = brain_init;
    mc->max_cpus = 1;
    mc->default_ram_id = "brain.ram";
    mc->default_ram_size = IMX28_DRAM_SIZE;
    mc->ignore_memory_transaction_failures = false;
    mc->auto_create_sdcard = false;
    mc->alias = "sharp-brain";
}

DEFINE_MACHINE_ARM("pw-sh6", brain_machine_init)

/*
 * PW-AJ2: same hardware platform as PW-SH6 (the EBOOT in the AJ2 eMMC
 * identifies itself as "SHARP ED-AA2 V030 / ED-SH6").  Kept as a
 * distinct machine so per-model board data (keyboard layout) can
 * diverge later.
 */
static void pw_aj2_machine_init(MachineClass *mc)
{
    brain_machine_init(mc);
    mc->desc = "SHARP Brain PW-AJ2 (SH6 platform, i.MX28, Windows CE)";
    mc->alias = NULL;
}

DEFINE_MACHINE_ARM("pw-aj2", pw_aj2_machine_init)
