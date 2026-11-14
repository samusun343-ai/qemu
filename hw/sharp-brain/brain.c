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
    DeviceState *edna2_mb;
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

    /*
     * EDNA2 sub-MCU mailbox device.  It is created before the SoC is
     * realized so the LRADC can link to it: the LRADC posts sidebar
     * touchkey events into the mailbox (see lradc.c).
     */
    s->edna2_mb = qdev_new(TYPE_BRAIN_EDNA2_MB);
    object_property_add_child(OBJECT(machine), "edna2-mb",
                              OBJECT(s->edna2_mb));
    object_property_set_link(OBJECT(s->soc.lradc), "edna2-mb",
                             OBJECT(s->edna2_mb), &error_fatal);

    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    /*
     * Board straps, read through PINCTRL DIN0 (EBOOT maps the PINCTRL
     * block at VA 0xb0000000 and reads "capability bits" as DIN0 bits):
     *
     *   GPIO0_5 - "microSD slot populated".  EBOOT's early-init query
     *             (edna2 ED-PARAM read at the CheckResumeInfo site)
     *             gates the whole slot-1 path on it: 0x4006fe94(1)
     *             refuses to open SSP1, g_bSDHCExist[1] stays 0 and
     *             both EBOOT and the kernel report "NO SD card1".
     *             The SH6 platform (PW-AJ1/AJ2/SH6) has the slot, so
     *             the strap reads high.
     */
    mxs_pinctrl_set_input(s->soc.pinctrl, 0, 1u << 5, 1u << 5);

    /* DRAM at 0x40000000 */
    memory_region_add_subregion(get_system_memory(), IMX28_DRAM_BASE,
                                machine->ram);

    /*
     * EDNA2 sub-MCU mailbox: a 4 KiB shared window inside the DRAM
     * aperture at 0x400ea000.  It must shadow DRAM (priority 1, as in
     * the reference machine's memory tree): the guest exchanges data
     * blocks with the sub-MCU here, and command completion plus the
     * response data are posted by the MCU side, never by the guest.
     */
    sysbus_realize(SYS_BUS_DEVICE(s->edna2_mb), &error_fatal);
    memory_region_add_subregion_overlap(
        get_system_memory(), BRAIN_EDNA2_MB_BASE,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(s->edna2_mb), 0), 1);

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
        /*
         * Card-detect: the BSP reads the microSD slot's CD level
         * through LRADC channel 5 (the SSP1 card-detect pad routed as
         * an analog input); a present card reads low.
         */
        MXS_LRADC(s->soc.lradc)->sd_card_present = true;
        mxs_pinctrl_set_input(s->soc.pinctrl, 0, 1u << 0, 1);
    }

    /* GPIO matrix keyboard (PW-SH6 layout; per-lineage board data) */
    s->kbd = qdev_new(TYPE_BRAIN_KBD);
    qdev_prop_set_string(s->kbd, "layout", "pw-sh6");
    object_property_set_link(OBJECT(s->kbd), "pinctrl",
                             OBJECT(s->soc.pinctrl), &error_fatal);
    object_property_add_child(OBJECT(machine), "keyboard", OBJECT(s->kbd));
    sysbus_realize(SYS_BUS_DEVICE(s->kbd), &error_fatal);

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

/*
 * PW-AJ1: another SH6-platform model, kept as a distinct machine so
 * per-model board data (keyboard layout, panel) can diverge later.
 */
static void pw_aj1_machine_init(MachineClass *mc)
{
    brain_machine_init(mc);
    mc->desc = "SHARP Brain PW-AJ1 (SH6 platform, i.MX28, Windows CE)";
    mc->alias = NULL;
}

DEFINE_MACHINE_ARM("pw-aj1", pw_aj1_machine_init)
