# Sharp Brain virtualization for QEMU (i.MX28 family)

This directory implements full-machine virtualization of the SHARP Brain
series of Windows CE electronic dictionaries, built around the
NXP (Freescale) i.MX283 SoC (ARM926EJ-S @ 454 MHz, ARMv5TEJ).

The machine is named after the Brain product lineages. PW-AJ2 belongs to
the same hardware platform as PW-SH6 (both are 4th-generation, 854x480
"wide VGA+" models); they are modelled by the `pw-sh6` machine, with
`pw-aj2` as an alias. Machines for other lineages (pw-sh1..sh5, pw-sa*,
pw-sj*, pw-hc* ...) share the same SoC implementation and differ in board
data (LCD panel, keyboard matrix, touch wiring).

## Implemented / planned machines

| machine      | alias       | lineage      | LCD        | RAM   | notes |
|--------------|-------------|--------------|------------|-------|-------|
| `pw-sh6`     | `pw-aj2`, `sharp-brain` | SH6 platform | 854x480 16bpp | 128MB | eMMC + microSD, DUART console |
| `pw-aj1`     | -           | SH6 platform | 854x480 16bpp | 128MB | same board data as pw-sh6 |
| (future) `pw-sh1`..`pw-sh5` | -    | 3rd gen      | 800x480    | 128MB | mostly same board data |

## Booting

    qemu-system-arm -M pw-aj2 \
        -drive if=sd,index=0,file=emmc.img,format=raw \
        [-drive if=sd,index=1,file=sd.img,format=raw] \
        -serial mon:stdio -display vnc=:1

`if=sd,index=0` is the on-board eMMC (SSP0, 8-bit bus, TYPE_EMMC),
`index=1` is the microSD slot (SSP1, 4-bit bus).

## i.MX28 boot chain (reverse engineered from the PW-AJ2 eMMC dump)

The boot flow was reconstructed by analysing the eMMC image shipped at
`samusun343-ai/wince-imgs` release tag `emmc` (`sharp-brain-pw-aj2-emmc`,
6,605,935,104 bytes) and cross-checked against the brain-hackers
`u-boot.brain`/`buildbrain` artifacts.

### eMMC layout (user area)

| LBA range        | contents |
|------------------|----------|
| 0                | MBR (see partitions below) |
| 1                | `STMP`-style marker: `STMP`, major=1, minor=0, imageBlocks=0x104 (=260) - unused by the boot flow |
| 256..1279        | **partition type 0x53 (SigmaTel boot partition)**: 4-sector boot config block + boot slots |
| 260..594         | boot slot A: `[20B SHA1 digest][SB v1.1 boot image]` (335 sectors) |
| 1284..1618       | boot slot B: identical copy of slot A (A/B redundancy) |
| 2304..67839      | partition type 0x10: Windows CE OS image (32MB) |
| 598031..1624063  | partition type 0x0E: FAT16 (501MB) |
| 1624073..        | partition type 0x0B: FAT32 (5.5GB) - user/dictionary data |

### Boot config block (type 0x53 partition, LBA 256)

Same structure as u-boot's `mx28_create_sd_image()` output
(`struct mx28_sd_config_block`):

    +0x00  magic 0x00112233
    +0x04  primary boot tag  (1)
    +0x08  secondary boot tag (2)
    +0x0C  number of copies  (2)
    +0x18  slot[0]: tag=1, first_sector=0x104 (260), sector_count=0x14f (335)
    +0x2C  slot[1]: tag=2, first_sector=0x504 (1284), sector_count=0x14f

### SB (Secure Boot, "Encore/Piano") image format

The SB file layout follows Freescale's elftosb `EncoreBootImage`:

* 20-byte SHA1 digest header (doubles as AES CBC IV; first 16 bytes)
* `STMP` signature, version 1.1, flags 0x0001
* section table + DEK dictionary; sections are AES-128-CBC encrypted
  with a data encryption key that itself is CBC-encrypted with the
  **all-zero KEK** (the Brain parts have no OTP keys blown, which is why
  community u-boot.sb images boot on real hardware)
* boot commands: NOP(0x00) TAG(0x01) LOAD(0x02) FILL(0x03) JUMP(0x04)
  CALL(0x05) MODE(0x06); command checksum = 0x5A + sum(bytes 1..15)

### PW-AJ2 boot stream (slot A/B)

    LOAD   0x00000004  0x2800    ; Stage0 preloader (10KB, IRAM) - DRAM/pinmux init
    LOAD   0x00008000  0x20      ; IVT (HAB v4): entry=0x00000004, self=0x8000
    CALL   0x00008000            ; ROM jumps to IVT entry (ARM), stage0 returns
    LOAD   0x400b4000  0x43a     ; 1082-byte parameter block
    LOAD   0x40060000  0x27000   ; Stage1 = EBOOT (Freescale i.MX28 WinCE BSP
                                  ; bootloader, customized by Sharp)
    LOAD   0x00008000  0x20      ; IVT: entry=0x40060000
    CALL   0x00008000            ; EBOOT runs (never returns)

EBOOT identifies itself as `SHARP   ED-AA2  V030  /  ED-SH6`, reads its
boot configuration from eMMC, loads the Windows CE image (type 0x10
partition) and jumps into the kernel. EBOOT also implements the
firmware-update path that executes `EDSH6EXE.BIN` (nk.bin wrapper) from
the microSD card - this is how brain-hackers' U-Boot/Linux gets started
on real hardware.

The 32-byte IVT at 0x8000 (HAB v4):

    +0x00  tag 0xD1, length 0x20, version 0x40
    +0x04  entry point (bit0 = Thumb)
    +0x0C  DCD address (0 here)
    +0x10  boot data address (0 here)
    +0x14  self address (0x8000)

## i.MX28 memory map (from linux-brain `imx28.dtsi`)

| address | block |
|---------|-------|
| 0x00000000..0x0001ffff | IRAM 128KB (ROM shadow at reset) |
| 0x40000000..0x47ffffff | DRAM (128MB on Brain) |
| 0x80000000 | ICOLL interrupt collector |
| 0x80004000 | APBH DMA controller (16 channels: SSP0-3=0-3, GPMI=4-11, HSADC=12, LCDIF=13) |
| 0x80010000/0x80012000 | SSP0 (eMMC, 8-bit) / SSP1 (microSD, 4-bit) |
| 0x80014000/0x80016000 | SSP2 (SPI sub-LCD) / SSP3 |
| 0x80018000 | PINCTRL (with GPIO banks 0-4) |
| 0x8001c000 | DIGCTL (microsecond counter at +0xc0) |
| 0x80024000 | APBX DMA |
| 0x80030000 | LCDIF (DMA channel 13, IRQ 38) |
| 0x80040000 | CLKCTRL |
| 0x80044000 | POWER |
| 0x80050000 | LRADC (4-wire resistive touch on Brain) |
| 0x80058000/0x8005a000 | I2C0 (SGTL5000 codec, keyboard ctrl on some models) / I2C1 |
| 0x80068000 | TIMROT (IRQ 48-51) |
| 0x8006a000..0x80072000 | AUART0-4 |
| 0x80074000 | DUART - **ARM PL011** debug console (IRQ 47) |
| 0x800e0000 | EMI/DRAM controller |
| 0x800f0000 | FEC ethernet |

Key IRQs: SSP0=96, SSP1=97, LCDIF=38, DUART=47, TIMROT=48-51,
LRADC touch=10.

All mxs register blocks use the 16-byte "stmp" layout
`[REG, SET(+4), CLR(+8), TOG(+0xc)]`.

### SSP (SD/MMC host) notes

* Registers (16-byte spaced): CTRL0 0x00, CMD0 0x10, CMD1 0x20,
  XFER_SIZE 0x30, BLOCK_SIZE 0x40, COMPREF 0x50, COMPMASK 0x60,
  TIMING 0x70, CTRL1 0x80, DATA 0x90, SDRESP0-3 0xa0-0xd0, DDR_CTRL
  0xe0, DLL_CTRL 0xf0, STATUS 0x100, DLL_STS 0x110.
* Writing CTRL0 with RUN (bit 29) starts the command (u-boot
  `mxsmmc.c` register-level path).
* With DMA enabled (CTRL1.DMA_ENABLE), command issue happens through
  APBH DMA PIO descriptors: pio_words[j] is written to
  `SSP_base + 0x10*j` (CTRL0, CMD0, CMD1, XFER_SIZE...). The DMA
  completion of the PIO descriptor effectively asserts RUN. Data then
  flows between the SSP FIFO and memory through a DMA descriptor with
  COMMAND=WRITE(1)/READ(2) + WAIT4END.
* STATUS.CARD_DETECT (bit 28) reads 0 when a card is present.

### Keyboard (PW-SH6 matrix, from linux-brain `imx28-pwsh6.dts`)

8 rows (inputs, GPIO4_0..4_7, pulled up) x 7 columns (outputs:
GPIO2_16..2_21 + GPIO4_8). Keys map to a standard PC-style map plus a
`symbols` key (matrix 2,4) that acts as a modifier selecting the
alternate blue legends (numbers/symbols) - the same trick the real
hardware uses.

Host keyboard -> guest mapping policy (per project requirements):
plain PC keys map 1:1; Brain-specific keys that have no PC equivalent
(dictionary keys: Home/Esc, History, My辞書, Ichiran, Kokugo, 音声,
英和, marker, jump...) are reached through key combinations
(see `brain-kbd.c` for the exact table).

## WinCE suspend/resume (works end to end)

The power manager's auto-suspend cycle runs the complete real-firmware
flow inside the emulator:

    WinCE idle timeout
      -> all PDD PowerDown handlers (display, SD, I2C, battery, USB...)
      -> VMC_PowerDown: kernel context saved to the blob at PA 0x400e8400,
         resume stub copied into IRAM, DRAM self-refresh requested,
         POWER_RESET.PWD written with the 0x3e77 unlock key
      -> POWER_RESET.PWD = the automatic power cycle the real i.MX28
         performs while 5V is present: guest reset (DRAM survives,
         like self-refreshed DRAM on hardware)
      -> boot ROM re-runs the SB stream (stage0 + EBOOT)
      -> EBOOT CheckResumeInfo: blob checksums valid -> resume path
      -> EBOOT restores cp15 + banked registers, jumps to the kernel
         resume entry (blob+0x38)
      -> kernel OEMResume, PMWM_EDPOWERONOFF ON (display back on),
         all PDD PowerUp handlers

With no user input the PM cycles suspend/resume indefinitely, exactly
like a real device sitting on USB power.  QEMU-divergence fixes needed
for this (see the individual file comments for details):

* `cp15 c9/CRm=1` RAZ/WI (registered from the SoC model - no upstream
  `target/arm` change): OEMSaveCpuContext blindly saves the ARM926EJ-S
  test/lockdown register space; unknown-cp15 reads used to raise UNDEF
  and crash the suspend thread inside nk.exe.
* `POWER_RESET`/`CLKCTRL_RESET` handling in `misc.c`: PWD (unlock key
  0x3e77) = power-cycle reset, PWD_OFF = emulator shutdown, CLKCTRL
  RESET.CHIP = software reboot.
* SIMDBG block as persistent storage: EBOOT parks resume scratch
  across the power-cycle reset there.
* EBOOT resume fixup in `bootrom.c` (pattern-matched at SB LOAD time):
  the MMU-off continuation jump and the position-dependent final-phase
  VA computation are redirected to their still-mapped VAs, and the
  context-blob literal is retargeted from PA to the uncached alias VA.
  (Real hardware executes the instruction after the SCTLR write from
  the pipeline; QEMU ends the TB there and re-fetches under the new
  MMU state - the same divergence already worked around for the DVFS
  trampolines with the IRAM mirror.)

The EBOOT splash screen renders on the LCDIF console during boot; the
kernel's display driver scans out the framebuffer at PA 0x40100000
continuously.

## Known issue: the FAT volumes never mount (no shell/UI)

After all drivers load (including the GWES display/touch/keyboard
DDIs) the system goes idle and the PM auto-suspends.  The dictionary
launcher never starts: block-layer tracing shows the WinCE storage
manager never reads the FAT16/FAT32 partitions at all (zero reads
past the OS partition), so no volume mounts and no launcher EXE
(WCEPRJ.EXE / the \SHARP startup folder on FAT32) can load.  The
screen stays black because the framebuffer simply contains no UI.
Tracking the mount chain (FSDMGR / the Sharp "FST" manager seen in
`[FST]HandleAddDevice` logs) is the next milestone.

Update (Nov 2026): the mount chain has been traced end to end with the
tools in tools/. The storage manager (FSDMGR + mspart, running in the
driver host process) DOES knock on every DSK after registration: it
queries geometry (ioctl 0x71C00/0x71C24), the profile name (0x71800,
"NandDic") and then issues scatter reads (ioctl 0x75C08, 28-byte SG
request {start, count, ..., buf, 0x200}). In the emulator those reads
"complete" successfully but return an all-zero buffer and no SD command
is ever issued: the request dies somewhere between edna2_SDMemory.dll
(NDF device), sdbus2.dll (SD bus) and sdhc.dll (SHC host controller),
so mspart sees a zero sector 0 (no MBR) and never mounts. The remaining
work is finding where in that chain the request is dropped; module
bases in the driver-host process are stable across boots (FSDMGR
0xc0330000, mspart 0xc0360000, sdbus2 0xc0500000, sdhc 0xc0780000,
edna2_SDMemory 0xc07d1000, block_readonly 0xc07e0000, block_writable
0xc0800000/0xc0810000/0xc0820000, block_com 0xc0850000).

Update (Nov 11 2026): full 6.6GB eMMC image reconstructed from the
split release assets and re-verified - boot behaviour is identical, so
the blocker is guest-driver-side, not an image artifact.  Boot
timeline with per-10s SSP command counts (qemu-dbg.log, CMD* traces):

    t<10s    EBOOT: 242 SSP commands (NK read from the OS partition)
    t=20s    [FST]HandleAddDevice + DSK_Init regions - 405 commands
    t=20-190s  idle, NO new SSP activity (405 -> 409)
    t>200s   PM suspend/resume cycles (409 -> 424, resume re-init)

So after the SD bus finishes enumerating the eMMC (CMD0/CMD8/CMD55/
CMD1/CMD2/CMD3/CMD9/CMD7/CMD16 + eMMC CMD6 SWITCH to 8-bit @52MHz,
all with correct responses and SSP IRQ 96 pulses), FSDMGR's scatter
reads still never reach the SSP.  The previously observed CMD17/18
traffic in the debug log is all bootloader/kernel image loading, not
the mount path.  Next step: disassemble the sdhc.dll/sdbus2.dll
scatter-read path from the nk.nb0 (XIP at PA 0x40200000 in the RAM
dump) and find the divergence.

## Known issue: microSD enumeration stops before the bus

The BSP reads card presence through the LRADC card-detect channels
(CH4 = eMMC, always present; CH5 = microSD, modelled from the
attached drive image - see lradc.c).  With a card image on
`if=sd,index=1`, EBOOT's detection chain (0x400795b8 in the dump)
still exits "NO SD card1" before any SD command reaches SSP1 -
one of its early setup sub-calls fails in the emulator.  The slot
wiring itself (SSP1 sd-bus + CD level) is in place; following the
chain call by call is the remaining work for microSD support.

## References

* eMMC dump: samusun343-ai/wince-imgs, release tag `emmc`
* brain-hackers/linux-brain device trees (`imx28-brain.dtsi`,
  `imx28-pwsh6.dts` ...)
* brain-hackers/u-boot-brain (register headers under
  `arch/arm/include/asm/arch-mxs/`)
* Freescale elftosb (SB image format, `common/EncoreBootImage.*`,
  `common/IVTDataSource.*`)
* u-boot `tools/mxsboot.c` (boot config block layout, 0x53 partition)
