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

8 rows (guest-driven outputs, GPIO4_0..4_7) x 7 columns (inputs,
GPIO2_16..2_21 + GPIO4_8, pulled up). The scan direction is
rows-out / columns-in: the driver drives the rows low and arms
PINCTRL level interrupts on the columns (a pressed key pulls its
column low). This was proven by tracing the guest's own GPIO
programming (boottest16): it sets DOE for GPIO4_0..7, drives
DOUT low, and programs PIN2IRQ/IRQLEV/IRQPOL/IRQEN for
GPIO2_16..21 + GPIO4_8. The model therefore injects keypresses
as column levels, and pulls the columns up at realise, so the
driver sees idle-high columns when it arms the IRQs (without
the pull-ups the columns read low, the level IRQ fires
immediately on enable, and the driver tears the IRQs back down
- which is why host keys were silently ignored before this fix).
Keys map to a standard PC-style map plus a `symbols` key
(matrix 2,4) that acts as a modifier selecting the alternate
blue legends (numbers/symbols) - the same trick the real
hardware uses.

Host keyboard -> guest mapping policy (per project requirements):
plain PC keys map 1:1; Brain-specific keys that have no PC equivalent
(dictionary keys: Home/Esc, History, My辞書, Ichiran, Kokugo, 音声,
英和, marker, jump...) are reached through key combinations
(see `brain-kbd.c` for the exact table).

Complete host-key table (brain-kbd.c, layout "pw-sh6"):

| host key(s) | contact | note |
|---|---|---|
| A..Z (QWERTY letters) | direct 1:1 | the 26 matrix letter keys |
| Space, Enter/KpEnter, Backspace | direct | |
| Up/Down/Left/Right | direct | the 4-way cursor pad |
| Left/Right Shift | direct | Shift |
| Esc / Home | row3 col6 | Brain Home key |
| PageUp / PageDown | direct | |
| Insert | row1 col2 | alias for History |
| Minus / KpMinus | direct | |
| 1..0, Kp0..Kp9 | symbols layer | 記号 + Q/W/E/R/T/Y/U/I/O/P (blue legends) |
| ` = ~ [ ] \ ; ' , . / | symbols layer | 記号 + base key |
| Ctrl+B | row0 col3 | 戻る (back) |
| Ctrl+H | row1 col2 | 履歴 History |
| Ctrl+D | row3 col2 | マイ辞書 (my dictionary) |
| Ctrl+L | row5 col2 | 一覧 (list) |
| Ctrl+K | row5 col6 | 国語 (Japanese dictionary) |
| Ctrl+J | row6 col6 | 英和 (English-Japanese dictionary) |
| Ctrl+V | row6 col4 | 音声 (voice) |
| Ctrl+S | row2 col4 | hold 記号 (symbols modifier) |
| Ctrl+M | row5 col2 + symbols | マーカー (marker) |
| Ctrl+F | row3 col6 + symbols | 検索 (search) |

Everything on the physical PW-SH6 keyboard is reachable this way:
the matrix (from the linux-brain device tree) contains only letters,
Space/Enter/Backspace/Shift/Minus, the cursor pad, Home, PageUp/Down,
History, マイ辞書, 一覧, 国語, 英和, 音声, 記号 and the two
blue-legend derivatives マーカー/検索.

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

## The FAT volume mount chain (RESOLVED)

After the APBH SEMA/PIO fixes the storage manager still stalled right
after the partition BPB reads.  Root cause (found with the braintrace
plugin + pmemsave analysis): every DMA-read sector lost its first 128
bytes.  The PIO/kick sequence that arms a card command pre-pulls up to
32 FIFO words (128 bytes) into the SSP RX FIFO for the CPU access
path; the DMA data descriptor then has to deliver those words first.
Without the FIFO drain the descriptor buffer started with a zeroed
head, so the volume BPBs the exfat FSD validated were corrupt and the
mount chain died silently after the probe reads.

With the drain in place (ssp.c) the whole stack delivers real sectors
end to end, verified in the block-layer traces:

* the FAT32 volume BPB (EB FE 90 / MSWIN4.1, totsec 0xac1740) reads back
  byte-exact through the edna2/sdbus2/sdhc scatter-read path
* the FSD walks the FAT area (reads at partition start + 0x20 with a
  0x200-sector stride) and enumerates the root directory (LFN dir
  entries for the Sharp content files visible in the SG buffers)
* the FSD reads file data deep into the 5.5GB user volume (LBAs around
  0xb37xxx), i.e. the volume is mounted and files are opened

The full 6.6GB image (all five split release assets re-assembled with
qemu-img resize to the real 8GiB card size) is required for the
launcher: with a truncated image the deep file-data reads return
zeros.  With the full image the Sharp boot UI starts drawing (the
top strip of the launcher splash appears on the LCDIF console).

Emulation speed is the remaining practical issue: the WinCE storage
stack completes requests at roughly 10-30/s under TCG (the guest
driver waits tens of ms per request; only 3 SSP IRQ 96 pulses per
boot - most completions go through driver timeout paths).  First boot
to the dictionary menu therefore takes a long wall time.  Improving
IRQ fidelity should recover most of it.

Status update (2026-11, after the GPIO0_5 strap + RTC fixes): with
the full image the kernel mounts every eMMC DSK region
(DSK2/DSK3/DSK5..DSK9), the SDMEMORY idle thread runs, and the
system cycles complete suspend/resume power management indefinitely
(`CheckResumeInfo` checksums valid, every PDD PowerDown/PowerUp
handler including `SHC_PowerDown(1)` for the microSD slot).  The
LCDIF console shows the boot logo / launcher splash strip (top-left
region) and the display driver keeps scanning out the framebuffer;
reaching the full launcher UI needs more wall time under TCG (a
10-minute run reaches the suspend-cycling steady state).

Measured interrupt activity during a full-image boot (icoll trace):
TIMROT (48) x449, APBH DMA channel 0 (82) x338, LCDIF (38) x14,
SSP0/SSP1 error (96/97) x4/x5 - the APBH completion IRQs the storage
stack waits on are flowing.

## microSD slot (RESOLVED)

Slot 1 (SSP1) enumerates end to end once the board strap is modelled:

    WARNING: SDMMC_Init: g_bSDHCExist[0]=1 g_bSDHCExist[1]=1
    BSPSDHCIsCardPresent(): The SD card1 is present.
    [FST]HandleAddDevice: ... CSDDevice::SDLoadDevice() Device_SD_Memory (2)

Root cause (the old "EBOOT gate word" analysis, corrected): the EBOOT
"ED-PARAM" query (0x400800b0, arg 5) does not read a DRAM config struct
at all.  EBOOT maps the PINCTRL block as a 1 MB section at VA
0xb0000000 (L1 entry at PA 0x400b0000 + 0x2c00: 0x80000402), and
*(0x8003d4bc) is simply PINCTRL's VA 0xb0018000 - the query reads
HW_PINCTRL_DIN0 (VA 0xb0018900) and returns bit 5, i.e. the **GPIO0_5
input level**.  That pin is the SH6 board strap for "microSD slot
populated":

* early EBOOT init (right after the `Not Resume !!!` print) latches
  the strap into the flag at VA 0x8003d5a8 (`strne r3, [r5]` after
  `bl 0x400800b0(5)` at 0x4006dbc4);
* `0x4006fe94(1)` (called first by the slot probe 0x40071854) gates
  slot 1 on that flag;
* slot 0's call unconditionally writes the work word at VA 0x8003e980
  (the earlier red herring) and returns 1.

In the emulator GPIO0_5 read 0, so EBOOT skipped slot 1 silently and
the kernel's BSPSDHCIsCardPresent() reported "NO SD card1" from the
EBOOT-args table.

Two changes fix it (brain.c / pinctrl.c):

1. the machine drives GPIO0_5 high as a board strap
   (`mxs_pinctrl_set_input(pinctrl, 0, 1 << 5, 1 << 5)`);
2. PINCTRL reset no longer clears `ext_input[]` - external pin levels
   (straps, card-detect, keyboard rows) survive chip resets like they
   do on real hardware, so the strap is still valid after the initial
   machine reset and through the suspend/resume power-cycle.

The bus wiring was already complete: SSP1 carries the sd-bus with the
index=1 drive, card-detect is modelled on LRADC channel 5 (a present
card reads low) and GPIO0_0, and the enumerating driver prints
`SDHCSlotOptionHandler - SDHCDSetSlotInterfaceEx : Clock Setting:
52000000` as it switches the slot to 8-bit high-speed timing.

## Display (LCDIF) - portrait framebuffer, always rotate CW (2026-11-12)

The guest framebuffer at 0x40100000 is a **portrait (480x854)**
image, not landscape.  Evidence (2026-11-12, QMP `pmemsave` + boot
trace): the framebuffer renders upright text and a left-hand menu
bar when rotated 90 deg CW; the guest programs MAC=0xd0 (MX=1 MY=1
**MV=0**), CASET 0..479, PASET 0..853 and pushes frames through the
LCDIF master path (`RUN master H=<rows> V=480`, chunks of 800 then
54 rows; no APBH DMA is used for these pushes).  The earlier code
gated the 90-degree rotation on the MAC MV bit, so with MV=0 the
console showed the portrait image identity-mapped: cropped and
diagonally sheared (the "top strip only" / ~45 deg shear symptom,
EBOOT splash and desktop alike).  The MV bit only changes how the
incoming pixel stream is addressed into GRAM (`panel_advance`); it
does not change the physical mounting, so `lcdif_gfx_update()` now
always rotates the portrait GRAM 90 deg CW into the landscape
console (`nx = y`, `ny = 854-1-x`).  Verified: the EBOOT "Brain"
splash renders upright and readable; the desktop is re-verified with
each boot.

The first transpose bug (2026-11-12, same function) is fixed the
same way: the panel-Y coordinate spans the native *height* (854),
not the width; using the width mapped every display column x >= 480
to GRAM row 0 (only 230400 of 409920 pixels visible, right ~44%
repeating one row).  The mapping is now a bijection over the whole
frame (enumerated: 409920 distinct pixels).

Open question, not changed yet: a frame pushed through APBH DMA
channel 13 (`mxs_lcdif_dma_data`) never raises the LCDIF
CUR_FRAME_DONE/VSYNC IRQ or swaps CUR/NEXT_BUF - only the CPU/master
paths call `lcdif_transfer_done()`.  The WinCE display driver
completes frames on the APBH channel IRQ (86) today, so this may be
fine as-is; revisit with a trace once the eMMC boot test runs.

Code audit 2026-11-12 (no change, by design): the DMA hook in
`apbh-dma.c` (ch 13, `mxs_lcdif_dma_data`) pushes pixels straight
into the panel GRAM and invalidates the console; descriptor chains
with CCW_IRQ set raise the per-channel APBH IRQ (ch 13 = IRQ 86) via
`mxs_dma_irq_update()`, which is the completion signal the driver
waits on.  Adding `lcdif_transfer_done()` to the DMA hook would clear
RUN and raise LCDIF IRQs the driver does not expect while a chain is
still running, and CUR/NEXT_BUF are not read during DMA pushes, so
the gap is benign for this driver.  Re-check at boot with a trace
counting LCDIF IRQ assertions vs APBH IRQ 86 before changing it.

### I2C master address form (2026-11-12, fix in `i2c.c`, unpushed)

`i2c_run()` passed the 8-bit address byte straight to QEMU's I2C
core, which matches slaves by 7-bit address - every in-tree master
passes `addr >> 1` (cf. `hw/i2c/imx_i2c.c`,
`extract32(s->i2dr_write, 1, 7)`).  Write path now passes
`buf[0] >> 1`; read path now takes the address from the first DATA
byte (`(s->data[0] >> 25) & 0x7f`) instead of CTRL1 - on the i.MX28
the address byte is the first byte of the data stream (Linux
`i2c-mxs.c` puts `i2c_8bit_addr_from_msg()` first into the queue),
while CTRL1.SLAVE_ADDRESS_BYTE is the address this block responds
to in slave mode.  Invisible so far (no slave attached, every
transfer NAKs and the guest handles that), but it unblocks attaching
an SX8650 on I2C1 and the RTC/battery sub-MCU on I2C0.

## Other known gaps

* `touchkey_point_calibration() FAILED!` (touchkey.cpp:223).  The full
  dependency chain has been mapped (2026-11 disassembly + runtime
  watchpoints, see "Touch bring-up analysis" below); the blocker is a
  guest-side driver-ordering race around the VMCopy/PMI1 stack, not a
  simple missing device.  The panel itself is a SEMTECH SX8650 I2C
  touch controller on I2C1 (per `SRC\EDNA2\MainTouch\SX8650Touch.cpp`);
  no I2C bus transactions are issued at all before the failure, so an
  SX8650 I2C device model still needs to be added once the software
  stack gets past the calibration gate.

### Touch bring-up analysis (2026-11)

Reverse-engineered call chain (module VAs from a pmemsave dump +
page-table walk; scattered WinCE6 DLL pages make VA != PA):

    touchkey.dll!touchkey_point_calibration (VA 0xC07C5288 in the dump)
      bl 0xC07C2548 -> 0xC07C23AC wrapper -> "get calibration data"
        CreateFileW("VMC1:")                      <- FAILS
        DeviceIoControl(0x80002093, 16-byte struct)
        requires shorts [0],[2],[6],[8] != 0, then magic [0xc]/[0xe]
      -> prints "ERROR: touchkey.cpp line 223: ..."

  VMC1: is provided by VMCopy.dll (registry
  `Drivers\BuiltIn\VMCopy`, Prefix VMC); its sub2.c:587
  `CreateFileW("PMI1:", GENERIC_READ|GENERIC_WRITE, 3, ...)`
  fails.

  PMI1: is provided by pmupdk.dll (registry `Drivers\BuiltIn\PMI`,
  Prefix PMI); it IS registered eventually (present in the
  Drivers\Active hive), but the VMCopy sub2 init runs from
  pmupdk's `PowerInitHandle` worker thread which executes BEFORE
  `PMI_Init` returns - i.e. before device.exe registers PMI1:.

Observed in the UART log:

    PowerInitHandle() ID = 01990006
    PowerInitHandle()++
    ERROR: ...VMCopy\VMCopy\.\sub2.c line 587: Failed in createFile() for PMI1:
    PowerInitHandle()--
    PowerInit()--            <- PMI_Init still running when the thread ran

Under TCG the guest completes far less work per scheduler tick than
the real 454 MHz part, so the PowerInitHandle thread gets its quantum
inside PMI_Init and the CreateFile loses the race deterministically
(also with `-icount shift=1`).  Next steps for touch:

  - reproduce the real-hardware boot order (compare against a serial
    log captured on a real PW-AJ2/SH6 - the Brain Hackers community
    has the hardware), or
  - model the SX8650 on I2C1 and check whether the WinCE touch MDD
    (`SX8650Touch.cpp`, `TPDriver`) can be driven directly, or
  - inspect the pmupdk PowerInitHandle thread's creation priority and
    delay requirements (pmupdk strings live at PA 0x40a80000+ in the
    dump; module delta 0x7FB82000).

Other facts established along the way: the WinCE image loads only the
I2C1 bus driver (registry has `Drivers\BuiltIn\I2C1`, no I2C0 entry;
`I2C_Init(1) : ClockRate=400kbps`), the audio codec is a BU26154
(`wavedev2_BU26154.dll`), and a `touchraw.dll` (Prefix TRD) exists as
a raw-touch helper driver.

* Storage throughput under TCG is low (most request completions go
  through driver timeout paths; only a handful of SSP IRQ 96 pulses
  per boot).  Improving IRQ fidelity should speed the WinCE storage
  stack up considerably.
* EBOOT's `LoadBootCFG / ResetDefaultBootCFG` flash errors (no
  GPMI/NAND model) - EBOOT falls back to compiled-in defaults that
  match the dump.

## References

* eMMC dump: samusun343-ai/wince-imgs, release tag `emmc`
* brain-hackers/linux-brain device trees (`imx28-brain.dtsi`,
  `imx28-pwsh6.dts` ...)
* brain-hackers/u-boot-brain (register headers under
  `arch/arm/include/asm/arch-mxs/`)
* Freescale elftosb (SB image format, `common/EncoreBootImage.*`,
  `common/IVTDataSource.*`)
* u-boot `tools/mxsboot.c` (boot config block layout, 0x53 partition)
