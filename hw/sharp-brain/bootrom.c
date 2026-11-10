/*
 * i.MX28 boot ROM emulation ("virtual ROM").
 *
 * Copyright (c) 2025 samusun343-ai
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 *
 * This reproduces what the real i.MX28 ROM does when booting from
 * eMMC/SD, without shipping any ROM code:
 *
 *  1. read the MBR and search for the "SigmaTel" boot partition
 *     (type 0x53)
 *  2. parse the 4-sector boot config block (magic 0x00112233) at the
 *     start of that partition and pick the primary boot slot
 *  3. read the SB (Secure Boot) stream from the slot
 *  4. unwrap the data encryption key with the all-zero KEK and
 *     AES-128-CBC decrypt the command sections
 *  5. execute the boot commands: LOAD copies payload into memory,
 *     CALL (HAB) interprets the IVT at the given address and jumps
 *     to the entry point
 *
 * The SB image layout was verified against the actual PW-AJ2 eMMC
 * dump (release "sharp-brain-pw-aj2-emmc") and matches u-boot's
 * tools/mxsimage.c generator:
 *
 *   header (cleartext, header_blocks*16 bytes):
 *     +0x00 SHA-1 digest (first 16 bytes double as the CBC IV)
 *     +0x14 'STMP' cookie
 *     +0x18 major/minor version (bytes), flags (u16)
 *     +0x1c image size in 16-byte blocks
 *     +0x20 first boot TAG block (in blocks)
 *     +0x24 first bootable section id
 *     +0x28 key count / key dictionary block (u16 each)
 *     +0x2c header size in blocks
 *     +0x2e section count / section header size (u16 each)
 *     +0x34 'sgtl' cookie (SB v1.1)
 *   section table at header end: 16 bytes per section
 *     { number, offset, length, flags } (offset/length in blocks)
 *   key dictionary at key_dict_block*16:
 *     { CBC-MAC[16], DEK encrypted with the zero KEK (IV=digest[0:16]) }
 *   first boot TAG block (16 bytes): encrypted with DEK, IV=digest[0:16]
 *   section command stream: encrypted with DEK, IV=digest[0:16], one
 *     continuous CBC chain; LOAD payload data is part of the chain and
 *     follows the 16-byte command, padded to a 16-byte multiple
 *
 * When the called stage returns, control comes back to a tiny
 * trampoline in IRAM that pokes a mailbox MMIO register; the mailbox
 * requests a guest reset with a saved "phase" so interpretation
 * resumes at the next boot command (guest RAM contents survive the
 * reset, matching hardware behaviour).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "crypto/cipher.h"
#include "system/runstate.h"
#include "system/reset.h"
#include "system/block-backend.h"
#include "system/block-backend-io.h"
#include "system/address-spaces.h"
#include "target/arm/cpu-qom.h"
#include "brain.h"

#define BOOTROM_TRAMPOLINE  0x0001ff00      /* top of IRAM (unused by ROM images) */
#define BOOTROM_MAILBOX     0x8003c000      /* SIMDBG block, unused on Brain */
#define BOOTROM_SP          0x00020000      /* IRAM top */

/* SB boot command tags */
#define SB_CMD_NOP    0x00
#define SB_CMD_TAG    0x01
#define SB_CMD_LOAD   0x02
#define SB_CMD_FILL   0x03
#define SB_CMD_JUMP   0x04
#define SB_CMD_CALL   0x05
#define SB_CMD_MODE   0x06
#define SB_FLAG_HAB   (1 << 0)              /* ROM_HAB_EXEC */
#define SB_FLAG_LAST_TAG (1 << 0)           /* ROM_TAG_CMD_FLAG_ROM_LAST_TAG */

#define SB_MAX_IMAGE  (4 * MiB)

typedef struct BrainBootRom {
    Imx28State *soc;
    BlockBackend *blk;

    uint8_t *cmd;              /* decrypted command stream */
    uint32_t cmd_size;
    uint32_t cmd_pos;          /* resume position (bytes) */
    bool active;               /* SB successfully parsed */
    bool mailbox_pending;
} BrainBootRom;

static BrainBootRom bootrom;

/* ------------------------------------------------------------------ */
/* helpers                                                            */

static bool bootrom_pread(uint64_t off, uint32_t len, void *buf)
{
    /* blk_pread() returns 0 on success, negative errno on failure */
    return blk_pread(bootrom.blk, off, len, buf, 0) >= 0;
}

static bool sb_cbc_decrypt(const uint8_t *key, const uint8_t *iv,
                           const uint8_t *in, uint8_t *out, size_t len)
{
    g_autoptr(QCryptoCipher) cipher = NULL;
    Error *err = NULL;

    cipher = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                                QCRYPTO_CIPHER_MODE_CBC,
                                key, 16, &err);
    if (!cipher) {
        error_reportf_err(err, "bootrom: cipher: ");
        return false;
    }
    if (qcrypto_cipher_setiv(cipher, iv, 16, &err) != 0) {
        error_reportf_err(err, "bootrom: setiv: ");
        return false;
    }
    if (qcrypto_cipher_decrypt(cipher, in, out, len, &err) != 0) {
        error_reportf_err(err, "bootrom: decrypt: ");
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* SB image parsing                                                   */

/*
 * SB v1.x image header (96 bytes for v1.1), matching u-boot's
 * struct sb_boot_image_header (tools/mxsimage.h).
 */
typedef struct QEMU_PACKED SBHeader {
    uint8_t  digest[20];             /* 0x00: SHA-1, [0:16] doubles as CBC IV */
    uint8_t  signature[4];           /* 0x14: 'STMP' */
    uint8_t  major_version;          /* 0x18 */
    uint8_t  minor_version;          /* 0x19 */
    uint16_t flags;                  /* 0x1a */
    uint32_t image_blocks;           /* 0x1c: total size in 16-byte blocks */
    uint32_t first_boot_tag_block;   /* 0x20 */
    uint32_t first_boot_section_id;  /* 0x24 */
    uint16_t key_count;              /* 0x28 */
    uint16_t key_dict_block;         /* 0x2a */
    uint16_t header_blocks;          /* 0x2c */
    uint16_t section_count;          /* 0x2e */
    uint16_t section_header_size;    /* 0x30: section header size in blocks */
    uint8_t  pad0[2];                /* 0x32 */
    uint8_t  signature2[4];          /* 0x34: 'sgtl' (v1.1) */
    uint64_t timestamp_us;           /* 0x38 */
    uint8_t  product_version[12];    /* 0x40 */
    uint8_t  component_version[12];  /* 0x4c */
    uint16_t drive_tag;              /* 0x58 */
    uint8_t  pad1[6];                /* 0x5a */
} SBHeader;

QEMU_BUILD_BUG_ON(sizeof(SBHeader) != 96);

/* one 16-byte section table entry */
typedef struct QEMU_PACKED SBSectionHeader {
    uint32_t number;                 /* section number */
    uint32_t offset;                 /* first command block, in blocks */
    uint32_t length;                 /* section length in blocks */
    uint32_t flags;                  /* bit 0: bootable */
} SBSectionHeader;

QEMU_BUILD_BUG_ON(sizeof(SBSectionHeader) != 16);

/*
 * Locate and load the SB image from the boot media.
 * Returns a malloc'd buffer and its size.
 */
static uint8_t *bootrom_load_sb(uint32_t *out_size)
{
    struct {
        uint8_t boot_code[446];
        uint8_t part[4][16];
        uint8_t sig[2];
    } QEMU_PACKED mbr;
    uint32_t boot_part_lba = 0;
    uint8_t cfg[512 * 4];
    uint32_t slot_lba, slot_sectors;
    uint8_t *img;

    if (!bootrom_pread(0, sizeof(mbr), &mbr)) {
        return NULL;
    }
    if (mbr.sig[0] != 0x55 || mbr.sig[1] != 0xaa) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: no MBR on boot media\n");
        return NULL;
    }
    for (int i = 0; i < 4; i++) {
        if (mbr.part[i][4] == 0x53) {
            boot_part_lba = ldl_le_p(&mbr.part[i][8]);
            break;
        }
    }
    if (!boot_part_lba) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: no type-0x53 boot partition found\n");
        return NULL;
    }

    if (!bootrom_pread((uint64_t)boot_part_lba * 512, sizeof(cfg), cfg)) {
        return NULL;
    }
    if (le32_to_cpu(*(uint32_t *)cfg) != 0x00112233) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: bad boot config block magic\n");
        return NULL;
    }

    /*
     * Boot config block (Sharp variant, as observed in the PW-AJ2 dump):
     *   +0x00 magic, +0x04/0x08/0x0c: primary tag / secondary tag / copies
     *   +0x18: slot[0] { tag, first_sector, sector_count, pad }
     *   +0x2c: slot[1] { ... }
     */
    slot_lba = le32_to_cpu(*(uint32_t *)&cfg[0x1c]);
    slot_sectors = le32_to_cpu(*(uint32_t *)&cfg[0x20]);
    if (slot_lba == 0 || slot_sectors == 0 || slot_sectors > 65536) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: bad boot slot geometry\n");
        return NULL;
    }

    img = g_malloc(slot_sectors * 512);
    if (!bootrom_pread((uint64_t)slot_lba * 512, slot_sectors * 512, img)) {
        g_free(img);
        return NULL;
    }
    *out_size = slot_sectors * 512;
    return img;
}

/* verify the per-command checksum: (b[0] - sum(b[1:16])) & 0xff == 0x5a */
static bool sb_cmd_checksum_ok(const uint8_t *c)
{
    uint8_t sum = 0;

    for (int i = 1; i < 16; i++) {
        sum += c[i];
    }
    return (uint8_t)(c[0] - sum) == 0x5a;
}

/*
 * Parse the SB image into the decrypted command stream:
 *   cmd = [16B boot TAG command] || [decrypted section stream]
 */
static bool bootrom_parse_sb(void)
{
    uint32_t img_size;
    uint8_t *img = bootrom_load_sb(&img_size);
    SBHeader hdr;
    uint32_t sect_off, tag_off;
    SBSectionHeader boot_sect;
    uint8_t dek[16];
    uint8_t iv[16];
    uint8_t *stream = NULL;
    uint32_t stream_size = 0;
    bool found = false;

    if (!img) {
        return false;
    }

    if (img_size < sizeof(hdr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: SB image too small\n");
        goto fail;
    }
    memcpy(&hdr, img, sizeof(hdr));
    if (memcmp(hdr.signature, "STMP", 4) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: bad SB cookie %c%c%c%c\n",
                      hdr.signature[0], hdr.signature[1],
                      hdr.signature[2], hdr.signature[3]);
        goto fail;
    }
    if (hdr.major_version != 1 || hdr.minor_version < 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: unsupported SB %d.%d\n",
                      hdr.major_version, hdr.minor_version);
        goto fail;
    }

    memcpy(iv, hdr.digest, 16);

    uint32_t hdr_end = hdr.header_blocks * 16;
    if (hdr_end < sizeof(hdr) || hdr_end > img_size ||
        hdr.image_blocks * 16 > img_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: bad header geometry "
                      "(header=%u blocks, image=%u blocks, media=%u)\n",
                      hdr.header_blocks, hdr.image_blocks, img_size / 16);
        goto fail;
    }

    if (hdr.key_count) {
        uint32_t kd = hdr.key_dict_block * 16;
        if (kd + 32 > img_size) {
            qemu_log_mask(LOG_GUEST_ERROR, "bootrom: key dictionary out of "
                          "range\n");
            goto fail;
        }
        /* unwrap the DEK with the all-zero KEK, IV = digest[0:16] */
        if (!sb_cbc_decrypt((uint8_t[16]) { 0 }, iv, &img[kd + 16], dek,
                            16)) {
            goto fail;
        }
    } else {
        /* unencrypted image */
        memset(dek, 0, 16);
    }

    /* locate the bootable section in the section table */
    sect_off = hdr_end;
    tag_off = hdr.first_boot_tag_block * 16;
    if (tag_off + 16 > img_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: boot tag out of range\n");
        goto fail;
    }

    for (uint32_t i = 0; i < hdr.section_count; i++) {
        SBSectionHeader sh;

        if (sect_off + (i + 1) * sizeof(sh) > img_size) {
            goto fail;
        }
        memcpy(&sh, &img[sect_off + i * sizeof(sh)], sizeof(sh));
        if (le32_to_cpu(sh.number) == hdr.first_boot_section_id) {
            boot_sect = sh;
            found = true;
            break;
        }
    }
    if (!found) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: bootable section %u not found\n",
                      hdr.first_boot_section_id);
        goto fail;
    }

    uint32_t data_off = le32_to_cpu(boot_sect.offset) * 16;
    uint32_t data_len = le32_to_cpu(boot_sect.length) * 16;
    if (data_off + data_len > img_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: section out of range "
                      "(off=%u len=%u image=%u)\n",
                      data_off, data_len, img_size);
        goto fail;
    }

    /*
     * Build the command stream:
     *   [boot TAG block] || [section data]
     * The TAG block and the section data are decrypted independently,
     * each starting from IV = digest[0:16]; within the section the CBC
     * chain runs continuously across commands and LOAD payloads.
     */
    stream_size = 16 + data_len;
    stream = g_malloc(stream_size);

    qemu_log_mask(LOG_TRACE,
                  "bootrom: SB image %u blocks, header %u blocks, "
                  "%u sections, boot section %u (off=%u len=%u), "
                  "boot tag at +%u%s\n",
                  hdr.image_blocks, hdr.header_blocks, hdr.section_count,
                  hdr.first_boot_section_id, data_off, data_len, tag_off,
                  hdr.key_count ? ", encrypted" : ", cleartext");

    if (hdr.key_count == 0) {
        /* unencrypted image */
        memcpy(stream, &img[tag_off], 16);
        memcpy(&stream[16], &img[data_off], data_len);
    } else {
        if (!sb_cbc_decrypt(dek, iv, &img[tag_off], stream, 16)) {
            goto fail;
        }
        if (!sb_cbc_decrypt(dek, iv, &img[data_off], &stream[16],
                            data_len)) {
            goto fail;
        }
    }

    if (!sb_cmd_checksum_ok(stream)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: boot TAG command checksum bad "
                      "(%02x %02x %02x %02x...)\n",
                      stream[0], stream[1], stream[2], stream[3]);
        goto fail;
    }

    g_free(bootrom.cmd);
    bootrom.cmd = stream;
    bootrom.cmd_size = stream_size;
    g_free(img);
    return true;

fail:
    g_free(stream);
    g_free(img);
    return false;
}

/* ------------------------------------------------------------------ */
/* command execution                                                  */

static void bootrom_park_cpu(void)
{
    ARMCPU *cpu = &bootrom.soc->cpu;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);
    cpu_set_pc(cs, 0x0001fff8);
    /* a self-loop is at the IRAM top-8; write it */
    uint32_t spin = 0xeafffffe;
    address_space_write(&address_space_memory, 0x0001fff8,
                        MEMTXATTRS_UNSPECIFIED, &spin, 4);
}

static bool bootrom_do_call(uint32_t ivt_addr)
{
    ARMCPU *cpu = &bootrom.soc->cpu;
    CPUState *cs = CPU(cpu);
    uint32_t ivt[8];
    uint32_t entry;
    static const uint32_t trampoline[] = {
        0xe59f0008,             /* ldr  r0, [pc, #8]   -> mailbox */
        0xe3a01001,             /* mov  r1, #1 */
        0xe5801000,             /* str  r1, [r0] */
        0xeafffffe,             /* b    . */
        BOOTROM_MAILBOX,        /* .word 0x8003c000 */
    };

    if (address_space_read(&address_space_memory, ivt_addr,
                           MEMTXATTRS_UNSPECIFIED, ivt, 32) != MEMTX_OK) {
        return false;
    }
    if ((le32_to_cpu(ivt[0]) & 0xff) != 0xd1) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: IVT at 0x%08x has bad tag 0x%08x\n",
                      ivt_addr, le32_to_cpu(ivt[0]));
        return false;
    }
    entry = le32_to_cpu(ivt[1]);

    /* install the return trampoline */
    address_space_write(&address_space_memory, BOOTROM_TRAMPOLINE,
                        MEMTXATTRS_UNSPECIFIED, trampoline,
                        sizeof(trampoline));

    cpu_reset(cs);
    cpu->env.uncached_cpsr = ARM_CPU_MODE_SVC | CPSR_I | CPSR_F;
    cpu->env.regs[0] = 0;
    cpu->env.regs[13] = BOOTROM_SP;
    cpu->env.regs[14] = BOOTROM_TRAMPOLINE;
    cpu->env.thumb = entry & 1;
    cpu_set_pc(cs, entry & ~1u);

    qemu_log_mask(LOG_TRACE,
                  "bootrom: CALL -> entry 0x%08x (%s mode), lr=0x%08x\n",
                  entry, entry & 1 ? "thumb" : "arm", BOOTROM_TRAMPOLINE);
    return true;
}

/* ------------------------------------------------------------------ */
/* EBOOT resume-path fixup (QEMU MMU-toggle semantics)                */

/*
 * The EBOOT resume path ends with this sequence (verified in the
 * PW-AJ2 dump, VA in EBOOT's 0x80000000-biased DRAM window):
 *
 *   ldr   r2, [pc, #0x1ac]     ; r2 = VA of the MMU-off continuation
 *   mov   r3, #0x40000000
 *   sub   r2, r2, r3           ; r2 = PA of the continuation
 *   mrc   p15, 0, r1, c1, c0, 0
 *   bic   r1, ... (x3)         ; clear MMU/I/D-cache bits
 *   mcr   p15, 0, r1, c1, c0, 0    ; MMU OFF
 *   mov   pc, r2               ; jump to the PA (identity execution)
 *
 * and the continuation then restores cp15 state from the saved
 * kernel context blob, re-enables the MMU (jumping to the uncached
 * alias VA 0xc007xxxx, which is valid under the restored kernel
 * page tables) and finally jumps to the kernel resume entry
 * (blob+0x38, a kernel VA).
 *
 * Real hardware executes the `mov pc, r2` right after the MMU-off
 * MCR straight from the pipeline.  QEMU ends the TB at the SCTLR
 * write and re-fetches the next instruction under the new MMU state:
 * the fetch at VA 0x80071ce4 now hits physical 0x80071ce4 - the
 * AUART3 register file - and execution slides off through the
 * peripheral space (the same class of divergence already worked
 * around for the DVFS trampolines with the IRAM mirror).
 *
 * Patch (applied to the EBOOT image when the boot ROM loads it,
 * pattern-matched so an unknown EBOOT build is left untouched):
 *
 *   - `mov r3, #0x40000000` -> `mov r3, #0`     : r2 stays the VA
 *   - the MMU-off `mcr`     -> `nop`            : MMU stays enabled
 *
 * so the jump lands on the same code at its (still-mapped) VA
 * 0x80071cec.  The continuation loads the context-blob pointer from
 * a literal that holds its physical address (0x400e8400) - valid
 * for the MMU-off identity execution the sequence was written for,
 * but unmapped with the MMU on - so the literal is patched to the
 * uncached alias VA (PA + 0x60000000, e.g. 0xa00e8400).
 *
 * The continuation ends with a second, position-dependent VA
 * computation:
 *
 *   add  r2, pc, #0x14        ; r2 = continuation VA
 *   mov  r3, #0x40000000
 *   sub  r2, r2, r3           ; PA-identity "VA minus DRAM base"
 *   mov  r4, #0x80000000
 *   add  r4, r4, r2           ; r4 = cached VA of the final phase
 *   mcr  p15, r1, c1, c0, 0   ; SCTLR := saved value (MMU on)
 *   mov  pc, r4
 *
 * Running at PA identity this computes the cached VA (0x80071e20,
 * where the kernel page tables map DRAM cached); running at the
 * 0x80000000-biased VA it would compute 0xc0071e20, which the
 * kernel tables map somewhere else entirely.  Patch it the same
 * way: `mov r3, #0` and `add r4, r4, r2` -> `mov r4, r2`, keeping
 * the final jump on the mapped cached VA.  Everything after that
 * (TTBR/SCTLR restore from the blob, the final `mov pc, lr` into
 * the kernel resume entry) already works under QEMU's re-fetch
 * semantics because both the EBOOT and kernel page tables map the
 * same 0x80000000-biased DRAM window.
 */
static void bootrom_patch_eboot(uint8_t *img, uint32_t count)
{
    static const uint32_t pattern[] = {
        0xe3a03101,     /* mov r3, #0x40000000      */
        0xe0422003,     /* sub r2, r2, r3           */
        0xee111f10,     /* mrc p15, 0, r1, c1, c0, 0 */
        0xe3c11c23,     /* bic r1, r1, #0x2300      */
        0xe3c11005,     /* bic r1, r1, #5           */
        0xe3c11a01,     /* bic r1, r1, #0x1000      */
        0xee011f10,     /* mcr p15, 0, r1, c1, c0, 0  (MMU off) */
        0xe1a0f002,     /* mov pc, r2               */
    };
    /* continuation: ldr r0,[pc,#0x17c] (blob PA literal) */
    const uint32_t cont_ldr = 0xe59f017c;
    /* final-phase VA computation: mov/add/add/mcr/mov pc */
    static const uint32_t tail_pattern[] = {
        0xe28f2014,     /* add r2, pc, #0x14        */
        0xe3a03101,     /* mov r3, #0x40000000      */
        0xe0422003,     /* sub r2, r2, r3           */
        0xe3a04102,     /* mov r4, #0x80000000      */
        0xe0844002,     /* add r4, r4, r2           */
        0xee011f10,     /* mcr p15, 0, r1, c1, c0, 0 */
        0xe1a0f004,     /* mov pc, r4               */
    };

    if (count < 0x2000) {
        return;
    }
    for (uint32_t off = 0; off + 0x200 <= count; off += 4) {
        uint32_t i;

        for (i = 0; i < ARRAY_SIZE(pattern); i++) {
            if (ldl_le_p(&img[off + i * 4]) != pattern[i]) {
                break;
            }
        }
        if (i != ARRAY_SIZE(pattern)) {
            continue;
        }

        /* find the continuation's blob-pointer load just behind */
        uint32_t ldr_off = 0;
        for (uint32_t o = off + ARRAY_SIZE(pattern) * 4;
             o < off + ARRAY_SIZE(pattern) * 4 + 0x20; o += 4) {
            if (ldl_le_p(&img[o]) == cont_ldr) {
                ldr_off = o;
                break;
            }
        }
        if (!ldr_off) {
            continue;
        }
        uint32_t lit_off = ldr_off + 8 + 0x17c;
        uint32_t blob_pa = ldl_le_p(&img[lit_off]);
        if (blob_pa < IMX28_DRAM_BASE ||
            blob_pa >= IMX28_DRAM_BASE + IMX28_DRAM_SIZE) {
            continue;
        }

        /* keep the jump target a mapped VA and the MMU enabled */
        stl_le_p(&img[off + 0], 0xe3a03000);        /* mov r3, #0 */
        stl_le_p(&img[off + 6 * 4], 0xe1a00000);    /* nop (MMU-off mcr) */
        /* blob PA -> uncached alias VA */
        stl_le_p(&img[lit_off], blob_pa + 0x60000000);

        /* find and fix the position-dependent final-phase jump */
        for (uint32_t o = off + ARRAY_SIZE(pattern) * 4;
             o + ARRAY_SIZE(tail_pattern) * 4 <= off + 0x400;
             o += 4) {
            uint32_t k;

            for (k = 0; k < ARRAY_SIZE(tail_pattern); k++) {
                if (ldl_le_p(&img[o + k * 4]) != tail_pattern[k]) {
                    break;
                }
            }
            if (k != ARRAY_SIZE(tail_pattern)) {
                continue;
            }
            stl_le_p(&img[o + 4], 0xe3a03000);     /* mov r3, #0 */
            stl_le_p(&img[o + 16], 0xe1a04002);    /* mov r4, r2 */
            break;
        }

        qemu_log_mask(LOG_TRACE,
                      "bootrom: EBOOT resume fixup applied at +%#x "
                      "(blob %#x -> VA %#x)\n",
                      off, blob_pa, blob_pa + 0x60000000);
        return;
    }
}

/* execute commands until the next CALL/JUMP, or end of stream */
static bool bootrom_run(void)
{
    uint32_t pos = bootrom.cmd_pos;

    while (pos + 16 <= bootrom.cmd_size) {
        uint8_t *c = &bootrom.cmd[pos];
        uint8_t cmd = c[1];
        uint16_t flags = lduw_le_p(&c[2]);
        uint32_t addr = le32_to_cpu(*(uint32_t *)&c[4]);
        uint32_t count = le32_to_cpu(*(uint32_t *)&c[8]);
        uint32_t data = le32_to_cpu(*(uint32_t *)&c[12]);
        uint32_t next = pos + 16;

        if (!sb_cmd_checksum_ok(c)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bootrom: bad command checksum at +%u "
                          "(tag 0x%02x)\n", pos, cmd);
            return false;
        }

        switch (cmd) {
        case SB_CMD_NOP:
        case SB_CMD_TAG:
        case SB_CMD_MODE:
            qemu_log_mask(LOG_TRACE, "bootrom: +%06u NOP/TAG/MODE "
                          "(tag 0x%02x flags 0x%04x)\n", pos, cmd, flags);
            break;
        case SB_CMD_LOAD:
            if (addr + count > IMX28_IRAM_SIZE &&
                (addr < IMX28_DRAM_BASE ||
                 addr + count > IMX28_DRAM_BASE + IMX28_DRAM_SIZE)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "bootrom: LOAD to invalid address 0x%08x "
                              "(%u bytes)\n", addr, count);
                return false;
            }
            qemu_log_mask(LOG_TRACE, "bootrom: +%06u LOAD 0x%08x, %u bytes\n",
                          pos, addr, count);
            if (addr >= IMX28_DRAM_BASE && count >= 0x2000) {
                bootrom_patch_eboot(&c[16], count);
            }
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, &c[16], count);
            next = (pos + 16 + count + 15) & ~15u;
            break;
        case SB_CMD_FILL:
        {
            qemu_log_mask(LOG_TRACE, "bootrom: +%06u FILL 0x%08x, %u bytes "
                          "with 0x%08x\n", pos, addr, count, data);
            uint32_t words = (count + 3) / 4;
            for (uint32_t i = 0; i < words; i++) {
                address_space_write(&address_space_memory,
                                    addr + i * 4, MEMTXATTRS_UNSPECIFIED,
                                    &data, 4);
            }
            break;
        }
        case SB_CMD_JUMP:
        case SB_CMD_CALL:
            bootrom.cmd_pos = next;    /* resume after the call */
            qemu_log_mask(LOG_TRACE, "bootrom: +%06u %s 0x%08x%s\n", pos,
                          cmd == SB_CMD_CALL ? "CALL" : "JUMP", addr,
                          (flags & SB_FLAG_HAB) ? " (HAB)" : "");
            if (!bootrom_do_call(addr)) {
                return false;
            }
            return true;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bootrom: unknown command 0x%02x at +%u\n",
                          cmd, pos);
            return false;
        }

        pos = next;
    }

    /* stream exhausted */
    return false;
}

/* ------------------------------------------------------------------ */
/* mailbox and reset                                                  */

static void bootrom_mailbox_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    if (!bootrom.active) {
        return;
    }
    bootrom.mailbox_pending = true;
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static uint64_t bootrom_mailbox_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    return 0;
}

static const MemoryRegionOps bootrom_mailbox_ops = {
    .read = bootrom_mailbox_read,
    .write = bootrom_mailbox_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void brain_bootrom_reset(void *opaque)
{
    if (!bootrom.blk) {
        return;
    }

    if (bootrom.mailbox_pending) {
        /* a stage returned to the ROM: resume the command stream */
        bootrom.mailbox_pending = false;
        qemu_log_mask(LOG_TRACE, "bootrom: stage returned, resuming\n");
        if (!bootrom_run()) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bootrom: boot stream complete after stage "
                          "return - parking CPU\n");
            bootrom_park_cpu();
        }
        return;
    }

    /* fresh boot: parse the SB image and start from the beginning */
    if (!bootrom_parse_sb()) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: failed to parse boot stream - parking CPU\n");
        bootrom_park_cpu();
        return;
    }
    bootrom.active = true;
    bootrom.cmd_pos = 0;
    if (!bootrom_run()) {
        bootrom_park_cpu();
    }
}

void brain_bootrom_setup(Imx28State *s, BlockBackend *blk)
{
    static MemoryRegion mailbox;

    bootrom.soc = s;
    bootrom.blk = blk;

    memory_region_init_io(&mailbox, OBJECT(s), &bootrom_mailbox_ops, NULL,
                          "brain-bootrom-mailbox", 4);
    memory_region_add_subregion(get_system_memory(), BOOTROM_MAILBOX,
                                &mailbox);

    qemu_register_reset(brain_bootrom_reset, NULL);
}
