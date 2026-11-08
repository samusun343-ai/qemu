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
    return blk_pread(bootrom.blk, off, len, buf, 0) == len;
}

/* ------------------------------------------------------------------ */
/* SB image parsing                                                   */

typedef struct SBHeader {
    uint8_t digest[20];
    uint32_t cookie;
    uint16_t major, minor, flags;
    uint32_t image_blocks;
    uint32_t first_boot_tag_block;
    uint32_t first_bootable_section;
    uint16_t key_count, key_dict_block, header_blocks;
    uint16_t section_count, section_header_size;
    uint32_t sig2;
} QEMU_PACKED SBHeader;

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
    if (qcrypto_cipher_decrypt(cipher, in, out, len, &err) != 0) {
        error_reportf_err(err, "bootrom: decrypt: ");
        return false;
    }
    return true;
}

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

/*
 * Parse the SB image into the decrypted command stream.
 * Layout notes (elftosb EncoreBootImage):
 *   - [20B digest][STMP header fields ...] padded to header_blocks*16
 *   - section table follows the header
 *   - DEK dictionary at key_dict_block*16: { mac[16], dek[16] } entries,
 *     dek CBC-encrypted with the zero KEK, IV = digest[0:16]
 *   - each section: [encrypted boot TAG command (1 block)]
 *     [encrypted section data] - both CBC streams with IV = digest[0:16]
 */
static bool bootrom_parse_sb(void)
{
    uint32_t img_size;
    uint8_t *img = bootrom_load_sb(&img_size);
    SBHeader hdr;
    uint32_t hdr_end, st_off;
    uint8_t dek[16];
    uint8_t iv[16];
    uint8_t *stream = NULL;
    uint32_t stream_size = 0;

    if (!img) {
        return false;
    }

    memcpy(&hdr, img, sizeof(hdr));
    if (le32_to_cpu(hdr.cookie) != 0x50444d53 /* 'STMP' LE */) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: bad SB cookie\n");
        goto fail;
    }
    if (hdr.major != 1 || hdr.minor < 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: unsupported SB %d.%d\n",
                      hdr.major, hdr.minor);
        goto fail;
    }

    memcpy(iv, hdr.digest, 16);
    hdr_end = hdr.header_blocks * 16;
    if (hdr_end == 0 || hdr_end > img_size) {
        qemu_log_mask(LOG_GUEST_ERROR, "bootrom: bad header size\n");
        goto fail;
    }

    if (hdr.key_count) {
        uint32_t kd = hdr.key_dict_block * 16;
        if (kd + 32 > img_size) {
            goto fail;
        }
        /* unwrap the DEK with the all-zero KEK */
        if (!sb_cbc_decrypt((uint8_t[16]) { 0 }, iv, &img[kd + 16], dek,
                            16)) {
            goto fail;
        }
    } else {
        /* unencrypted image */
        memset(dek, 0, 16);
    }

    /* section table */
    st_off = hdr_end;
    uint32_t boot_tag = hdr.first_boot_tag_block * 16;
    if (boot_tag + 16 > img_size) {
        goto fail;
    }

    /* build the command stream: [boot TAG block][boot section data] */
    for (uint32_t i = 0; i < hdr.section_count; i++) {
        uint32_t tag, off, len, flags;

        if (st_off + (i + 1) * 16 > img_size) {
            goto fail;
        }
        tag = le32_to_cpu(*(uint32_t *)&img[st_off + i * 16]);
        off = le32_to_cpu(*(uint32_t *)&img[st_off + i * 16 + 4]);
        len = le32_to_cpu(*(uint32_t *)&img[st_off + i * 16 + 8]);
        flags = le32_to_cpu(*(uint32_t *)&img[st_off + i * 16 + 12]);

        if (tag != hdr.first_bootable_section) {
            continue;
        }
        if (off * 16 + len * 16 > img_size) {
            goto fail;
        }

        /* boot tag command block + section data, decrypted separately */
        stream_size = 16 + len * 16;
        stream = g_malloc(stream_size);
        if (flags & 2) {          /* ROM_SECTION_CLEARTEXT */
            memcpy(stream, &img[boot_tag], 16);
            memcpy(&stream[16], &img[off * 16], len * 16);
        } else {
            if (!sb_cbc_decrypt(dek, iv, &img[boot_tag], stream, 16)) {
                goto fail;
            }
            if (!sb_cbc_decrypt(dek, iv, &img[off * 16], &stream[16],
                                len * 16)) {
                goto fail;
            }
        }
        break;
    }

    if (!stream) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: bootable section not found\n");
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
                           MEMTXATTRS_UNSPECIFIED, ivt, 32) != 32) {
        return false;
    }
    if ((ivt[0] & 0xff) != 0xd1) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bootrom: IVT at 0x%08x has bad tag 0x%08x\n",
                      ivt_addr, ivt[0]);
        return false;
    }
    entry = le32_to_cpu(ivt[1]);

    /* install the return trampoline */
    address_space_write(&address_space_memory, BOOTROM_TRAMPOLINE,
                        MEMTXATTRS_UNSPECIFIED, trampoline,
                        sizeof(trampoline));

    cpu_reset(cs);
    CPU_FOREACH(cs) {
        /* single CPU machine; nothing else to prepare */
    }
    cs = CPU(cpu);
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

/* execute commands until the next CALL/JUMP, or end of stream */
static bool bootrom_run(void)
{
    uint32_t pos = bootrom.cmd_pos;

    while (pos + 16 <= bootrom.cmd_size) {
        uint8_t *c = &bootrom.cmd[pos];
        uint8_t cmd = c[1];
        uint16_t flags = le16_to_cpu(*(uint16_t *)&c[2]);
        uint32_t addr = le32_to_cpu(*(uint32_t *)&c[4]);
        uint32_t count = le32_to_cpu(*(uint32_t *)&c[8]);
        uint32_t data = le32_to_cpu(*(uint32_t *)&c[12]);
        uint32_t next = pos + 16;

        switch (cmd) {
        case SB_CMD_NOP:
        case SB_CMD_TAG:
            break;
        case SB_CMD_LOAD:
            if (addr + count > IMX28_IRAM_SIZE &&
                (addr < IMX28_DRAM_BASE ||
                 addr + count > IMX28_DRAM_BASE + IMX28_DRAM_SIZE)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "bootrom: LOAD to invalid address 0x%08x\n",
                              addr);
                return false;
            }
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, &c[16], count);
            next = (pos + 16 + count + 15) & ~15u;
            break;
        case SB_CMD_FILL:
        {
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
            if (flags & SB_FLAG_HAB) {
                if (!bootrom_do_call(addr)) {
                    return false;
                }
            } else {
                /* plain JUMP/CALL: use the address directly */
                if (!bootrom_do_call(addr)) {
                    return false;
                }
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
