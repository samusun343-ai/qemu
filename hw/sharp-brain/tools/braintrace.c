/*
 * braintrace QEMU plugin: TB-execution trace for VA ranges, with
 * register capture at specific TB entry addresses.
 *
 * Args:  out=<path>              output log
 *        rNlo=0x.. rNhi=0x..     TB vaddr ranges (N=0..3), events "E<N> <pc>"
 *        regvas=0x..,0x..        TB entry VAs whose registers get dumped
 *        lo=0x.. hi=0x..         masked (va & 0x1fffffff) range, tag E9
 *        mlo/mhi                 MMIO store watch (masked), events "M <va>"
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static FILE *out;
static uint64_t lo = 0, hi = 0;
static uint64_t mlo = 0, mhi = 0;
static uint64_t rlo[4] = {0, 0, 0, 0};
static uint64_t rhi[4] = {0, 0, 0, 0};
static int nranges = 0;
static uint64_t regvas[24];
static int nregvas = 0;
static int dumpmem = 0;
static uint64_t nevents;
static uint64_t maxevents = 40000000;
static GMutex lock;

static uint64_t mask(uint64_t va)
{
    return va & 0x1fffffffULL;
}

static void vcpu_tb_exec(unsigned int cpu_index, void *udata)
{
    uint64_t ud = (uint64_t)(uintptr_t)udata;
    int tag = (int)(ud >> 32);
    uint64_t pc = ud & 0xffffffffULL;
    if (nevents < maxevents) {
        g_mutex_lock(&lock);
        fprintf(out, "E%d %08" PRIx64 "\n", tag, pc);
        nevents++;
        g_mutex_unlock(&lock);
    }
}

static uint64_t reg_of(GArray *regs, const char *name)
{
    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        if (g_strcmp0(d->name, name) == 0) {
            GByteArray *ba = g_byte_array_new();
            uint64_t v = 0;
            if (qemu_plugin_read_register(d->handle, ba)) {
                memcpy(&v, ba->data, ba->len < 8 ? ba->len : 8);
            }
            g_byte_array_unref(ba);
            return v;
        }
    }
    return 0;
}

static void dump_mem(GString *s, const char *label, uint64_t addr, size_t len)
{
    GByteArray *ba = g_byte_array_sized_new(len);
    if (qemu_plugin_read_memory_vaddr(addr, ba, len)) {
        g_string_append_printf(s, "%s[%llx]=", label,
                               (unsigned long long)addr);
        for (guint i = 0; i < ba->len; i++) {
            g_string_append_printf(s, "%02x", ba->data[i]);
        }
        g_string_append_c(s, ' ');
    } else {
        g_string_append_printf(s, "%s[%llx]=UNREADABLE ", label,
                               (unsigned long long)addr);
    }
    g_byte_array_unref(ba);
}

static void vcpu_tb_exec_regs(unsigned int cpu_index, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;
    GString *s = g_string_new("");
    g_autoptr(GArray) regs = qemu_plugin_get_registers();
    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        GByteArray *ba = g_byte_array_new();
        if (qemu_plugin_read_register(d->handle, ba)) {
            uint64_t v = 0;
            memcpy(&v, ba->data, ba->len < 8 ? ba->len : 8);
            g_string_append_printf(s, "%s=%llx ", d->name,
                                   (unsigned long long)v);
        }
        g_byte_array_unref(ba);
    }
    if (dumpmem) {
        uint64_t sp = reg_of(regs, "sp");
        uint64_t r2 = reg_of(regs, "r2");
        uint64_t r5 = reg_of(regs, "r5");
        uint64_t r8 = reg_of(regs, "r8");
        dump_mem(s, "stk", sp, 0x40);
        if (r2) { dump_mem(s, "r2buf", r2, 0x40); }
        if (r5) {
            dump_mem(s, "r5buf", r5, 0x40);
            /* SG request: [r5+0x14] = data buffer pointer */
            GByteArray *sg = g_byte_array_sized_new(4);
            if (qemu_plugin_read_memory_vaddr(r5 + 0x14, sg, 4) && sg->len == 4) {
                uint64_t pbuf = 0;
                memcpy(&pbuf, sg->data, 4);
                if (pbuf > 0x10000) {
                    dump_mem(s, "sgbuf", pbuf, 0x40);
                }
            }
            g_byte_array_unref(sg);
        }
        if (r8) { dump_mem(s, "r8buf", r8, 0x40); }
    }
    g_mutex_lock(&lock);
    fprintf(out, "R %08" PRIx64 " %s\n", pc, s->str);
    nevents++;
    g_mutex_unlock(&lock);
    g_string_free(s, TRUE);
}

static void vcpu_tb_trans(struct qemu_plugin_tb *tb, void *udata)
{
    uint64_t pc = qemu_plugin_tb_vaddr(tb);
    for (int i = 0; i < nregvas; i++) {
        if (pc == regvas[i]) {
            qemu_plugin_register_vcpu_tb_exec_cb(
                tb, vcpu_tb_exec_regs, QEMU_PLUGIN_CB_R_REGS,
                (void *)(uintptr_t)pc);
        }
    }
    for (int i = 0; i < nranges; i++) {
        if (pc >= rlo[i] && pc < rhi[i]) {
            qemu_plugin_register_vcpu_tb_exec_cb(
                tb, vcpu_tb_exec, QEMU_PLUGIN_CB_NO_REGS,
                (void *)(uintptr_t)(pc | ((uint64_t)i << 32)));
            break;
        }
    }
    if (hi > lo && mask(pc) >= lo && mask(pc) < hi) {
        qemu_plugin_register_vcpu_tb_exec_cb(
            tb, vcpu_tb_exec, QEMU_PLUGIN_CB_NO_REGS,
            (void *)(uintptr_t)(pc | ((uint64_t)9 << 32)));
    }
}

static void vcpu_mem(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                     uint64_t vaddr, void *udata)
{
    if (!qemu_plugin_mem_is_store(info)) {
        return;
    }
    uint64_t m = mask(vaddr);
    if (m >= mlo && m < mhi && nevents < maxevents) {
        g_mutex_lock(&lock);
        fprintf(out, "M %08" PRIx64 " cpu=%u\n", m, cpu_index);
        nevents++;
        g_mutex_unlock(&lock);
    }
}

static void plugin_exit(void *p)
{
    if (out) {
        fprintf(out, "# events %" PRIu64 "\n", nevents);
        fclose(out);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) tokens = g_strsplit(argv[i], "=", 2);
        if (!tokens[1]) {
            fprintf(stderr, "braintrace: bad arg %s\n", argv[i]);
            return -1;
        }
        if (g_strcmp0(tokens[0], "out") == 0) {
            out = fopen(tokens[1], "w");
            if (!out) {
                perror("braintrace: open");
                return -1;
            }
        } else if (g_strcmp0(tokens[0], "lo") == 0) {
            lo = strtoull(tokens[1], NULL, 0);
        } else if (g_strcmp0(tokens[0], "hi") == 0) {
            hi = strtoull(tokens[1], NULL, 0);
        } else if (g_strcmp0(tokens[0], "mlo") == 0) {
            mlo = strtoull(tokens[1], NULL, 0);
        } else if (g_strcmp0(tokens[0], "mhi") == 0) {
            mhi = strtoull(tokens[1], NULL, 0);
        } else if (g_strcmp0(tokens[0], "dumpmem") == 0) {
            dumpmem = atoi(tokens[1]);
        } else if (g_strcmp0(tokens[0], "regvas") == 0) {
            g_auto(GStrv) items = g_strsplit(tokens[1], ",", 24);
            for (gint k = 0; items[k]; k++) {
                if (nregvas < 24) {
                    regvas[nregvas++] = strtoull(items[k], NULL, 0);
                }
            }
        } else if (g_str_has_prefix(tokens[0], "r") &&
                   g_str_has_suffix(tokens[0], "lo")) {
            int idx = tokens[0][1] - '0';
            if (idx >= 0 && idx < 4) {
                rlo[idx] = strtoull(tokens[1], NULL, 0);
                if (idx >= nranges) { nranges = idx + 1; }
            }
        } else if (g_str_has_prefix(tokens[0], "r") &&
                   g_str_has_suffix(tokens[0], "hi")) {
            int idx = tokens[0][1] - '0';
            if (idx >= 0 && idx < 4) {
                rhi[idx] = strtoull(tokens[1], NULL, 0);
                if (idx >= nranges) { nranges = idx + 1; }
            }
        } else if (g_strcmp0(tokens[0], "maxevents") == 0) {
            maxevents = strtoull(tokens[1], NULL, 0);
        } else {
            fprintf(stderr, "braintrace: unknown arg %s\n", argv[i]);
            return -1;
        }
    }
    if (!out) {
        fprintf(stderr, "braintrace: need out=\n");
        return -1;
    }
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
