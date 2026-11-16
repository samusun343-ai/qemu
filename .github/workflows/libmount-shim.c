/*
 * Minimal libmount compatibility shim for the static CI build.
 *
 * Ubuntu does not ship a static libmount (libmount-dev provides only
 * the shared object), but a fully static link of QEMU still needs to
 * resolve -lmount: it appears in the private dependencies of the
 * system gio-2.0, whose GUnixMounts code references the functions
 * below.
 *
 * QEMU never uses GUnixMounts (host mount-table enumeration plays no
 * role in emulation), so every entry point here simply reports
 * "unavailable / empty": table constructors return a dummy handle,
 * parsers fail, iterators are empty and the monitor never signals.
 * glib handles all of those as "no mount information", which is the
 * correct degradation for a statically linked emulator binary.
 *
 * This file is compiled against the real <libmount/libmount.h> so
 * the signatures always match the system header, and the resulting
 * archive is placed on the linker search path only for the CI build
 * (see build-linux.yml).  It is not part of QEMU proper.
 */

#include <stdlib.h>

#include <libmount/libmount.h>

const char *mnt_get_fstab_path(void)
{
    return NULL;
}

int mnt_has_regular_mtab(const char **mtab, int *writable)
{
    if (mtab) {
        *mtab = NULL;
    }
    if (writable) {
        *writable = 0;
    }
    return 0;
}

struct libmnt_table *mnt_new_table(void)
{
    return calloc(1, 16);
}

void mnt_free_table(struct libmnt_table *tb)
{
    free(tb);
}

struct libmnt_iter *mnt_new_iter(int direction)
{
    (void)direction;
    return calloc(1, 16);
}

void mnt_free_iter(struct libmnt_iter *itr)
{
    free(itr);
}

int mnt_table_parse_fstab(struct libmnt_table *tb, const char *filename)
{
    (void)tb;
    (void)filename;
    return -1;
}

int mnt_table_parse_mtab(struct libmnt_table *tb, const char *filename)
{
    (void)tb;
    (void)filename;
    return -1;
}

int mnt_table_next_fs(struct libmnt_table *tb, struct libmnt_iter *itr,
                      struct libmnt_fs **fs)
{
    (void)tb;
    (void)itr;
    if (fs) {
        *fs = NULL;
    }
    return 1; /* no more entries */
}

const char *mnt_fs_get_source(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

const char *mnt_fs_get_target(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

const char *mnt_fs_get_fstype(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

const char *mnt_fs_get_options(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

const char *mnt_fs_get_root(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

char *mnt_fs_strdup_options(struct libmnt_fs *fs)
{
    (void)fs;
    return NULL;
}

const struct libmnt_optmap *mnt_get_builtin_optmap(int id)
{
    (void)id;
    return NULL;
}

int mnt_optstr_get_flags(const char *optstr, unsigned long *flags,
                         const struct libmnt_optmap *map)
{
    (void)optstr;
    (void)map;
    if (flags) {
        *flags = 0;
    }
    return -1;
}

struct libmnt_monitor *mnt_new_monitor(void)
{
    return calloc(1, 16);
}

void mnt_unref_monitor(struct libmnt_monitor *mn)
{
    free(mn);
}

int mnt_monitor_enable_kernel(struct libmnt_monitor *mn, int enable)
{
    (void)mn;
    (void)enable;
    return -1;
}

int mnt_monitor_enable_userspace(struct libmnt_monitor *mn, int enable,
                                 const char *filename)
{
    (void)mn;
    (void)enable;
    (void)filename;
    return -1;
}

int mnt_monitor_get_fd(struct libmnt_monitor *mn)
{
    (void)mn;
    return -1;
}

int mnt_monitor_next_change(struct libmnt_monitor *mn, const char **filename,
                            int *type)
{
    (void)mn;
    if (filename) {
        *filename = NULL;
    }
    if (type) {
        *type = 0;
    }
    return -1;
}

int mnt_monitor_event_cleanup(struct libmnt_monitor *mn)
{
    (void)mn;
    return 0;
}
