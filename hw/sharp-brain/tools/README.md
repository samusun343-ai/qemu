# sharp-brain debugging tools

Investigation tooling used to reverse-engineer the WinCE boot/storage stack
on the PW-AJ2 eMMC dump. Kept here because they are part of the bring-up
workflow.

## braintrace.c - QEMU TCG plugin

Build:  gcc -O2 -fPIC -shared -I../../include/plugins $(pkg-config --cflags glib-2.0) -o libbraintrace.so braintrace.c

Trace TB execution in up to four VA ranges plus full register/memory dumps
at selected TB entry addresses, without stopping the VM (unlike the gdbstub,
which perturbs guest timing through timer jumps on attach):

    qemu-system-arm -M pw-aj2 ... -plugin file=libbraintrace.so,\
        out=trace.log,r0lo=0xC0330000,r0hi=0xC0360000,\
        regvas=0xC07E1634,regvas=0xC07E16F0,dumpmem=1

Args: out=<file>; rNlo/rNhi (N=0..3) address ranges logged as "E<N> <pc>";
regvas=<va> (repeatable) TB entry VAs whose full register state (and, with
dumpmem=1, stack/buffer memory and the SG data buffer at [r5+0x14]) gets
dumped as "R <pc> reg=val ...".

## rsp.py - gdbstub RSP client

Minimal GDB remote-serial-protocol client (attach, interrupt, continue,
step, read/write memory, breakpoints/watchpoints, registers). Note that
stopping the VM shifts guest timers, so module load addresses can differ
between gdbstub runs; prefer the plugin for reproducible traces.

## regparse.py / armre.py

Parsers for the WinCE ROM registry (reginit cell records) found in the OS
image partition and for post-mortem analysis of QEMU pmemsave RAM dumps
(string search, literal-reference search, capstone disassembly).

See the top-level README.md "Known issue" section for the current
mount-blocker investigation state.
