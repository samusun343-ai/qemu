#!/usr/bin/env python3
"""ARM RE helper for the Brain WinCE RAM dump.

Commands:
  find <text>            - find ASCII string in RAM, print VA
  findw <text>           - find UTF-16LE string in RAM, print VA
  refs <hexva>           - find 32-bit literal-pool references to VA
  dis <hexva> [len]      - disassemble ARM code at VA
  hexptr <hexva>         - dump 16 dwords at VA
RAM: /home/z/my-project/work/ram2.bin loaded at 0x40000000.
"""
import struct
import sys

from capstone import Cs, CS_ARCH_ARM, CS_MODE_ARM, CS_MODE_THUMB

RAM_PATH = "/home/z/my-project/work/ram2.bin"
BASE = 0x40000000
ram = open(RAM_PATH, "rb").read()


def va_to_off(va):
    o = va - BASE
    assert 0 <= o < len(ram), "VA out of range"
    return o


def find(text, wide=False):
    pat = text.encode("utf-16le") if wide else text.encode()
    hits, start = [], 0
    while True:
        i = ram.find(pat, start)
        if i < 0:
            break
        hits.append(BASE + i)
        start = i + 1
    for h in hits[:20]:
        print(hex(h))
    print(f"({len(hits)} hits)")


def refs(va):
    pat = struct.pack("<I", va)
    hits, start = [], 0
    while True:
        i = ram.find(pat, start)
        if i < 0:
            break
        hits.append(BASE + i)
        start = i + 1
    for h in hits[:20]:
        print(hex(h))
    print(f"({len(hits)} hits)")


def dis(va, length=0x80, thumb=False):
    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB if thumb else CS_MODE_ARM)
    o = va_to_off(va)
    for ins in md.disasm(ram[o:o + length], va):
        # annotate literal loads of addresses inside RAM
        extra = ""
        if ins.mnemonic in ("ldr",) and "[pc" in ins.op_str:
            try:
                imm = int(ins.op_str.split("#")[1].rstrip("]"), 0)
                target = ins.address + 8 + imm
                to = target - BASE
                val = struct.unpack_from("<I", ram, to)[0]
                extra = f"  ; @0x{target:x} = 0x{val:08x}"
                if 0x40200000 <= val < 0x41E86000:
                    s = ram[val - BASE:val - BASE + 48].split(b"\0")[0]
                    if all(32 <= c < 127 for c in s) and len(s) > 3:
                        extra += f' "{s.decode()}"'
            except Exception:
                pass
        print(f"0x{ins.address:08x}:  {ins.mnemonic:<8} {ins.op_str}{extra}")


def hexptr(va, n=16):
    o = va_to_off(va)
    for i in range(n):
        v = struct.unpack_from("<I", ram, o + 4 * i)[0]
        tag = ""
        if 0x40000000 <= v < 0x48000000:
            s = ram[v - BASE:v - BASE + 40].split(b"\0")[0]
            if all(32 <= c < 127 for c in s) and len(s) > 3:
                tag = f'  "{s.decode()}"'
        print(f"  [{i:2}] +0x{4*i:03x}: 0x{v:08x}{tag}")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "find":
        find(sys.argv[2])
    elif cmd == "findw":
        find(sys.argv[2], wide=True)
    elif cmd == "refs":
        refs(int(sys.argv[2], 16))
    elif cmd == "dis":
        dis(int(sys.argv[2], 16),
            int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x80,
            thumb=(len(sys.argv) > 4))
    elif cmd == "hexptr":
        hexptr(int(sys.argv[2], 16),
               int(sys.argv[3], 16) if len(sys.argv) > 3 else 16)
    elif cmd == "disasm-raw":
        # disasm-raw <va> <thumb?>
        dis(int(sys.argv[2], 16), 0x100, thumb=(sys.argv[3] == "t"))
    else:
        print(__doc__)
