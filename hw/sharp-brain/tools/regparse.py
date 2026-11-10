#!/usr/bin/env python3
"""Parse the WinCE ROM registry (reginit cell) blob from the OS image.

Record: [size:u16][marker:u16][...][size counts bytes after the first 4]
  key:   01 00 | 02 00 | namelen(wchars incl NUL):u16 | name
  value: 02 00 | valtype:u16 | namelen(wchars incl NUL):u16 | datalen(bytes):u16 | name | data
Walks forward from a start offset, printing the tree.
"""
import struct
import sys

img = open("/home/z/my-project/work/os-image.bin", "rb").read()

REG_SZ = 1
REG_DWORD = 4
REG_BINARY = 3
REG_MULTI_SZ = 7


def parse(start, end):
    off = start
    out = []
    while off + 4 <= end:
        size, marker = struct.unpack_from("<HH", img, off)
        if size < 4 or off + size + 4 > end:
            break
        total = size + 4
        if marker == 0x0001:  # key
            flags, namelen = struct.unpack_from("<HH", img, off + 4)
            name = img[off + 8:off + 8 + namelen * 2].decode("utf-16le").rstrip("\0")
            out.append(("K", name))
        elif marker == 0x0002:  # value
            valtype, namelen, datalen = struct.unpack_from("<HHH", img, off + 4)
            name = img[off + 10:off + 10 + namelen * 2].decode("utf-16le").rstrip("\0")
            data = img[off + 10 + namelen * 2:off + 10 + namelen * 2 + datalen]
            if valtype == REG_DWORD:
                val = struct.unpack("<I", data)[0] if len(data) == 4 else data.hex()
            elif valtype in (REG_SZ, REG_MULTI_SZ):
                val = data.decode("utf-16le", errors="replace").rstrip("\0")
            else:
                val = data.hex()
            out.append(("V", name, val, valtype))
        else:
            break
        off += total
    return out, off


def main():
    start = int(sys.argv[1], 16)
    end = int(sys.argv[2], 16) if len(sys.argv) > 2 else len(img)
    recs, stop = parse(start, end)
    for r in recs:
        if r[0] == "K":
            print(f"[KEY] {r[1]}")
        else:
            print(f"    {r[1]} = {r[2]!r}  (type {r[3]})")
    print(f"-- stopped at 0x{stop:x} ({len(recs)} records)")


if __name__ == "__main__":
    main()
