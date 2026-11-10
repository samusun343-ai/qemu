#!/usr/bin/env python3
"""Minimal GDB Remote Serial Protocol client for QEMU gdbstub (ARM).

Usage (library):  from rsp import Rsp; r = Rsp('localhost', 1234)
Commands: interrupt(), cont(), step(), read_mem(addr, len) -> bytes,
          set_bp(addr), del_bp(addr), regs() -> dict, send_cmd(str) -> str
"""
import socket


class Rsp:
    def __init__(self, host, port, timeout=30):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.buf = b""

    def _send(self, raw):
        self.s.sendall(raw)

    def _send_pkt(self, data):
        csum = sum(data.encode()) & 0xFF
        self._send(b"$" + data.encode() + b"#" + f"{csum:02x}".encode())

    def _recv_pkt(self, timeout=None):
        if timeout is not None:
            self.s.settimeout(timeout)
        while True:
            if b"$" in self.buf and b"#" in self.buf.split(b"$", 1)[1]:
                start = self.buf.index(b"$")
                rest = self.buf[start + 1:]
                end = rest.index(b"#")
                if len(rest) >= end + 3:
                    data = rest[:end]
                    self.buf = self.buf[start + end + 4:]
                    return data.decode(errors="replace")
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("gdbstub closed")
            self.buf += chunk

    def send_cmd(self, cmd, timeout=None):
        self._send_pkt(cmd)
        # retry on '+-' acks
        while True:
            pkt = self._recv_pkt(timeout)
            if pkt.startswith("E") and len(pkt) == 3:
                # error reply is a valid response
                return pkt
            return pkt

    def interrupt(self):
        self._send(b"\x03")
        return self._recv_pkt()

    def cont(self):
        self._send_pkt("c")
        return self._recv_pkt()

    def step(self):
        self._send_pkt("s")
        return self._recv_pkt()

    def read_mem(self, addr, length):
        out = b""
        while length > 0:
            n = min(length, 1024)
            pkt = self.send_cmd(f"m{addr:x},{n:x}")
            if pkt.startswith("E"):
                raise RuntimeError(f"read_mem error at {addr:#x}: {pkt}")
            out += bytes.fromhex(pkt)
            addr += n
            length -= n
        return out

    def write_mem(self, addr, data):
        hexdata = data.hex()
        pkt = self.send_cmd(f"M{addr:x},{len(data):x}:{hexdata}")
        return pkt

    def set_bp(self, addr, kind=4):
        return self.send_cmd(f"Z0,{addr:x},{kind}")

    def del_bp(self, addr, kind=4):
        return self.send_cmd(f"z0,{addr:x},{kind}")

    def set_watch(self, addr, kind=4, wtype=2):
        # wtype: 2=write watchpoint, 3=access, 4=read
        return self.send_cmd(f"Z{wtype},{addr:x},{kind}")

    def regs(self):
        pkt = self.send_cmd("g")
        raw = bytes.fromhex(pkt)
        names = ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
                 "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"]
        d = {}
        for i, n in enumerate(names):
            d[n] = int.from_bytes(raw[i * 4:i * 4 + 4], "little")
        d["cpsr"] = int.from_bytes(raw[16 * 4:16 * 4 + 4], "little")
        return d

    def recv_stop(self, timeout=None):
        return self._recv_pkt(timeout)
