#!/usr/bin/env python3
"""gdbrsp.py - minimal GDB Remote Serial Protocol client (ares N64 debug server, Azahar gdbstub).

Only what statediff needs: read registers/memory, software breakpoints, continue, interrupt.
"""
import socket, time


class GdbRsp:
    def __init__(self, host="127.0.0.1", port=9123, timeout=30.0, connect_wait=60.0):
        deadline = time.time() + connect_wait
        while True:
            try:
                self.s = socket.create_connection((host, port), timeout=timeout)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.5)
        self.buf = b""
        self.s.sendall(b"+")  # real gdb opens with an ack; ares drops clients that don't
        self.cmd("qSupported:multiprocess+;swbreak+;hwbreak+;qRelocInsn+;fork-events+;vfork-events+;exec-events+;vContSupported+;QThreadEvents+;no-resumed+;xmlRegisters=i386")

    # --- packet layer ---
    def _recv_byte(self):
        if not self.buf:
            self.buf = self.s.recv(65536)
            if not self.buf:
                raise ConnectionError("gdb server closed the connection")
        b, self.buf = self.buf[:1], self.buf[1:]
        return b

    def _read_packet(self):
        while True:
            b = self._recv_byte()
            if b == b"$":
                break
        data = b""
        while True:
            b = self._recv_byte()
            if b == b"#":
                break
            data += b
        self._recv_byte(); self._recv_byte()  # checksum
        self.s.sendall(b"+")
        return data.decode("latin-1")

    def send(self, payload):
        csum = sum(payload.encode("latin-1")) & 0xFF
        self.s.sendall(("$%s#%02x" % (payload, csum)).encode("latin-1"))

    def cmd(self, payload):
        self.send(payload)
        return self._read_packet()

    # --- operations ---
    def read_regs(self, reg_hex_digits=16):
        """Return the 'g' register block as a list of ints (ares MIPS: 64-bit regs, big-endian hex)."""
        r = self.cmd("g")
        return [int(r[i:i + reg_hex_digits], 16) for i in range(0, len(r) - reg_hex_digits + 1, reg_hex_digits)]

    def read_mem(self, addr, size, chunk=0x800):
        out = bytearray()
        while size > 0:
            n = min(chunk, size)
            r = self.cmd("m%x,%x" % (addr, n))
            if (len(r) == 3 and r[0] == "E") or len(r) != 2 * n:  # error replies are exactly "Exx"
                raise IOError("memory read failed at 0x%x (%s)" % (addr, r[:16]))
            out += bytes.fromhex(r)
            addr += n
            size -= n
        return bytes(out)

    def write_mem(self, addr, data):
        r = self.cmd("M%x,%x:%s" % (addr, len(data), data.hex()))
        if r != "OK":
            raise IOError("memory write failed at 0x%x (%s)" % (addr, r[:16]))

    def set_break(self, addr, kind=4):
        r = self.cmd("Z0,%x,%d" % (addr, kind))
        if r != "OK":
            raise IOError("breakpoint at 0x%x refused: %r" % (addr, r))

    def clear_break(self, addr, kind=4):
        self.cmd("z0,%x,%d" % (addr, kind))

    def cont(self, timeout=None):
        """Continue and wait for the next stop reply (T/S packet)."""
        self.send("c")
        old = self.s.gettimeout()
        self.s.settimeout(timeout)
        try:
            while True:
                r = self._read_packet()
                if r[:1] in ("T", "S", "W", "X"):
                    return r
        finally:
            self.s.settimeout(old)

    def step(self, timeout=10):
        self.send("s")
        old = self.s.gettimeout()
        self.s.settimeout(timeout)
        try:
            while True:
                r = self._read_packet()
                if r[:1] in ("T", "S", "W", "X"):
                    return r
        finally:
            self.s.settimeout(old)

    def cont_past(self, addr, timeout=None):
        """Continue from a stop AT breakpoint `addr`: lift it, step one instruction, re-arm, continue."""
        self.clear_break(addr)
        self.step()
        self.set_break(addr)
        return self.cont(timeout)

    def pc(self):
        return self.read_regs()[37] & 0xFFFFFFFF  # ares MIPS 'g' block: 32 GPRs, sr, lo, hi, bad, cause, pc

    def cont_until(self, addr, timeout=None, from_addr=None):
        """Continue until execution stops AT `addr`, passing over other stops (ares also reports some
        exceptions the game handles itself, e.g. one in AudioLoad_Init at boot)."""
        r = self.cont_past(from_addr, timeout) if from_addr is not None else self.cont(timeout)
        while self.pc() != addr:
            r = self.cont(timeout)
        return r

    def interrupt(self):
        self.s.sendall(b"\x03")
        return self._read_packet()

    def detach(self):
        try:
            self.cmd("D")
        except Exception:
            pass
        self.s.close()
