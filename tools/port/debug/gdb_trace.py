#!/usr/bin/env python3
"""gdb_trace.py - which functions run before a crash, through Azahar's GDB stub.

Sets a one-shot breakpoint on every function of the given object files (comma-separated .o paths, e.g.
build/3ds/src/code/z_bgcheck.o), boots build/3ds/mm.3ds and prints the functions in the order they are first
entered (r0, r1, lr), until the emulator stops or quits. Azahar's stub cannot single-step, so each breakpoint fires
once. The last function entered is where to look. Notes: send '?' before anything else (Azahar crashes otherwise);
the 'g' packet gives r0-r15 as 8 hex digits, little-endian. Azahar's qt-config.ini is restored afterwards.

usage: tools/port/debug/gdb_trace.py OBJ[,OBJ...]
"""
import bisect, collections, os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)
from gdbrsp import GdbRsp  # noqa: E402

CFG = os.path.expanduser("~/Library/Application Support/Azahar/config/qt-config.ini")
ELF = os.path.join(REPO, "build/3ds/mm.elf")
ROM = os.path.join(REPO, "build/3ds/mm.3ds")
NM = "/opt/devkitpro/devkitARM/bin/arm-none-eabi-nm"


def main():
    names = set()
    for o in sys.argv[1].split(","):
        out = subprocess.run([NM, "--defined-only", o], capture_output=True, text=True).stdout
        names |= {l.split()[2] for l in out.splitlines() if len(l.split()) == 3 and l.split()[1] in "tT"}
    addr = {}
    for l in subprocess.run([NM, "--defined-only", ELF], capture_output=True, text=True).stdout.splitlines():
        f = l.split()
        if len(f) == 3 and f[2] in names and f[1] in "tT":
            addr[int(f[0], 16)] = f[2]
    app = subprocess.run("ls -d /Applications/*[Aa]zahar*/Azahar.app | head -1", shell=True, capture_output=True,
                         text=True).stdout.strip()
    keep = open(CFG).read()
    open(CFG, "w").write(keep.replace("use_gdbstub=false", "use_gdbstub=true")
                         .replace("use_gdbstub\\default=true", "use_gdbstub\\default=false"))
    seq = collections.deque(maxlen=40)
    try:
        subprocess.run("pkill -9 -f MacOS/azahar", shell=True)
        time.sleep(1)
        subprocess.run(["open", "-n", "-a", app, "--args", ROM])
        g = GdbRsp(port=24689, timeout=30.0, connect_wait=90.0)
        g.cmd("?")
        for a in addr:
            g.set_break(a)
        print("breakpoints:", len(addr), flush=True)
        while True:
            g.cont(timeout=60)
            r = g.cmd("g")
            reg = lambda i: int.from_bytes(bytes.fromhex(r[i * 8:i * 8 + 8]), "little")
            pc = reg(15)
            seq.append("%s r0=%08x r1=%08x lr=%08x" % (addr.get(pc, "?%08x" % pc), reg(0), reg(1), reg(14)))
            g.clear_break(pc)
    except Exception as e:
        print("ended:", type(e).__name__)
    finally:
        open(CFG, "w").write(keep)
        subprocess.run("pkill -9 -f MacOS/azahar", shell=True)
    print("\n".join(seq))


if __name__ == "__main__":
    main()
