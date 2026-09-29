#!/usr/bin/env python3
"""Drive the USB-Serial/JTAG at ws://127.0.0.1:PORT/usj (protocol: esp-soc/src/web.rs) the way the
browser flasher does, without a browser: open (and a second client is refused), esptool-js 0.6.0
UsbJtagSerialReset (reset.ts:113-129) into download mode, SYNC, READ_REG of the ESP32 revision word
esptool-js reads from every chip (0x3FF5A00C), ESP Web Tools' reset after flashing (RTS pulse),
Improv Serial REQUEST_INFO to the booted firmware, close. Prints what it sees with wall times.
usage: usjprobe.py PORT   (a run with --boot rom --web PORT). Standard library only."""
import base64, json, os, socket, struct, sys, time

class WS:
    def __init__(self, port, path="/usj"):
        self.s = socket.create_connection(("127.0.0.1", port)); self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(f"GET {path} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        head = b""
        while b"\r\n\r\n" not in head: head += self.s.recv(1)
        self.status = head.split(b"\r\n")[0].decode()
        self.buf = b""
    def send(self, payload, op=2):
        m = os.urandom(4); n = len(payload)
        h = bytes([0x80 | op]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n) if n < 65536 else bytes([0x80 | 127]) + struct.pack(">Q", n))
        self.s.sendall(h + m + bytes(b ^ m[i & 3] for i, b in enumerate(payload)))
    def _need(self, n, deadline):
        while len(self.buf) < n:
            self.s.settimeout(max(0.001, deadline - time.time()))
            try: d = self.s.recv(65536)
            except socket.timeout: return False
            if not d: return False
            self.buf += d
        return True
    def recv(self, timeout):
        dl = time.time() + timeout
        if not self._need(2, dl): return None
        n = self.buf[1] & 0x7f; off = 2
        if n == 126: self._need(4, dl); n = struct.unpack(">H", self.buf[2:4])[0]; off = 4
        elif n == 127: self._need(10, dl); n = struct.unpack(">Q", self.buf[2:10])[0]; off = 10
        if not self._need(off + n, dl): return None
        op, d = self.buf[0] & 0xf, self.buf[off:off + n]; self.buf = self.buf[off + n:]
        return op, d

def slip(b): return b"\xc0" + b.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"
def cmd(op, data, chk=0): return slip(struct.pack("<BBHI", 0, op, len(data), chk) + data)

port = int(sys.argv[1]); t0 = time.time()
def log(*a): print(f"{time.time() - t0:7.3f}", *a, flush=True)
ws = WS(port); log("upgrade:", ws.status)
data = bytearray()
def pump(sec, until=None):
    end = time.time() + sec
    while time.time() < end:
        f = ws.recv(end - time.time())
        if f is None: break
        op, d = f
        if op == 2 and d[:1] == b"\x10": log("event", d[1:].decode())
        elif op == 2 and d[:1] == b"\x00": data.extend(d[1:])
        if until and until(): return True
    return False
ws.send(b"\x02"); pump(1.0, lambda: False)
other = WS(port); other.send(b"\x02"); f = other.recv(2.0); log("second client:", f and f[1][1:].decode()); other.s.close()

dtr = False
def lines(d, r): ws.send(bytes([1, (1 if d else 0) | (2 if r else 0)]))
def setDTR(v):
    global dtr; dtr = v; lines(dtr, rts)
def setRTS(v):
    global rts; rts = v; lines(dtr, rts); setDTR(dtr)
rts = False
# esptool-js 0.6.0 reset.ts:113-129
setRTS(False); setDTR(False); time.sleep(0.1)
setDTR(True); setRTS(False); time.sleep(0.1)
setRTS(True); setDTR(False); setRTS(True); time.sleep(0.1)
setRTS(False); setDTR(False)
pump(1.0, lambda: b"waiting for download" in data)
log("banner:", bytes(data[data.find(b"rst:"):]).split(b"\r\n")[0].decode(errors="replace"), "| waiting for download:", b"waiting for download" in data)
data.clear()
ts = time.time(); ws.send(b"\x00" + cmd(0x08, b"\x07\x07\x12\x20" + b"\x55" * 32))
pump(1.0, lambda: data.count(b"\xc0\x01\x08\x04\x00\x07\x07\x12\x20") >= 8)
n_sync = data.count(bytes.fromhex("c00108040007071220")); log(f"SYNC: {n_sync} answers, all within {(time.time() - ts) * 1000:.0f} ms")
data.clear()
ts = time.time(); ws.send(b"\x00" + cmd(0x0a, struct.pack("<I", 0x3FF5A00C)))
pump(1.0, lambda: data.endswith(b"\xc0") and len(data) >= 14)
log(f"READ_REG 0x3ff5a00c -> {bytes(data).hex()} in {(time.time() - ts) * 1000:.0f} ms")
data.clear()
# ESP Web Tools 10.4.0 after flashing: setRTS(true), 100 ms, HardReset: 100 ms, setRTS(false)
setRTS(True); time.sleep(0.2); setRTS(False)
pump(8.0, lambda: b"Improv: listening" in data)
log("after reset:", [l for l in bytes(data).decode(errors="replace").split("\r\n") if l.startswith("rst:") or "Improv" in l][:3])
data.clear()
imp = b"IMPROV" + bytes([1, 3, 2, 3, 0]); imp += bytes([sum(imp) & 0xff, 10])
ws.send(b"\x00" + imp); pump(3.0, lambda: b"AnimatedPixelClock" in data)
j = data.find(b"IMPROV"); log("improv info:", bytes(data[j + 9: j + 9 + data[j + 8]]) if j >= 0 else None)
ws.send(b"\x03"); pump(1.0, lambda: False)
ws.send(b"\x03\xe8", op=8); log("close answer:", ws.recv(1.0))
