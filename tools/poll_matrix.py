#!/usr/bin/env python3
"""Ask the receiver directly, through the bridge, which polls it answers.

The bridge synthesises a MON-COMMS reply when the module stays silent, so a
0a/36 with a 12-byte payload is ours; the module's own would be 8 + 26 * nPorts.
This separates the two and also checks that ordinary polls (MON-VER, CFG-PRT)
are answered at all -- if they are, the receiver is fine and only MON-COMMS is
being ignored.
"""
import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else "esp32p4-ntp"


def frame(cls, mid, payload=b""):
    f = bytes([0xB5, 0x62, cls, mid, len(payload) & 0xFF, len(payload) >> 8]) + payload
    a = b = 0
    for x in f[2:]:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return f + bytes([a, b])


POLLS = [
    ("MON-COMMS", frame(0x0A, 0x36)),
    ("MON-VER", frame(0x0A, 0x04)),
    ("CFG-PRT poll", frame(0x06, 0x00)),
    ("NAV-SAT poll", frame(0x01, 0x35)),
]

s = socket.create_connection((host, 3333), timeout=5)
s.setblocking(False)
buf = bytearray()


def drain(seconds):
    out = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            data = s.recv(65536)
        except BlockingIOError:
            time.sleep(0.02)
            continue
        if not data:
            break
        buf.extend(data)
    while True:
        i = buf.find(b"\xb5\x62")
        if i < 0 or len(buf) - i < 8:
            break
        cls, mid = buf[i + 2], buf[i + 3]
        ln = buf[i + 4] | (buf[i + 5] << 8)
        if len(buf) - i < 8 + ln:
            break
        out.append((cls, mid, ln, bytes(buf[i + 8 : i + 8 + ln])))
        del buf[: i + 8 + ln]
    return out


for name, req in POLLS:
    s.sendall(req)
    got = drain(2.5)
    answers = [(c, m, n) for (c, m, n, _) in got if (c, m) == (req[2], req[3])]
    others = sorted({(c, m) for (c, m, n, _) in got if (c, m) != (req[2], req[3])})
    note = ""
    for (c, m, n, p) in got:
        if (c, m, n) in answers and c == 0x0A and m == 0x36:
            note = "  <- payload len %d (%s)" % (n, "bridge fake" if n == 12 else "MODULE")
    print("%-13s answers %s%s" % (name, answers or "none", note))
    if others:
        print("                also saw %s" % ", ".join("%02x/%02x" % o for o in others))
s.close()
