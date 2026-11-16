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

import ubx

host = sys.argv[1] if len(sys.argv) > 1 else "esp32p4-ntp"


POLLS = [
    ("MON-COMMS", ubx.build(0x0A, 0x36)),
    ("MON-VER", ubx.build(0x0A, 0x04)),
    ("CFG-PRT poll", ubx.build(0x06, 0x00)),
    ("NAV-SAT poll", ubx.build(0x01, 0x35)),
]

s = socket.create_connection((host, 3333), timeout=5)
s.setblocking(False)


def drain(seconds):
    """Frames that arrived in the window, through the same decoder the firmware
    uses -- so a split frame still counts and a corrupt one does not."""
    dec = ubx.Decoder()
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            data = s.recv(65536)
        except BlockingIOError:
            time.sleep(0.02)
            continue
        if not data:
            break
        dec.feed(data)
    if dec.dropped_checksum:
        print("                dropped %d frame(s) on checksum" % len(dec.dropped_checksum))
    return dec.frames


for name, req in POLLS:
    s.sendall(req)
    got = drain(2.5)
    answers = [(f.cls, f.id, f.len) for f in got if (f.cls, f.id) == (req[2], req[3])]
    others = sorted({(f.cls, f.id) for f in got if (f.cls, f.id) != (req[2], req[3])})
    note = ""
    for f in got:
        if (f.cls, f.id) == (req[2], req[3]) and (f.cls, f.id) == (0x0A, 0x36):
            note = "  <- payload len %d (%s)" % (f.len, "bridge fake" if f.len == 12 else "MODULE")
    print("%-13s answers %s%s" % (name, answers or "none", note))
    if others:
        print("                also saw %s" % ", ".join("%02x/%02x" % o for o in others))
s.close()
