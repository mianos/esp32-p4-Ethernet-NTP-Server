#!/usr/bin/env python3
"""Stand in for u-center's opening handshake and report what comes back.

u-center attaches by polling MON-COMMS; if the receiver does not answer it
waits out a timeout and starts again. This sends that exact poll and then
holds the socket, so the bridge's reply -- and anything that follows -- is
visible as bytes rather than as "u-center does nothing".
"""
import socket
import sys
import time
from collections import Counter

import ubx

host = sys.argv[1] if len(sys.argv) > 1 else "esp32p4-ntp"
MON_COMMS = bytes([0xB5, 0x62, 0x0A, 0x36, 0x00, 0x00, 0x40, 0xCA])

s = socket.create_connection((host, 3333), timeout=5)
s.setblocking(False)
t0 = time.time()
s.sendall(MON_COMMS)
print("sent MON-COMMS poll")

total = 0
buf = bytearray()
while time.time() - t0 < 6.0:
    try:
        data = s.recv(65536)
    except BlockingIOError:
        time.sleep(0.02)
        continue
    if not data:
        print("closed by peer")
        break
    total += len(data)
    buf += data

# One pass at the end, over the whole capture: a frame split across reads is
# still a frame, which is the thing a byte search would have lost.
dec = ubx.Decoder()
dec.feed(bytes(buf))

print("bytes %d, frames %d, dropped on checksum %d" % (total, len(dec.frames), len(dec.dropped_checksum)))
for (c, m), n in sorted(Counter((f.cls, f.id) for f in dec.frames).items(), key=lambda kv: -kv[1])[:10]:
    print("  %02x/%02x x%d" % (c, m, n))
s.close()
