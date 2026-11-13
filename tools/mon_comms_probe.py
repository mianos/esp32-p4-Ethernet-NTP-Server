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

host = sys.argv[1] if len(sys.argv) > 1 else "esp32p4-ntp"
MON_COMMS = bytes([0xB5, 0x62, 0x0A, 0x36, 0x00, 0x00, 0x40, 0xCA])

s = socket.create_connection((host, 3333), timeout=5)
s.setblocking(False)
t0 = time.time()
s.sendall(MON_COMMS)
print("sent MON-COMMS poll")

total = 0
frames = 0
classes = {}
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
    while True:
        i = buf.find(b"\xb5\x62")
        if i < 0 or len(buf) - i < 8:
            break
        cls, mid = buf[i + 2], buf[i + 3]
        ln = buf[i + 4] | (buf[i + 5] << 8)
        if len(buf) - i < 8 + ln:
            break
        frames += 1
        classes[(cls, mid)] = classes.get((cls, mid), 0) + 1
        del buf[: i + 8 + ln]

print("bytes %d, frames %d" % (total, frames))
for (c, m), n in sorted(classes.items(), key=lambda kv: -kv[1])[:10]:
    print("  %02x/%02x x%d" % (c, m, n))
print("unparsed tail %d bytes: %s" % (len(buf), bytes(buf[:24]).hex(" ")))
s.close()
