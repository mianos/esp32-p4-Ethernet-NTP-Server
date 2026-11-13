#!/usr/bin/env python3
"""Capture the console UART to a file with timestamps.

Opens without touching DTR/RTS so the board is not reset by the capture tool
itself -- a reset would re-run the boot burst and make the log that follows
unreadable as evidence.
"""
import sys
import time

import serial

port, out = sys.argv[1], sys.argv[2]
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

s = serial.Serial()
s.port = port
s.baudrate = 115200
s.timeout = 0.2
s.dtr = False
s.rts = False
s.open()

t0 = time.time()
with open(out, "w") as f:
    while time.time() - t0 < seconds:
        data = s.read(4096)
        if data:
            f.write("%9.3f %s\n" % (time.time() - t0, data.decode("utf-8", "replace")))
            f.flush()
s.close()
