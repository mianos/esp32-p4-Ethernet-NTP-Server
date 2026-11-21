#!/usr/bin/env python3
"""Capture the console UART to a file with timestamps.

Opens the port the way `stty` and `cat` do, and leaves DTR and RTS exactly as
the open left them, so the board is not reset by the capture tool itself -- a
reset would re-run the boot burst and make the log that follows unreadable as
evidence.

This used to go through pyserial with dtr=False and rts=False. On Linux the
kernel raises both lines on open and pyserial then drops DTR before RTS, and
DTR low with RTS high is the auto-reset circuit's "pull EN" state: the tool
that promised not to reset the board reset it every time.
"""
import os
import sys
import termios
import time

port, out = sys.argv[1], sys.argv[2]
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = 0                                        # iflag: raw
attrs[1] = 0                                        # oflag: raw
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: no HUPCL, so
attrs[3] = 0                                        # close leaves the lines too
attrs[4] = attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)

t0 = time.time()
with open(out, "w") as f:
    while time.time() - t0 < seconds:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b""
        if data:
            f.write("%9.3f %s\n" % (time.time() - t0, data.decode("utf-8", "replace")))
            f.flush()
        else:
            time.sleep(0.05)
os.close(fd)
