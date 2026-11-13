#!/usr/bin/env python3
"""Measure the sky over a window: how many satellites, how strong, how high.

A survey-in averages a position for hours, so what decides whether it can be
done is not "is there a fix" but the strength and the geometry that will still
be there in six hours. Reports the distribution over *all tracked* satellites,
not just the used ones, and the front-end figures beside it.

NAV-SAT group layout: gnssId, svId, cno, elev, azim, prRes(3), flags(4). Bit 3
of flags is svUsed. payload[5] is numSvs.
"""
import json
import socket
import sys
import time
import urllib.request

host = sys.argv[1] if len(sys.argv) > 1 else "esp32p4-ntp"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS", 6: "GLONASS"}

s = socket.create_connection((host, 3333), timeout=5)
s.setblocking(False)

buf = bytearray()
t0 = time.time()
nextq = t0
inview = []
cno_all = []
cno_used = []
elevs = []
consts = {}
frontend = []

while time.time() - t0 < seconds:
    if time.time() >= nextq:
        nextq += 10
        try:
            d = json.load(urllib.request.urlopen("http://%s/gps/info" % host, timeout=4))
            frontend.append((round(d["noise_dbm"], 1), d["agc"], d["jam_ind"], round(d["cn0_mean"], 1)))
        except Exception:
            pass
    try:
        data = s.recv(65536)
    except BlockingIOError:
        time.sleep(0.02)
        continue
    if not data:
        break
    buf += data
    while True:
        i = buf.find(b"\xb5\x62")
        if i < 0 or len(buf) - i < 8:
            break
        cls, mid = buf[i + 2], buf[i + 3]
        ln = buf[i + 4] | (buf[i + 5] << 8)
        if len(buf) - i < 8 + ln:
            break
        if (cls, mid) == (0x01, 0x35):
            p = buf[i + 6 : i + 6 + ln]  # payload starts after the 6-byte header
            num = p[5]
            inview.append(num)
            for k in range(num):
                g = p[8 + 12 * k : 8 + 12 * (k + 1)]
                if len(g) < 12:
                    break
                gnss, _sv, cno, el = g[0], g[1], g[2], g[3] - 256 if g[3] > 127 else g[3]
                fl = int.from_bytes(g[8:12], "little")
                consts[GNSS.get(gnss, gnss)] = consts.get(GNSS.get(gnss, gnss), 0) + 1
                if cno:
                    cno_all.append(cno)
                    elevs.append(el)
                if fl & (1 << 3) and cno:
                    cno_used.append(cno)
        del buf[: i + 8 + ln]

s.close()


def pct(vals, *qs):
    v = sorted(vals)
    if not v:
        return ["-"] * len(qs)
    return [v[min(len(v) - 1, int(q / 100 * len(v)))] for q in qs]


print("frames            %d" % len(inview))
if inview:
    print("in view           mean %.1f  max %d" % (sum(inview) / len(inview), max(inview)))
print("tracked w/ C/N0   %d sats, by constellation: %s" % (len(cno_all), consts))
for q, label in ((10, "p10"), (50, "median"), (90, "p90")):
    a = pct(cno_all, q)[0]
    print("C/N0 %-8s     %s dBHz" % (label, a))
print("above 30 / 35 / 40  %d / %d / %d" % (
    sum(1 for c in cno_all if c >= 30), sum(1 for c in cno_all if c >= 35),
    sum(1 for c in cno_all if c >= 40)))
print("used by the fix   %d  (mean C/N0 %.1f)" % (
    len(cno_used), sum(cno_used) / len(cno_used) if cno_used else 0))
if elevs:
    print("elevation p10/med  %s / %s deg" % (pct(elevs, 10)[0], pct(elevs, 50)[0]))
if frontend:
    print("front end (noise_dbm, agc, jam_ind, cn0_mean):")
    for row in frontend:
        print("   %s" % (row,))
