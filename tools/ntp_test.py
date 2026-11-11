#!/usr/bin/env python3
"""Wire-level checks against the NTP responder, independent of any NTP client.

    python3 tools/ntp_test.py 192.168.1.50
    python3 tools/ntp_test.py 192.168.1.50 2401:db8::50    # also probe IPv6

sntp/chrony tell you whether a *client* accepts the server; this tells you what
is on the wire, including the packets a real client never sends. It needs no
dependencies beyond python3.

One warning that will otherwise look like a bug: the burst section deliberately
trips the rate limiter for your address (8 burst, then 1 packet per 2 s). Run
this script no more often than about once a minute, and avoid running `sntp`
right after it -- macOS sntp sends five queries per invocation, so back-to-back
runs answer with a RATE kiss-of-death and then silence for a second.
"""
import argparse
import socket
import struct
import sys
import time

TRANS = bytes(range(8))  # a distinctive client transmit timestamp


def request(transmit=TRANS, version=4, mode=3, poll=0):
    hdr = bytearray(48)
    hdr[0] = (version << 3) | mode
    hdr[2] = poll
    hdr[40:48] = transmit
    return bytes(hdr)


def ask(host, payload, timeout=1.0):
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(family, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(payload, (host, 123))
        return sock.recvfrom(512)[0]
    except socket.timeout:
        return None
    finally:
        sock.close()


def show(name, reply, transmitter=TRANS):
    if reply is None:
        print(f"{name:<34} NO REPLY")
        return
    li = reply[0] >> 6
    vn = (reply[0] >> 3) & 7
    mode = reply[0] & 7
    stratum = reply[1]
    root_delay, root_disp = struct.unpack(">II", reply[4:12])
    print(f"{name:<34} len={len(reply)} li={li} vn={vn} mode={mode} "
          f"stratum={stratum} prec={struct.unpack('b', reply[3:4])[0]} "
          f"delay={root_delay / 65536.0:.6f} disp={root_disp / 65536.0:.6f} "
          f"refid={reply[12:16]!r} origin_ok={reply[24:32] == transmitter}")


def main():
    parser = argparse.ArgumentParser(description="Probe an NTP responder on port 123.")
    parser.add_argument("host", help="IPv4 address or hostname of the device")
    parser.add_argument("host6", nargs="?", help="its IPv6 address, to check dual-stack")
    args = parser.parse_args()

    print("== a request a client would send ==")
    show("mode 3, vn 4", ask(args.host, request()))

    print("\n== requests that must stay silent ==")
    show("mode 4 (server)", ask(args.host, request(mode=4)))
    show("mode 1 (symmetric active)", ask(args.host, request(mode=1)))
    show("vn 2", ask(args.host, request(version=2)))
    show("12 bytes", ask(args.host, b"\x23" + b"\x00" * 11))

    if args.host6:
        print("\n== the same over IPv6 ==")
        show("mode 3, vn 4", ask(args.host6, request()))

    print("\n== origin timestamp echoed for a zero transmit ==")
    zeros = b"\x00" * 8
    reply = ask(args.host, request(zeros))
    show("mode 3, transmit=0", reply, transmitter=zeros)

    print("\n== burst of 20 from one address (this trips the limiter) ==")
    answered = kissed = silent = 0
    for _ in range(20):
        reply = ask(args.host, request(), timeout=0.5)
        if reply is None:
            silent += 1
        elif reply[1] == 0:
            kissed += 1
            if reply[12:16] != b"RATE":
                print(f"  unexpected kiss code: {reply[12:16]!r}")
        else:
            answered += 1
    print(f"answered={answered} kod={kissed} silent={silent}")
    print("expected: the whole 16-packet burst answered, then RATE, then silence; "
          "the counts\ndrift between runs because tokens refill while the loop waits")

    print("\n== is it still serving after the burst? ==")
    time.sleep(2.5)
    show("mode 3, vn 4", ask(args.host, request(), timeout=2.0))

    return 0


if __name__ == "__main__":
    sys.exit(main())
