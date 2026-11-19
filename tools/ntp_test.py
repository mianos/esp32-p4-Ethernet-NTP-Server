#!/usr/bin/env python3
"""Wire-level checks against the NTP responder, independent of any NTP client.

    python3 tools/ntp_test.py 192.0.2.20
    python3 tools/ntp_test.py 192.0.2.20 2001:db8::50    # also probe IPv6

sntp/chrony tell you whether a *client* accepts the server; this tells you what
is on the wire, including the packets a real client never sends. It needs no
dependencies beyond python3.

The last section asks the mode 6 control interface what the server thinks of
itself, and checks the reply against RFC 9327's framing (sequence and associd
echoed, count honoured, datagram padded) rather than just eyeballing it.

One warning that will otherwise look like a bug: the burst section deliberately
trips the rate limiter for your address (a 16-packet burst, then 1 packet per
2 s). Run this script no more often than about once a minute, and avoid running
`sntp` right after it -- macOS sntp sends five queries per invocation, so
back-to-back runs answer with a RATE kiss-of-death and then silence for a second.
"""
import argparse
import random
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
    show("mode 3, vn 3 (reply carries vn 3)", ask(args.host, request(version=3)))

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

    control_section(args.host)
    # The burst above emptied this host's token bucket and mode 6 does not use
    # it, so let the limiter refill before asking for replies again: this
    # section needs six tokens and they arrive one per two seconds.
    time.sleep(11)
    interleaved_section(args.host)

    return 0


def exchange(host, origin=b"\x00" * 8, receive=b"\x00" * 8, transmit=None):
    """One mode 3 packet with chosen origin and receive fields.

    `transmit` defaults to a fresh value so that origin == receive is not
    accidentally also equal to transmit, which would be read as a basic request.
    """
    if transmit is None:
        transmit = struct.pack(">II", int(time.time()) + 2208988800, random.getrandbits(32))
    hdr = bytearray(request(transmit))
    hdr[24:32] = origin
    hdr[32:40] = receive
    return ask(host, bytes(hdr), timeout=2.0)


def interleaved_section(host):
    print("\n== interleaved mode (RFC 9769) ==")

    # 1. The first request of an association is basic: zero origin.
    first = exchange(host)
    if first is None:
        print("first (basic)                     NO REPLY")
        return
    print(f"first (basic)                     len={len(first)} origin_echoed="
          f"{first[24:32] == struct.unpack_from('>8s', first, 24)[0]}")
    t2_first = first[32:40]
    t3_first = first[40:48]

    # 2. A request whose origin is the previous receive timestamp is interleaved:
    #    the reply must echo that as origin and carry the *previous* transmit
    #    timestamp, not a fresh one.
    t1 = struct.pack(">II", int(time.time()) + 2208988800, random.getrandbits(32))
    receive = struct.pack(">II", int(time.time()) + 2208988800, random.getrandbits(32))
    second = exchange(host, origin=t2_first, receive=receive, transmit=t1)
    if second is None:
        print("second (interleaved)              NO REPLY")
        return
    got_origin = second[24:32]
    got_tx = second[40:48]
    print(f"second (interleaved)              len={len(second)} "
          f"origin_is_our_receive={got_origin == receive} "
          f"transmit_is_previous={got_tx == t3_first}")

    # 3. The same origin again must not answer interleaved a second time: the
    #    entry was consumed.
    third = exchange(host, origin=t2_first, receive=receive, transmit=t1)
    if third is None:
        print("reused origin                     NO REPLY")
    else:
        print(f"reused origin                     origin_echo={third[24:32] == t3_first} "
              f"(basic reply echoes our transmit; interleaved would echo our receive)")

    # 4. Basic requests stay basic: an origin nobody knows.
    junk = struct.pack(">II", 1, 2)
    fourth = exchange(host, origin=junk)
    if fourth is None:
        print("unknown origin                    NO REPLY")
    else:
        print(f"unknown origin                    interleaved={fourth[24:32] == junk} "
              f"(must be False: basic echoes the transmit field, not the origin)")

    # 5. A request whose receive equals its transmit cannot be interleaved. The
    #    reply's origin cannot tell the modes apart in that case (both echo the
    #    same value), so this uses the transmit field and an aged pair: a basic
    #    reply stamps the moment, an interleaved one replays the old transmit.
    fresh = exchange(host)
    if fresh is None:
        print("receive == transmit               NO REPLY")
        return
    t2_fresh, t3_fresh = fresh[32:40], fresh[40:48]
    time.sleep(2.2)
    same = struct.pack(">II", int(time.time()) + 2208988800, 7)
    fifth = exchange(host, origin=t2_fresh, receive=same, transmit=same)
    if fifth is None:
        print("receive == transmit               NO REPLY")
    else:
        print(f"receive == transmit               replayed_old_transmit="
              f"{fifth[40:48] == t3_fresh} (must be False: basic)")

    print(f"  check /healthz ntp_interleaved_served increments, and "
          f"ntp_tx_correction_us is a plausible few-microsecond figure")


def control(host, opcode, associd=0, data=b"", sequence=None):
    """A mode 6 request: header, then the data field, padded to 4 octets."""
    if sequence is None:
        sequence = random.getrandbits(15) | 1
    hdr = bytearray(12)
    hdr[0] = (4 << 3) | 6
    hdr[1] = opcode
    struct.pack_into(">H", hdr, 2, sequence)
    struct.pack_into(">H", hdr, 6, associd)
    struct.pack_into(">H", hdr, 10, len(data))
    payload = bytes(hdr) + data
    payload += b"\x00" * (-len(payload) % 4)
    return payload, sequence


def mode6(host, opcode, associd=0, data=b"", label=""):
    """Send a control request and check the reply honours the protocol."""
    payload, sequence = control(host, opcode, associd, data)
    reply = ask(host, payload, timeout=2.0)
    if reply is None:
        print(f"{label:<34} NO REPLY")
        return None
    mode = reply[0] & 7
    r_bit = bool(reply[1] & 0x80)
    e_bit = bool(reply[1] & 0x40)
    echoed_seq = struct.unpack_from(">H", reply, 2)[0]
    echoed_assoc = struct.unpack_from(">H", reply, 6)[0]
    count = struct.unpack_from(">H", reply, 10)[0]
    ok_shape = (mode == 6 and r_bit and echoed_seq == sequence and
                echoed_assoc == associd and len(reply) % 4 == 0 and
                len(reply) >= 12 + count and (12 + count + 3) // 4 * 4 == len(reply))
    print(f"{label:<34} len={len(reply)} mode={mode} R={int(r_bit)} E={int(e_bit)} "
          f"seq_ok={echoed_seq == sequence} assoc={echoed_assoc} count={count} "
          f"padded={len(reply) % 4 == 0} shape_ok={ok_shape}")
    return reply


def fields(reply):
    """The reply's data field as a dict, split the way ntpq splits it."""
    count = struct.unpack_from(">H", reply, 10)[0]
    text = reply[12:12 + count].decode("ascii", "replace")
    pairs = {}
    for item in text.split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            pairs[key.strip()] = value.strip()
    return pairs


def control_section(host):
    print("\n== mode 6 (ntpq) ==")

    reply = mode6(host, 2, 0, label="readvar, system")
    if reply is not None and not reply[1] & 0x40:
        got = fields(reply)
        print(f"  stratum={got.get('stratum')} refid={got.get('refid')} "
              f"rootdisp={got.get('rootdisp')} leap={got.get('leap')} "
              f"jitter={got.get('sys_jitter')} vars={len(got)}")
        print(f"  status={struct.unpack_from('>H', reply, 4)[0]:04x} "
              f"(LI/source/event of the system status word)")
        missing = [k for k in ("stratum", "refid", "rootdisp", "reftime") if k not in got]
        print(f"  expected stratum 1 or 3 with a refid; missing={missing}")

    reply = mode6(host, 2, 1, label="readvar, association 1")
    if reply is not None and not reply[1] & 0x40:
        got = fields(reply)
        print(f"  srcadr={got.get('srcadr')} srchost={got.get('srchost')} "
              f"stratum={got.get('stratum')} reach={got.get('reach')} "
              f"hmode={got.get('hmode')}")
        selection = " x.-+#*o"[struct.unpack_from(">H", reply, 4)[0] >> 8 & 0x7]
        print(f"  selection char {selection!r} (must be '*' while locked, and the "
              f"status must carry conf|reach)")

    reply = mode6(host, 1, 0, label="readstat")
    if reply is not None and not reply[1] & 0x40:
        count = struct.unpack_from(">H", reply, 10)[0]
        rows = [struct.unpack_from(">HH", reply, 12 + 4 * i) for i in range(count // 4)]
        print(f"  associations={rows} (one row, associd 1, status high byte 0x96 while locked)")

    reply = mode6(host, 2, 0, b"stratum,refid", label="readvar, named subset")
    if reply is not None:
        print(f"  returned={sorted(fields(reply))} (must be exactly the two named)")

    # A client that leaves the count at zero but sends names means all of them,
    # not none: answering the whole document here would hide the omission.
    payload, sequence = control(host, 2, 0, b"stratum,refid")
    payload = payload[:10] + b"\x00\x00" + payload[12:]
    reply = ask(host, payload, timeout=2.0)
    if reply is None:
        print(f"{'readvar, count 0':<34} NO REPLY")
    else:
        print(f"{'readvar, count 0':<34} E={int(bool(reply[1] & 0x40))} "
              f"returned={sorted(fields(reply))} (must still be the two named)")

    reply = mode6(host, 2, 99, label="readvar, bad association")
    if reply is not None:
        print(f"  status={struct.unpack_from('>H', reply, 4)[0]:04x} "
              f"(expected 0x0400 = bad association, E set)")

    reply = mode6(host, 3, 0, label="writevar, refused")
    if reply is not None:
        print(f"  status={struct.unpack_from('>H', reply, 4)[0]:04x} "
              f"(expected 0x0100 = permission denied, E set)")

    reply = mode6(host, 30, 0, label="opcode 30, reserved")
    if reply is not None:
        print(f"  status={struct.unpack_from('>H', reply, 4)[0]:04x} "
              f"(expected 0x0300 = bad opcode, E set)")

    print("\n  now the real client, if it is installed:")
    print(f"    ntpq -p {host} ; ntpq -c rv {host} ; ntpq -c sysstats {host}")


if __name__ == "__main__":
    sys.exit(main())
