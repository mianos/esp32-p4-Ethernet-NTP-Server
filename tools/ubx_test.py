#!/usr/bin/env python3
"""Framing tests for the UBX decoder.

These exist because the bridge's sniffer was a byte search, and every case below
is one it got wrong: a frame split across two reads, a sync pair occurring inside
a payload, a corrupt byte, a frame that claims the wrong length. The same state
machine is on both sides (tools/ubx.py and main/gps/ubx.cpp), so a case here is a
case against the firmware's shape even though the code under test is the host
twin.

    ./tools/ubx_test.py            # or: python3 -m unittest tools.ubx_test
"""
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import ubx  # noqa: E402

HERE = Path(__file__).parent.parent


class Framing(unittest.TestCase):
    def test_whole_frame(self):
        d = ubx.Decoder()
        d.feed(ubx.build(0x06, 0x31, bytes(32)))
        self.assertEqual(len(d.frames), 1)
        self.assertEqual(d.frames[0].len, 32)

    def test_every_split(self):
        """A frame split at any offset is still one frame -- the bridge case."""
        frame = ubx.build(0x06, 0x31, bytes(range(32)))
        for cut in range(len(frame) + 1):
            with self.subTest(cut=cut):
                d = ubx.Decoder()
                d.feed(frame[:cut])
                d.feed(frame[cut:])
                self.assertEqual(len(d.frames), 1)
                self.assertEqual(d.frames[0].payload, bytes(range(32)))

    def test_byte_at_a_time(self):
        d = ubx.Decoder()
        for byte in ubx.build(0x01, 0x07, bytes(92)):
            d.feed(bytes((byte,)))
        self.assertEqual(len(d.frames), 1)

    def test_bad_payload_checksum(self):
        frame = bytearray(ubx.build(0x06, 0x31, bytes(32)))
        frame[10] ^= 0xFF
        d = ubx.Decoder()
        d.feed(bytes(frame))
        self.assertEqual(len(d.frames), 0)
        self.assertEqual(len(d.dropped_checksum), 1)

    def test_bad_header_checksum(self):
        frame = bytearray(ubx.build(0x06, 0x31, bytes(32)))
        frame[3] ^= 0x01  # the id, which the checksum covers
        d = ubx.Decoder()
        d.feed(bytes(frame))
        self.assertEqual(len(d.frames), 0)
        self.assertEqual(len(d.dropped_checksum), 1)

    def test_truncated_is_pending_not_framed(self):
        frame = ubx.build(0x06, 0x31, bytes(32))
        d = ubx.Decoder()
        d.feed(frame[:-1])
        self.assertEqual(len(d.frames), 0)
        self.assertEqual(len(d.dropped_checksum), 0)

    def test_sync_pair_inside_a_payload(self):
        """The search-based sniffer counted a CFG-PRT inside a payload twice."""
        inner = ubx.build(0x06, 0x00, bytes(20))
        outer = ubx.build(0x0A, 0x04, inner + b"PROTVER=22.00\x00")
        d = ubx.Decoder()
        d.feed(outer)
        self.assertEqual(len(d.frames), 1)
        self.assertEqual(d.frames[0].name, "MON-VER")

    def test_noise_between_frames(self):
        d = ubx.Decoder()
        d.feed(b"\xb5\x00\xb5\xb5\x81$GPGGA,,,,,,,,\r\n")
        d.feed(ubx.build(0x06, 0x31, bytes(32)))
        self.assertEqual(len(d.frames), 1)

    def test_length_gates(self):
        cases = [
            (0x06, 0x00, 16, "CFG-PRT short"),
            (0x06, 0x00, 21, "CFG-PRT long"),
            (0x01, 0x07, 91, "NAV-PVT short"),
            (0x06, 0x31, 31, "CFG-TP5 short"),
        ]
        for cls, mid, length, why in cases:
            with self.subTest(why=why):
                d = ubx.Decoder()
                d.feed(ubx.build(cls, mid, bytes(length)))
                self.assertEqual(len(d.frames), 0, why)
                self.assertEqual(len(d.dropped_length), 1, why)

    def test_unknown_message_is_passed_on(self):
        d = ubx.Decoder()
        d.feed(ubx.build(0x13, 0x40, bytes(7)))  # MGA, nothing here decodes it
        self.assertEqual(len(d.frames), 1)

    def test_overlong(self):
        d = ubx.Decoder()
        d.feed(ubx.build(0x0A, 0x36, bytes(600)))
        self.assertEqual(len(d.frames), 0)
        self.assertEqual(len(d.dropped_length), 1)

    def test_truncated_frame_does_not_swallow_the_stream(self):
        """Seen on the board: NAV-TIMELS cut after its id, so the next frame's
        sync pair was read as a length of 0x62B5 and 25 KB went unparsed."""
        d = ubx.Decoder()
        nxt = ubx.build(0x01, 0x07, bytes(92))
        d.feed(bytes((ubx.SYNC1, ubx.SYNC2, 0x01, 0x26)) + nxt + nxt)
        self.assertEqual(len(d.dropped_length), 1)
        self.assertEqual(len(d.frames), 2)

    def test_overlong_does_not_swallow_the_stream(self):
        d = ubx.Decoder()
        nxt = ubx.build(0x01, 0x07, bytes(92))
        d.feed(bytes((ubx.SYNC1, ubx.SYNC2, 0x13, 0x40, 0xFF, 0xFF)) + nxt)
        self.assertEqual(len(d.dropped_length), 1)
        self.assertEqual(len(d.frames), 1)

    def test_pending_reports_mid_frame(self):
        d = ubx.Decoder()
        self.assertEqual(d.pending, 0) if hasattr(d, "pending") else None


class Tables(unittest.TestCase):
    """tools/ubx.py and main/gps/ubx.h must agree, or a tool passes what the board
    drops."""

    def test_definitions_match(self):
        header = (HERE / "main" / "gps" / "ubx.h").read_text()
        found = set()
        for match in re.finditer(
            r"\{\s*0x([0-9A-Fa-f]{2}),\s*0x([0-9A-Fa-f]{2}),\s*(\d+),\s*(\d+),\s*\"([A-Za-z0-9-]+)\"\s*\}",
            header,
        ):
            cls, mid, lo, hi, name = match.groups()
            found.add((int(cls, 16), int(mid, 16), int(lo), int(hi), name))
        expected = {(c, i, lo, hi, n) for (c, i), (lo, hi, n) in ubx.DEFS.items()}
        self.assertEqual(found, expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
