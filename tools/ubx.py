"""UBX framing, the same rules as main/ubx.cpp.

The tools used to search a buffer for b"\\xb5\\x62" and trust whatever length
followed, then delete up to it -- so one false sync inside a payload derailed
every frame after it, and a frame split across two recv() calls was never seen at
all. This is the incremental state machine instead, and it is deliberately a
direct transcription of the firmware's: the tools exist to test the firmware, so
a second implementation with its own bugs would be worse than none.

Differences from the C++ are only the ones a host makes natural -- frames come
back as objects rather than through a listener, and a command builder is here so
callers stop hand-assembling headers.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC1 = 0xB5
SYNC2 = 0x62
MAX_PAYLOAD = 512

# class, id, minLen, maxLen, name -- the same table as ubx.h, kept in step by
# tools/ubx_test.py, which cross-checks the two.
DEFS = {
    (0x01, 0x07): (92, 92, "NAV-PVT"),
    (0x01, 0x26): (24, 24, "NAV-TIMELS"),
    (0x01, 0x27): (16, 16, "NAV-TIM-TP"),
    (0x01, 0x35): (8, 512, "NAV-SAT"),
    (0x0D, 0x04): (28, 28, "TIM-SVIN"),
    (0x05, 0x00): (2, 2, "ACK-ACK"),
    (0x05, 0x01): (2, 2, "ACK-NAK"),
    (0x06, 0x00): (20, 20, "CFG-PRT"),
    (0x06, 0x01): (3, 8, "CFG-MSG"),
    (0x06, 0x31): (32, 32, "CFG-TP5"),
    (0x06, 0x3D): (28, 28, "CFG-TMODE2"),
    (0x0A, 0x04): (30, 512, "MON-VER"),
    (0x0A, 0x09): (60, 60, "MON-HW"),
    (0x0A, 0x36): (8, 512, "MON-COMMS"),
}


@dataclass(frozen=True)
class Frame:
    cls: int
    id: int
    payload: bytes

    @property
    def len(self) -> int:
        return len(self.payload)

    @property
    def name(self) -> str:
        return DEFS.get((self.cls, self.id), ("", "", "unknown"))[2]

    def u1(self, at: int) -> int:
        return self.payload[at]

    def u2(self, at: int) -> int:
        return struct.unpack_from("<H", self.payload, at)[0]

    def u4(self, at: int) -> int:
        return struct.unpack_from("<I", self.payload, at)[0]

    def i4(self, at: int) -> int:
        return struct.unpack_from("<i", self.payload, at)[0]

    def hex(self) -> str:
        return " ".join(f"{b:02x}" for b in self.payload)


def checksum(body: bytes) -> bytes:
    """Fletcher over class..last payload byte."""
    a = b = 0
    for byte in body:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def build(cls: int, mid: int, payload: bytes = b"") -> bytes:
    """A complete frame, checksum included. A poll is build(cls, id)."""
    body = bytes((cls, mid, len(payload) & 0xFF, len(payload) >> 8)) + payload
    return bytes((SYNC1, SYNC2)) + body + checksum(body)


class Decoder:
    """Feed bytes in; frames and drops come out."""

    IDLE, SYNC2, CLASS, ID, LENLO, LENHI, PAYLOAD, CKA, CKB = range(9)

    def __init__(self) -> None:
        self._state = self.IDLE
        self._cls = self._id = 0
        self._len = 0
        self._buf = bytearray()
        self._cka = self._ckb = 0
        self._got = 0
        self._overlong = False
        self.frames: list[Frame] = []
        self.dropped_checksum: list[Frame] = []
        self.dropped_length: list[Frame] = []

    def feed(self, data: bytes) -> None:
        for byte in data:
            self._byte(byte)

    def _byte(self, byte: int) -> None:
        state = self._state
        if state == self.IDLE:
            if byte == SYNC1:
                self._state = self.SYNC2
        elif state == self.SYNC2:
            if byte == SYNC2:
                self._state = self.CLASS
            elif byte != SYNC1:
                self._state = self.IDLE
        elif state == self.CLASS:
            self._cls = byte
            self._cka = self._ckb = byte
            self._state = self.ID
        elif state == self.ID:
            self._id = byte
            self._add(byte)
            self._state = self.LENLO
        elif state == self.LENLO:
            self._len = byte
            self._add(byte)
            self._state = self.LENHI
        elif state == self.LENHI:
            self._len |= byte << 8
            self._add(byte)
            self._buf = bytearray()
            self._got = 0
            self._overlong = self._len > MAX_PAYLOAD
            self._state = self.CKA if self._len == 0 else self.PAYLOAD
        elif state == self.PAYLOAD:
            if not self._overlong:
                self._buf.append(byte)
            self._got += 1
            self._add(byte)
            if self._got >= self._len:
                self._state = self.CKA
        elif state == self.CKA:
            if byte != self._cka:
                self.dropped_checksum.append(Frame(self._cls, self._id, bytes(self._buf)))
                self._state = self.IDLE
            else:
                self._state = self.CKB
        elif state == self.CKB:
            frame = Frame(self._cls, self._id, bytes(self._buf))
            if byte != self._ckb:
                self.dropped_checksum.append(frame)
            else:
                self._emit(frame)
            self._state = self.IDLE

    def _add(self, byte: int) -> None:
        self._cka = (self._cka + byte) & 0xFF
        self._ckb = (self._ckb + self._cka) & 0xFF

    def _emit(self, frame: Frame) -> None:
        if self._overlong:
            self.dropped_length.append(frame)
            return
        limit = DEFS.get((frame.cls, frame.id))
        if limit is not None and not (limit[0] <= frame.len <= limit[1]):
            self.dropped_length.append(frame)
            return
        self.frames.append(frame)


def read_frames(sock, seconds: float, chunk: int = 4096) -> Decoder:
    """Drain a socket for a while and return the decoder holding what arrived."""
    import time

    dec = Decoder()
    end = time.time() + seconds
    while time.time() < end:
        try:
            data = sock.recv(chunk)
        except (BlockingIOError, TimeoutError):
            time.sleep(0.02)
            continue
        if not data:
            break
        dec.feed(data)
    return dec
