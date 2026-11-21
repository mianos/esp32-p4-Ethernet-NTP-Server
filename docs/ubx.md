# UBX framing

The receiver's line and the u-center bridge both carry this, and both feed one
decoder (`main/gps/ubx.cpp`). This note is what that code assumes, and why.

## A frame

```
B5 62  cls  id  lenLo lenHi  payload…  ckA ckB
```

`len` is little-endian. The checksum is the 8-bit Fletcher algorithm over
`cls`..the last payload byte — it starts at the class byte, so `ckA`/`ckB` are
seeded with `cls` and then accumulated.

## The gate

**A frame that fails its checksum is dropped, never repaired.** A lost or
invented byte slides every field by one; without the gate the firmware anchors
the clock to a second that never happened and looks healthy doing it. A frame
that fails is counted (`gps_dropped_crc` at `/healthz`), and the first few are
logged with their class and id so "the line is corrupt" can be told from "the
receiver is quiet" — the two are otherwise indistinguishable.

The same applies to length: a message outside the range in the definition table
is dropped before any handler can read past its end (`gps_dropped_len`), and the
first few are logged.

## State

```
Idle --B5--> Sync2 --62--> Class --id--> Id --lo--> LenLo --hi--> LenHi
LenHi --len==0--> CkA
LenHi --len>0--> Payload --n bytes--> CkA
CkA --match--> CkB --match--> frame, to Idle
CkA/CkB --mismatch--> drop, to Idle
Sync2 --neither B5 nor 62--> Idle
Sync2 --B5--> Sync2
```

`Sync2` holding on a repeated `B5` matters: `B5 B5 62` is a frame with a byte of
noise in front of it. NMEA is off at this receiver but a boot burst or a u-center
session can still put anything on the line, so the state has to sit through
garbage without inventing frames.

Everything needed between calls is a member, so a frame split at any offset —
between the sync bytes, across the length, halfway through the payload — is
assembled identically to one that arrives whole. That is the property the old
`B5 62 xx yy` scan lacked: it looked at one `recv()` at a time, so a frame
straddling a read was invisible, and it re-read the same bytes once per frame.

## Definitions

`ubx.h` carries a table of `{class, id, minLen, maxLen, name}` for the messages
this firmware decodes, and the decoder enforces it. Handlers read named fields
(`f.u4(8)`) rather than open-coding offsets with a length guard apiece, so "is
this long enough for the field I am about to read" is answered once, in the
table, instead of in a guard somebody has to keep in step with the offsets
below it.

Messages outside the table are delivered ungated. The gate exists to protect the
readers of specific fields, not to police the protocol, and u-center sends plenty
this firmware has no opinion about.

## Buffer

`kMaxPayload` is 512. A frame claiming more, or a known message claiming a
length outside its definition, is dropped the moment the length is read
(`gps_dropped_len`) and the two length bytes go back through the sync hunt.
It used to be walked to its end, on the grounds that one over-long message
should cost a frame and not the stream -- but the length is sixteen bits that
nothing has checked yet. A frame cut short on the line runs into the next
frame's sync pair, `B5 62` reads as 25269, and the decoder sat through 25 KB of
good stream: five minutes with no frames, a baud scan, and a reset of a receiver
that had never stopped. NAV-SAT and TIM-SVIN are the largest decoded; both are
far below the limit.

## The bridge

The bridge decodes both directions separately, because the client's session and
the receiver's message set are two streams that merely cross one socket. What it
looks for:

- a client `CFG-PRT` for port 1 → arm a rate change, wait for the ACK, drain the
  transmitter, then switch. A NAK cancels. See the comment in `bridgeTask` for
  why an ACK that never arrives still switches.
- a client `MON-COMMS` poll → the receiver ignores this message, so the bridge
  answers it after 400 ms of silence (u-center opens the port by polling it).
- a receiver `MON-COMMS` reply → the module answered after all; stand down.

## Silence

Five seconds without a frame sets the baud scan going again
(`gps_rescans` at `/healthz`). The decoder is reset and the rate that was
working keeps its full fifteen seconds before the scan moves off it. A u-center session holds the port and must not have
its rate moved underneath it, so the bridge suppresses the rescan while attached.

## Tests

`tools/ubx_test.py` covers the cases that broke the search: every split offset, a
sync pair inside a payload, a corrupt byte in the header or the payload, a
truncated frame, an over-long one, a frame cut short ahead of the next one's
sync, and the length gates. `tools/ubx.py` is the
host twin the tests run against and the tools now use; a table-parity test keeps
it and `main/gps/ubx.h` in step.

## Attribution

Field offsets and lengths are transcribed from the u-blox 8 / M8 receiver
description, with the TIM 1.10 dialect for `NAV-TIMELS` and `TIM-SVIN`. The layout
of the definition table follows [pyubx2](https://github.com/semuconsulting/pyubx2)
(BSD-3-Clause, © semuadmin); the framing follows [gpsd](https://gitlab.com/gpsd/gpsd)'s
`driver_ubx.c` (BSD-2-Clause). Neither is vendored — the tables here are
hand-written for the thirteen messages used — but both were read.
