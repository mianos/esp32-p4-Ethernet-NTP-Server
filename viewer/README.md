# esp32p4-ntp sky view

A local page for the P4 timing box: satellite sky plot, receiver health, signal
and framing counters. It reads the box's own JSON and draws it; it does not
configure anything.

It lives here rather than in its own repository because it reads this
firmware's JSON: a field renamed in `web_server.cpp` breaks it, and the two
are easier kept in step side by side.

    ./setup.sh          # once
    ./run.sh            # http://127.0.0.1:8765/

## Why a server and not a page that fetches the box directly

The box's httpd has a single worker. A browser hitting it per tab, per refresh,
means the poll rate depends on how many tabs are open, and the plot changes
shape with load. Here one process polls on a fixed 2 s timer and every tab reads
the same snapshot, so the cost against the box is constant and the page renders
at a steady rate.

The last good answer is kept and its age shown when a poll fails, so a dropped
poll dims a label instead of blanking the page.

## What it shows, and what it refuses to

Positions come from `/gps/sats` (azimuth and elevation per satellite). Radius is
C/N0, angle is azimuth with north up, and the rings are 30° of elevation. In-use
satellites are ringed in white, unhealthy ones dashed.

Colour is switchable. **C/N0** is signal strength. **Residual** is the receiver's
pseudorange residual, which is multipath itself rather than a proxy for it --
they disagree exactly where it matters, because a strong reflection is loud and
wrong. The `Res` column carries the same number in centimetres.

Before there is a fix the receiver sends elevation `-91` with azimuth `0`. That
is a sentinel, not a position, so rather than stacking every satellite on the
horizon at due north the plot says "geometry pending" and lists the rows
without positions.

## Beyond signal

- **Health** — fix, time validity, PPS age and edge count, stratum, frames. A 3D
  fix with invalid time is the state that quietly serves rubbish, so it gets its
  own cell rather than being folded into "synced".
- **Responder** — answered, per-minute rate, KoD, dropped, distinct clients,
  last-served age, leap, root delay and dispersion. A rate-limited client and
  one that stopped asking both leave a request count flat; these separate them.
- **Receiver** — offset, rate, jitter, tAcc (the receiver's own accuracy
  estimate), root delay, dispersion, servo steps, rate clamps and edge spikes.
  Steps and clamps show a servo fighting the oscillator; a mean offset hides it.

## Endpoints

| path | |
|---|---|
| `GET /` | the page |
| `GET /api/snapshot` | everything the page draws, merged and sanitised |
| `GET /api/history` | last 5 minutes of C/N0, noise, AGC and offset |
| `GET /api/sats.csv` | current satellites as CSV |
| `GET /healthz` | this viewer's own health, 503 when the box is stale |

## Environment

| variable | default |
|---|---|
| `BOX_HOST` | `esp32p4-ntp` |
| `BOX_PORT` | `80` |
| `POLL_S` | `2` |
| `PORT` | `8765` (listener) |
| `BIND` | `127.0.0.1` |
| `DB` | `samples.db` beside the code; empty disables writing |
| `RECORD_S` | `1` — floor on the interval between rows |
| `RETENTION_D` | `90` days |

## Why nc and not a socket library

On this host, python and curl both fail to reach a LAN address with
`EHOSTUNREACH` while `/usr/bin/nc` connects fine — the two back to back, one
always failing and the other always working. It is the binaries, not the
network, so the poller shells out to `nc` and the viewer keeps working.

## Long-term samples, and adev.py

The sparkline keeps 300 s, which is what the firmware's own 300 s fit is over
and therefore the most the live page can honestly draw. Anything longer needs
storage, so the poller also appends a row per poll to SQLite (default 2 s):
`rate_ppm`,
`offset_us`, `dispersion_us`, `timtp_flags`, fix, satellites and the last
holdover. A failed poll writes a **gap row** rather than nothing, because
interpolating across an outage invents stability at the longest averaging
times, which is where nobody can check it by eye.

`adev.py` reads that file and prints overlapping Allan deviation. Frequency by
default, since the 1 µs stamp makes phase ADEV a picture of the timer below a
minute; `--phase` gives the phase form for comparison. Gaps split the series
into runs and no window spans a hole.

```
./adev.py --db samples.db --max-tau 86400
./adev.py --db samples.db --phase --max-tau 600
```

Two things the numbers need beside them: with the servo running this measures
the loop, not the crystal, and tau under 300 s is inside the fit window, so it
reports smoothing rather than the oscillator.

## Notes

Polling is read-only: `/healthz`, `/gps/info`, `/gps/sats`, none of which write
settings or touch the receiver. Safe to leave running while a survey is going.
