"""Sky view and health for the esp32p4-ntp box.

The box serves plain JSON on three endpoints; this puts a page over them and
keeps the polling on the server side. The browser talks only to this process,
so a tab refresh, or five tabs, costs the box the same single poll -- and the
page never has to care about CORS, or about what to show while a request is in
the air.

Polling is deliberately dumb and constant-rate: a 2 s interval against a board
that answers in a few milliseconds, with the last good answer kept and its age
shown when a poll fails. The alternative -- a poll per request -- makes the
display change shape depending on how busy it is.

Run it with ./run.sh, then open http://127.0.0.1:8765/.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import os
import statistics
import sqlite3
import time
from collections import deque
from dataclasses import dataclass, field
from collections.abc import AsyncIterator
from typing import Any

import adev as analysis
import httpx
from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse

HERE = os.path.dirname(os.path.abspath(__file__))

BOX_HOST = os.environ.get("BOX_HOST", "esp32p4-ntp")
BOX_PORT = int(os.environ.get("BOX_PORT", "80"))
POLL_S = float(os.environ.get("POLL_S", "2"))
HISTORY_S = 300  # seconds of sparkline, ~150 samples at the default interval
RECENT_S = 300  # window over which a cumulative fault counter is judged
MIN_BASELINE_S = 30  # below this much history, claim nothing

# Long-term recording, which the sparkline's 300 s cannot serve. sqlite because
# the geometry here is one writer, one reader later, and no server to run.
# RECORD_S is a floor on the interval between rows, not a scheduler: the poller
# already ticks at POLL_S and a row per poll is what the statistics want. A day
# is 86 400 rows and about 5 MB, so RETENTION_D can be generous.
DB_PATH = os.environ.get("DB", os.path.join(HERE, "samples.db"))
RECORD_S = float(os.environ.get("RECORD_S", "1"))
RETENTION_D = int(os.environ.get("RETENTION_D", "90"))
PRUNE_S = 3600.0
# Long-span transport: never more than this many buckets in one reply, and a
# reply is reused for LONG_CACHE_S rather than recomputed per view.
LONG_BUCKETS = 600
LONG_CACHE_S = 30.0
LONG_SPANS = (300, 3600, 21600, 86400, 604800)

# gnssId as the receiver numbers it. 255 is the receiver's "no slot" value,
# which shows up on the GLONASS rows here.
GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 4: "IMES", 5: "QZSS", 6: "GLONASS", 7: "NavIC"}

# UBX-NAV-SAT flags, only the bits that change what the page says.
FLAG_USED = 1 << 3
FLAG_HEALTHY = 1 << 4
FLAG_EPH = 1 << 8


class Recorder:
    """Append-only sample log, with the gaps written down as gaps.

    The point of a gap row is that a failed poll is not the same as a clock
    that did not move. Interpolating across an outage makes a clean-looking
    series whose longest averaging times are fiction, so the writer says so and
    the reader is left to drop the run.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        self.db: sqlite3.Connection | None = None
        self.last_t = 0.0
        self.next_prune = time.time() + PRUNE_S

    def open(self) -> None:
        if not self.path:
            return
        self.db = sqlite3.connect(self.path, isolation_level=None)
        # WAL so the analysis can read while this keeps writing; NORMAL sync is
        # enough because losing the last second of a stability series to a power
        # cut costs one sample, not a file.
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=NORMAL")
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS samples ("
            " t REAL PRIMARY KEY, gap INTEGER NOT NULL,"
            " rate_ppm REAL, offset_us REAL, dispersion_us REAL,"
            " flags INTEGER, fix INTEGER, sats INTEGER, holdover_s REAL)"
        )

    def _insert(self, row: tuple[Any, ...]) -> None:
        if self.db is None:
            return
        self.db.execute("INSERT OR REPLACE INTO samples VALUES (?,?,?,?,?,?,?,?,?)", row)
        if time.time() >= self.next_prune:
            self.next_prune = time.time() + PRUNE_S
            self.db.execute("DELETE FROM samples WHERE t < ?", (time.time() - RETENTION_D * 86400.0,))

    def sample(self, info: dict[str, Any], health: dict[str, Any]) -> None:
        now = time.time()
        if now - self.last_t < RECORD_S:
            return
        self.last_t = now
        self._insert((
            now, 0, num(info.get("rate_ppm")), num(info.get("offset_us")),
            num(info.get("dispersion_us")), num(info.get("timtp_flags")),
            num(health.get("gps_fix")), num(health.get("gps_sats")),
            num(info.get("last_holdover_s")),
        ))

    def gap(self) -> None:
        """A failed poll, or the box not answering."""
        now = time.time()
        if now - self.last_t < RECORD_S:
            return
        self.last_t = now
        self._insert((now, 1, None, None, None, None, None, None, None))

    def close(self) -> None:
        if self.db is not None:
            self.db.close()
            self.db = None


@dataclass
class Poller:
    """One shared snapshot, refreshed on a timer, never on demand."""

    doc: dict[str, Any] = field(default_factory=dict)
    history: deque[dict[str, Any]] = field(
        default_factory=lambda: deque(maxlen=max(30, int(HISTORY_S / max(0.5, POLL_S))))
    )
    last_ok: float = 0.0
    last_try: float = 0.0
    error: str | None = "not polled yet"
    polls: int = 0
    # Serialised once per poll, not per request: the cost of a snapshot then
    # depends on the poll rate rather than on how many clients are watching.
    body: bytes = b'{}'
    etag: str = '"0"'
    built: int = 0
    failures: int = 0
    consecutive_failures: int = 0
    task: asyncio.Task[None] | None = None
    recorder: Recorder = field(default_factory=lambda: Recorder(DB_PATH))

    async def _get(self, path: str) -> dict[str, Any]:
        """GET through httpx.

        This used to shell out to nc: on the Mac python and curl both failed to
        reach a LAN address with EHOSTUNREACH while nc worked, so nc was the
        only thing that ran. On a container that quirk is gone, and every poll
        was paying for a fork, so the client library does it now. The failure is
        still a RuntimeError so the caller and the page need not know.
        """
        try:
            reply = await CLIENT.get(f"http://{BOX_HOST}:{BOX_PORT}{path}")
        except httpx.HTTPError as exc:
            raise RuntimeError(f"{path}: {type(exc).__name__}") from exc
        if reply.status_code != 200:
            raise RuntimeError(f"{path}: {reply.status_code}")
        try:
            decoded = reply.json()
        except ValueError as exc:
            raise RuntimeError(f"{path}: not JSON") from exc
        if not isinstance(decoded, dict):
            raise RuntimeError(f"{path}: not an object")
        return decoded

    async def once(self) -> None:
        self.polls += 1
        self.last_try = time.time()
        try:
            # Concurrent: the three are independent, and the box's httpd has a
            # single worker, so serialising them only adds latency.
            health, info, sats = await asyncio.gather(
                self._get("/healthz"), self._get("/gps/info"), self._get("/gps/sats")
            )
        except RuntimeError as exc:
            self.failures += 1
            self.consecutive_failures += 1
            self.error = str(exc)
            self.recorder.gap()
            # Rebuilt on failure too, so a client that arrives mid-outage gets
            # the stale-but-honest view rather than the last good one silently.
            self.rebuild()
            return

        self.consecutive_failures = 0
        self.error = None
        self.last_ok = time.time()
        self.doc = {"health": health, "info": info, "sats": sats}
        self.history.append(
            {
                "t": self.last_ok,
                "cn0_mean": num(sats.get("mean")),
                "noise_dbm": num(info.get("noise_dbm")),
                "agc": num(info.get("agc")),
                "offset_ns": num(info.get("offset_ns")),
                "fix": num(health.get("gps_fix")),
                "used": num(sats.get("used")),
                # Cumulative since boot, so the delta over the window is what
                # says whether a fault is still happening.
                "crc": num(health.get("gps_dropped_crc")),
                "spikes": num(info.get("edge_spikes")),
                "clamps": num(info.get("servo_clamps")),
                "rescans": num(health.get("gps_rescans")),
                "io_rx": num(info.get("io_rx_bytes")),
                "io_tx": num(info.get("io_tx_bytes")),
                "skipped": num(info.get("msg_skipped")),
                "ubx": num(info.get("msg_ubx")),
                "adev1s": num(info.get("adev_1s")),
            }
        )
        self.recorder.sample(info, health)
        # After the append: build() reads the history this poll just added.
        self.rebuild()

    def rebuild(self) -> None:
        """Serialise the snapshot once. Called from the poller only."""
        payload = build(self)
        self.body = json.dumps(payload, separators=(",", ":")).encode()
        self.built += 1
        # The poll identity, not a hash of the bytes: two polls with identical
        # content (a quiet box) still represent different observations, and a
        # client should get its age updated.
        self.etag = f'"{payload.get("polled_at") or 0}-{self.built}"'

    def recent_activity(self) -> dict[str, Any]:
        """How much the cumulative fault counters have moved lately.

        Those counters only ever climb, so a single CRC error at boot would
        otherwise keep a boot-long "warnings" badge lit with nothing behind it.
        The window is whatever history exists, up to RECENT_S; with less than
        MIN_BASELINE_S of it every field is None and nothing is claimed either
        way, because "no errors in 3 seconds" is not a clean bill of health.
        """
        now = time.time()
        if not self.history:
            return {"window_s": None, "crc": None, "spikes": None, "clamps": None,
                    "rescans": None, "io_rx": None, "io_tx": None, "skipped": None, "ubx": None}
        keys = ("crc", "spikes", "clamps", "rescans", "io_rx", "io_tx", "skipped", "ubx")
        activity: dict[str, Any] = {"window_s": None}
        activity.update({k: None for k in keys})

        # A baseline per key, not one for the whole snapshot: after a reboot the
        # oldest in-window sample predates fields that have since appeared, and a
        # shared baseline would refuse to report anything for five minutes.
        current = self.history[-1]
        window = 0.0
        for key in keys:
            base = None
            for sample in self.history:
                if now - sample["t"] <= RECENT_S and sample.get(key) is not None:
                    base = sample
                    break
            if base is None or current.get(key) is None:
                continue
            span = now - base["t"]
            if span < MIN_BASELINE_S:
                continue
            activity[key] = max(0, int(current[key] - base[key]))
            window = max(window, span)
        if window == 0.0:
            return activity
        activity["window_s"] = round(window)
        return activity
        window = now - baseline["t"]
        if window < MIN_BASELINE_S:
            return activity

        activity["window_s"] = round(window)
        current = self.history[-1]
        for key in ("crc", "spikes", "clamps", "rescans", "io_rx", "io_tx", "skipped", "ubx"):
            if current.get(key) is not None and baseline.get(key) is not None:
                activity[key] = max(0, int(current[key] - baseline[key]))
        return activity

    async def loop(self) -> None:
        while True:
            started = time.monotonic()
            with contextlib.suppress(RuntimeError):
                await self.once()
            await asyncio.sleep(max(0.0, POLL_S - (time.monotonic() - started)))


def num(value: Any) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def integer(value: Any) -> int | None:
    got = num(value)
    return None if got is None else int(got)


def text(value: Any, limit: int = 200) -> str | None:
    if not isinstance(value, str):
        return None
    return value[:limit]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clean_sat(raw: Any) -> dict[str, Any] | None:
    """One satellite row, with the receiver's sentinels resolved.

    Elevation -91 is what NAV-SAT carries before there is a fix: the field is
    an int8 and the receiver leaves the azimuth at 0 too. It is not a real
    geometry, so the row gets no sky position and the page says so, rather than
    stacking every satellite on the horizon at due north.
    """
    if not isinstance(raw, dict):
        return None

    gnss = integer(raw.get("gnss"))
    sv = integer(raw.get("sv"))
    if gnss is None or sv is None:
        return None

    flags = integer(raw.get("flags")) or 0
    elevation = num(raw.get("elev"))
    azimuth = num(raw.get("azim"))

    has_geometry = elevation is not None and elevation >= -90.0 and azimuth is not None
    cno = num(raw.get("cno"))

    return {
        "gnss": gnss,
        "constellation": GNSS.get(gnss, f"gnss {gnss}"),
        "sv": sv,
        "cno": None if cno is None else round(clamp(cno, 0, 99), 1),
        "elev": None if not has_geometry else round(clamp(elevation, 0, 90), 1),
        "azim": None if not has_geometry else round(clamp(azimuth, 0, 359.9), 1),
        "has_geometry": has_geometry,
        "prres_cm": num(raw.get("prres_cm")),
        "used": bool(flags & FLAG_USED),
        "healthy": bool(flags & FLAG_HEALTHY),
        "ephemeris": bool(flags & FLAG_EPH),
        "quality": flags & 0x07,
    }


def verdict(doc: dict[str, Any], healthy: bool, recent: dict[str, Any]) -> tuple[str, str, list[str]]:
    """The one-line answer, and the reasons behind it.

    State notes come from the current fields; fault counts come from `recent`
    deltas, not the boot totals, so a transient at start-up stops being an
    accusation once it stops happening.
    """
    health = doc.get("health", {})
    info = doc.get("info", {})
    sats_doc = doc.get("sats", {})
    notes: list[str] = []

    if not health.get("link_up"):
        notes.append("link down")
    if health.get("time_set") is False:
        notes.append("clock not set")
    if health.get("gps_time_valid") is False:
        notes.append("time not valid")
    if info.get("ant_short"):
        notes.append("antenna short")
    if info.get("ant") not in (None, "ok"):
        notes.append(f"antenna {info.get('ant')}")
    if info.get("jam") not in (None, "unknown", "ok"):
        notes.append(f"jamming {info.get('jam')}")
    if sats_doc.get("age_ms") is not None and num(sats_doc["age_ms"]) > 10000:
        notes.append("satellite data stale")

    baud = num(info.get("baud")) or 9600.0
    window = recent.get("window_s")
    if window and recent.get("io_tx") is not None:
        # Ten bits on the wire per byte, 8N1. Warn only well into the range
        # where a NAV-SAT burst starts queueing behind other traffic.
        util = (recent["io_tx"] * 10.0) / (window * max(1200.0, baud))
        if util > 0.75:
            notes.append(f"link at {util * 100:.0f}%")
    wire = sum(recent.get(k) or 0 for k in ("io_parity", "io_framing", "io_overrun"))
    if wire:
        notes.append(f"{wire} wire errors")
    if (num(info.get("rbuf_peak")) or 0) > 0:
        notes.append(f"receive backlog peaked {integer(info['rbuf_peak'])} B")

    if window:
        for key, label in (("crc", "bad frames"), ("spikes", "edge spikes"),
                           ("clamps", "rate clamps"), ("rescans", "line rescans")):
            if recent.get(key):
                notes.append(f"{recent[key]} {label} in {window} s")

    fix = integer(health.get("gps_fix")) or 0
    if not healthy:
        return "bad", "Box unreachable", notes
    if not health.get("ntp_synced"):
        if fix >= 2:
            return "warn", "Fix, but PPS not locked", notes
        return "warn", "No fix", notes
    if notes:
        return "warn", "Locked, with warnings", notes
    return "good", "Locked", notes


def build(poller: Poller) -> dict[str, Any]:
    # `have_data` is "there is something to draw"; `ok` is "the box answered just
    # now". They differ while a poll is failing: the numbers are then real but
    # old, so the page keeps them, dims them, and says how old they are rather
    # than blanking out.
    if not poller.doc:
        return {
            "ok": False,
            "have_data": False,
            "error": poller.error,
            "box": BOX_HOST,
            "polled_at": poller.last_ok or None,
            "polls": poller.polls,
            "failures": poller.failures,
            "consecutive_failures": poller.consecutive_failures,
            "verdict": {"state": "bad", "headline": "Box unreachable", "notes": [poller.error or "no response"]},
        }

    health = poller.doc["health"]
    info = poller.doc["info"]
    sats_doc = poller.doc["sats"]

    raw_sats = sats_doc.get("sats")
    sats = [row for row in (clean_sat(s) for s in raw_sats or []) if row is not None]
    sats.sort(key=lambda s: (-(s["cno"] or 0), s["constellation"], s["sv"]))

    healthy = poller.error is None
    recent = poller.recent_activity()
    state, headline, notes = verdict(poller.doc, healthy, recent)
    if poller.error is not None:
        # While the box is unreachable the per-field notes are read off old data
        # and would restate a problem that may already be gone; only the poll
        # failure is current.
        notes = [poller.error]

    by_constellation: dict[str, dict[str, int]] = {}
    for sat in sats:
        bucket = by_constellation.setdefault(sat["constellation"], {"seen": 0, "used": 0})
        bucket["seen"] += 1
        bucket["used"] += 1 if sat["used"] else 0

    pps_age_ms = num(health.get("gps_pps_age_ms"))
    uptime = integer(health.get("uptime"))
    target_s = num(info.get("svin_target_s"))
    elapsed_s = num(info.get("svin_dur_s"))
    limit_mm = num(info.get("svin_limit_mm"))
    acc_mm = num(info.get("svin_acc_mm"))

    return {
        "ok": poller.error is None,
        "have_data": True,
        "box": BOX_HOST,
        "polled_at": poller.last_ok,
        "age_s": None if not poller.last_ok else round(time.time() - poller.last_ok, 1),
        "stale": poller.consecutive_failures > 0,
        "error": poller.error,
        "consecutive_failures": poller.consecutive_failures,
        "polls": poller.polls,
        "failures": poller.failures,
        "verdict": {"state": state, "headline": headline, "notes": notes},
        "device": {
            "hostname": text(health.get("hostname")),
            "ip": text(health.get("ip")),
            "version": text(health.get("version")),
            "partition": text(health.get("partition")),
            "reset_reason": text(health.get("reset_reason")),
            "boots": integer(health.get("boots")),
            "uptime_s": uptime,
            "heap_free": integer(health.get("heap_free")),
            "link_up": health.get("link_up"),
        },
        "time": {
            "synced": health.get("ntp_synced"),
            "source": text(health.get("ntp_source")),
            "fix": integer(health.get("gps_fix")),
            "stratum": integer(health.get("ntp_stratum")),
            "refid": text(health.get("ntp_refid")),
            "pps_age_ms": pps_age_ms,
            "pps_count": integer(health.get("gps_pps_count")),
            "time_valid": health.get("gps_time_valid"),
            "anchored": health.get("gps_anchored"),
            "time": text(health.get("time")),
        },
        "ntp": {
            "serving": health.get("serve_ntp"),
            "leap": integer(health.get("ntp_leap")),
            "root_delay_us": num(health.get("ntp_root_delay_us")),
            "root_dispersion_us": num(health.get("ntp_root_dispersion_us")),
            "answered": integer(health.get("ntp_answered")),
            "kod": integer(health.get("ntp_kod")),
            "dropped": integer(health.get("ntp_dropped")),
            "per_minute": num(health.get("ntp_rate_per_min")),
            "clients": integer(health.get("ntp_clients")),
            "last_served_s": num(health.get("ntp_last_served_s")),
        },
        "receiver": {
            "module": text(info.get("module"), 300),
            "gnss": text(info.get("gnss")),
            "gnss_cfg": text(info.get("gnss_cfg")),
            "time_mode": text(info.get("time_mode")),
            "tp5_ok": info.get("tp5_ok"),
            "timtp": text(info.get("timtp")),
            "offset_ns": integer(info.get("offset_ns")),
            "rate_ppm": num(info.get("rate_ppm")),
            "dispersion_us": num(info.get("dispersion_us")),
            "lock_window_s": num(info.get("lock_window_s")),
            "tp_jitter_ns": num(info.get("tp_jitter_ns")),
            "root_delay_ns": num(info.get("root_delay_ns")),
            "servo_steps": integer(info.get("servo_steps")),
            "servo_clamps": integer(info.get("servo_clamps")),
            "edge_spikes": integer(info.get("edge_spikes")),
            "gdop": num(info.get("gdop")),
            "pdop": num(info.get("pdop")),
            "hdop": num(info.get("hdop")),
            "vdop": num(info.get("vdop")),
            "dop_age_s": num(info.get("dop_age_s")),
            "clk_bias_ns": num(info.get("clk_bias_ns")),
            "clk_drift_ns_s": num(info.get("clk_drift_ns_s")),
            "t_acc_ns": num(info.get("t_acc_ns")),
            "f_acc_ps_s": num(info.get("f_acc_ps_s")),
            "ttff_ms": num(info.get("ttff_ms")),
            "rf_bands": num(info.get("rf_bands")),
            "rf_jam_worst": num(info.get("rf_jam_worst")),
            "rf_agc_min": num(info.get("rf_agc_min")),
            "baud": num(info.get("baud")),
            "io_parity": num(info.get("io_parity")),
            "io_framing": num(info.get("io_framing")),
            "io_overrun": num(info.get("io_overrun")),
            "rbuf_pending": num(info.get("rbuf_pending")),
            "rbuf_peak": num(info.get("rbuf_peak")),
            "tbuf_pending": num(info.get("tbuf_pending")),
            "tbuf_peak": num(info.get("tbuf_peak")),
            "tbuf_errors": num(info.get("tbuf_errors")),
            "rf_cfg": text(info.get("rf_cfg")),
            "bridge_active": info.get("bridge_active"),
            "bridge_clients": integer(info.get("bridge_clients")),
            "recent_frames": text(info.get("recent_frames"), 200),
            "nav_status_flags": integer(info.get("nav_status_flags")),
            "nav_status_flags2": integer(info.get("nav_status_flags2")),
            "f_acc_ps_s": num(info.get("f_acc_ps_s")),
            "leap_pending": info.get("leap_pending"),
            "leap_in_s": num(info.get("leap_in_s")),
            "last_holdover_s": num(info.get("last_holdover_s")),
            "last_holdover_error_us": num(info.get("last_holdover_error_us")),
            "msgpp_hex": text(info.get("msgpp_hex"), 300),
            "tie_max_ns": num(info.get("tie_max_ns")),
            "tie_rms_ns": num(info.get("tie_rms_ns")),
            "io_hex": text(info.get("io_hex"), 300),
        },
        "stability": {
            # The firmware publishes ppb per octave and withholds the whole set
            # until it has 60 samples, so a missing key is "not yet", not zero.
            "adev_n": integer(info.get("adev_n")),
            "adev": [num(info.get(f"adev_{1 << k}s")) for k in range(7)],
        },
        "signal": {
            "ant": text(info.get("ant")),
            "ant_power": text(info.get("ant_power")),
            "ant_bias": text(info.get("ant_bias")),
            "ant_short": info.get("ant_short"),
            "ant_delay_ns": num(info.get("ant_delay_ns")),
            "noise_dbm": num(info.get("noise_dbm")),
            "agc": num(info.get("agc")),
            "jam": text(info.get("jam")),
            "jam_ind": num(info.get("jam_ind")),
            "cn0_mean": num(sats_doc.get("mean")),
            "cn0_max": num(sats_doc.get("max")),
            "cn0_used": integer(sats_doc.get("used")),
            "sats_age_ms": num(sats_doc.get("age_ms")),
        },
        "survey": {
            "running": info.get("time_mode") == "survey" and not info.get("svin_valid"),
            "valid": info.get("svin_valid"),
            "elapsed_s": elapsed_s,
            "target_s": target_s,
            "progress": None
            if not target_s or elapsed_s is None
            else round(clamp(elapsed_s / target_s, 0.0, 1.0), 4),
            "observations": integer(info.get("svin_obs")),
            "accuracy_mm": acc_mm,
            "limit_mm": limit_mm,
            "variance_mm2": num(info.get("svin_var_mm2")),
        },
        "sats": sats,
        "geometry_ready": any(sat["has_geometry"] for sat in sats),
        "residual_ready": any(sat["prres_cm"] is not None for sat in sats),
        "constellations": by_constellation,
        "recent": recent,
        "link": {
            "baud": num(info.get("baud")),
            "tx_bps": None if not recent.get("window_s") or recent.get("io_tx") is None
            else round(recent["io_tx"] / recent["window_s"], 1),
            "rx_bps": None if not recent.get("window_s") or recent.get("io_rx") is None
            else round(recent["io_rx"] / recent["window_s"], 1),
            "util_pct": None
            if not recent.get("window_s") or recent.get("io_tx") is None
            else round(100.0 * recent["io_tx"] * 10.0 / (recent["window_s"] * max(1200.0, num(info.get("baud")) or 9600.0)), 1),
            "skipped_total": integer(info.get("msg_skipped")),
            "skipped_delta": (recent or {}).get("skipped"),
            "ubx_total": integer(info.get("msg_ubx")),
            "parity": integer(info.get("io_parity")),
            "framing": integer(info.get("io_framing")),
            "overrun": integer(info.get("io_overrun")),
            "rbuf_peak": integer(info.get("rbuf_peak")),
            "tbuf_peak": integer(info.get("tbuf_peak")),
            "tbuf_errors": integer(info.get("tbuf_errors")),
        },
        "frames": {
            "good": integer(health.get("gps_frames")),
            "dropped_crc": integer(health.get("gps_dropped_crc")),
            "dropped_len": integer(health.get("gps_dropped_len")),
            "rescans": integer(health.get("gps_rescans")),
        },
    }


poller: Poller
CLIENT = httpx.AsyncClient(timeout=8.0)
LONG_CACHE: dict[int, tuple[float, dict[str, Any]]] = {}


@contextlib.asynccontextmanager
async def lifespan(_: FastAPI) -> AsyncIterator[None]:
    """Start the one poller, and stop it on the way out."""
    global poller
    if not BOX_HOST:
        raise SystemExit("BOX_HOST is empty")
    poller = Poller()
    poller.recorder.open()
    poller.task = asyncio.create_task(poller.loop())
    try:
        yield
    finally:
        poller.recorder.close()
        await CLIENT.aclose()
        if poller.task:
            poller.task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await poller.task



app = FastAPI(title="esp32p4-ntp sky view", docs_url=None, redoc_url=None, lifespan=lifespan)


PAGE = os.path.join(HERE, "static", "index.html")


def not_modified(request: Request, etag: str) -> bool:
    """True when the caller already holds this exact version."""
    return etag in [tag.strip() for tag in request.headers.get("if-none-match", "").split(",")]


@app.get("/")
async def index(request: Request) -> Response:
    """The page, cached by the browser but always revalidated.

    It never changes while the process runs, so it is loaded once and served
    with an ETag: a refresh is a 304 instead of a file read, and clients do not
    stampede it.
    """
    stat = os.stat(PAGE)
    etag = f'"{stat.st_mtime_ns:x}-{stat.st_size:x}"'
    if not_modified(request, etag):
        return Response(status_code=304, headers={"etag": etag})
    with open(PAGE, "rb") as handle:
        body = handle.read()
    return Response(body, media_type="text/html",
                    headers={"etag": etag, "cache-control": "no-cache"})


@app.get("/api/snapshot")
async def snapshot(request: Request) -> Response:
    """The cached snapshot, serialised once per poll.

    Revalidation is the point: with several clients each polling at 2 s, the
    build and the bytes are paid once per poll and every other request is a
    cheap 304. Cache-Control is no-cache (revalidate), never no-store, or the
    clients would re-download unchanged bodies.
    """
    if not_modified(request, poller.etag):
        return Response(status_code=304, headers={"etag": poller.etag, "cache-control": "no-cache"})
    return Response(poller.body, media_type="application/json",
                    headers={"etag": poller.etag, "cache-control": "no-cache"})


def build_long(span: int) -> dict[str, Any]:
    """Bucket the log and take an ADEV over it, for one span.

    Buckets carry min and max as well as the mean, because a decimator that
    only averages hides exactly the spikes the page exists to show. The ADEV is
    computed from the raw rows, not the buckets: downsampling first would put a
    false floor under the short octaves.
    """
    since = time.time() - span
    db = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
    rows = db.execute(
        "SELECT t, offset_us, rate_ppm FROM samples WHERE gap = 0 AND t >= ? ORDER BY t", (since,)
    ).fetchall()
    db.close()

    width = max(1.0, span / LONG_BUCKETS)
    buckets: dict[int, list[tuple[float, float | None, float | None]]] = {}
    for t, off, rate in rows:
        buckets.setdefault(int((t - since) / width), []).append((t, off, rate))

    series = []
    for key in sorted(buckets):
        group = buckets[key]
        point: dict[str, Any] = {"t": round(since + (key + 0.5) * width, 1)}
        for name, index in (("off", 1), ("rate", 2)):
            values = [g[index] for g in group if g[index] is not None]
            if values:
                point[name] = round(sum(values) / len(values), 3)
                point[f"{name}lo"] = round(min(values), 3)
                point[f"{name}hi"] = round(max(values), 3)
        series.append(point)

    # One tau per octave, out to an eighth of the span: past that there are too
    # few independent windows for the answer to mean anything.
    deviation = []
    pairs = [(t, rate) for t, _, rate in rows if rate is not None]
    parts = analysis.runs(pairs)
    tau0 = analysis.sample_interval(pairs)
    if parts and tau0 > 0:
        m = 1
        while m * tau0 <= span / 8.0:
            numbers: list[float] = []
            for run in parts:
                numbers.extend(analysis.adev_freq(run, m))
            if numbers:
                sigma = math.sqrt(0.5 * statistics.fmean(numbers)) * 1e3
                deviation.append({"tau": round(m * tau0, 2), "sigma": round(sigma, 4),
                                  "pairs": len(numbers)})
            m *= 2

    return {"span": span, "series": series, "adev": deviation, "rows": len(rows)}


@app.get("/api/long")
async def long_window(span: int = 3600) -> JSONResponse:
    span = max(600, min(span, RETENTION_D * 86400))
    cached = LONG_CACHE.get(span)
    now = time.time()
    if cached is not None and now - cached[0] < LONG_CACHE_S:
        return JSONResponse(cached[1], headers={"cache-control": "max-age=10"})
    payload = await asyncio.to_thread(build_long, span)
    LONG_CACHE[span] = (now, payload)
    return JSONResponse(payload, headers={"cache-control": "max-age=10"})


@app.get("/api/history")
async def history() -> JSONResponse:
    return JSONResponse({"box": BOX_HOST, "samples": list(poller.history)}, headers={"cache-control": "no-store"})


@app.get("/healthz")
async def healthz() -> JSONResponse:
    """This viewer's own health, distinct from the box's."""
    fresh = bool(poller.last_ok) and (time.time() - poller.last_ok) < max(10.0, POLL_S * 4)
    body = {
        "ok": poller.error is None and fresh,
        "box": BOX_HOST,
        "last_ok_age_s": None if not poller.last_ok else round(time.time() - poller.last_ok, 1),
        "error": poller.error,
        "polls": poller.polls,
        "failures": poller.failures,
    }
    return JSONResponse(body, status_code=200 if body["ok"] else 503)


@app.get("/api/sats.csv")
async def sats_csv() -> JSONResponse:
    rows = build(poller).get("sats", []) if poller.doc else []
    lines = ["gnss,sv,cno,elev,azim,used,healthy,ephemeris"]
    for sat in rows:
        lines.append(
            ",".join(
                str(sat[key]) for key in ("constellation", "sv", "cno", "elev", "azim", "used", "healthy", "ephemeris")
            )
        )
    return JSONResponse({"csv": "\n".join(lines) + "\n"}, headers={"cache-control": "no-store"})
