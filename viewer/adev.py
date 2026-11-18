#!/usr/bin/env python3
"""Overlapping Allan deviation from the viewer's sample log.

Feed it samples.db and it prints sigma_y(tau): the loop's frequency stability,
or with --phase the phase stability, over the octaves it can actually resolve.

Why frequency by default. The phase here is stamped on a 1 us timer, which is
289 ppb of noise at 1 s on its own, so phase below a minute is a picture of the
timer. rate_ppm is the servo's own estimate, already averaged over the 300 s
fit, and is the only quantity that carries the oscillator rather than the
stamp.

Gaps are dropped, not bridged. A failed poll is not a clock that did not move,
and interpolating across an outage invents stability at the longest tau -- the
exact place nobody can check by eye.

  ./adev.py --db samples.db --max-tau 86400
  ./adev.py --db samples.db --phase --max-tau 600
"""

from __future__ import annotations

import argparse
import math
import sqlite3
import statistics
import sys

# Below this the rate series is reporting its own smoothing: the box fits over
# 300 s and kRateMinSamples is 60, so tau under the window is not a measurement
# of the oscillator.
FIT_WINDOW_S = 300.0


def load(path: str, field: str) -> list[tuple[float, float]]:
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    rows = db.execute(
        f"SELECT t, {field} FROM samples WHERE gap = 0 AND {field} IS NOT NULL ORDER BY t"
    ).fetchall()
    db.close()
    return [(float(t), float(v)) for t, v in rows]


def runs(samples: list[tuple[float, float]]) -> list[list[float]]:
    """Split into stretches with no gap and no interval out of family."""
    if len(samples) < 3:
        return []
    steps = [b[0] - a[0] for a, b in zip(samples, samples[1:]) if b[0] > a[0]]
    nominal = statistics.median(steps)
    out: list[list[float]] = [[samples[0][1]]]
    for (t0, _), (t1, v) in zip(samples, samples[1:]):
        if nominal * 0.5 <= (t1 - t0) <= nominal * 2.0:
            out[-1].append(v)
        else:
            out.append([v])
    return [r for r in out if len(r) >= 2]


def adev_freq(run: list[float], m: int) -> list[float]:
    """Squared second differences of m-sample means, the ADEV numerator."""
    if len(run) < m + 1:
        return []
    means = [statistics.fmean(run[i:i + m]) for i in range(len(run) - m + 1)]
    return [(b - a) ** 2 for a, b in zip(means, means[1:])]


def adev_phase(run: list[float], m: int) -> list[float]:
    """Second differences of the phase, m samples apart."""
    if len(run) < 2 * m + 1:
        return []
    return [(run[i + 2 * m] - 2.0 * run[i + m] + run[i]) ** 2 for i in range(len(run) - 2 * m)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default="samples.db")
    ap.add_argument("--field", default=None, help="column to use (default rate_ppm)")
    ap.add_argument("--phase", action="store_true", help="phase ADEV of offset_us instead")
    ap.add_argument("--min-tau", type=float, default=1.0)
    ap.add_argument("--max-tau", type=float, default=86400.0)
    args = ap.parse_args()

    field = args.field or ("offset_us" if args.phase else "rate_ppm")
    # ppb for a frequency in ppm; seconds for a phase in microseconds. ADEV is
    # a fractional quantity in both cases, so the phase is divided by tau too.
    scale = 1e3 if field == "rate_ppm" else 1e-6

    try:
        samples = load(args.db, field)
    except sqlite3.Error as exc:
        print(f"{args.db}: {exc}", file=sys.stderr)
        return 2
    if not samples:
        print(f"{args.db}: no usable {field} rows", file=sys.stderr)
        return 1

    parts = runs(samples)
    if not parts:
        print("no run long enough to say anything", file=sys.stderr)
        return 1
    tau0 = statistics.median(
        [b[0] - a[0] for a, b in zip(samples, samples[1:]) if b[0] > a[0]] or [1.0]
    )

    print(f"# {args.db}: {len(samples)} {field} samples, {tau0:.2f} s apart, {len(parts)} run(s)")
    if args.phase:
        print("# phase ADEV: below ~60 s this is the 1 us stamp, not the clock")
    elif args.min_tau < FIT_WINDOW_S:
        print(f"# rate ADEV: tau under {FIT_WINDOW_S:.0f} s is the fit window, not the crystal")
    print()
    print(f"{'tau (s)':>10} {'pairs':>8} {'sigma_y':>12} {'+/- 95%':>10}  {'band':>10}")

    m = 1
    while True:
        tau = m * tau0
        if tau > args.max_tau:
            break
        if tau >= args.min_tau:
            nums: list[float] = []
            for run in parts:
                nums.extend(adev_phase(run, m) if args.phase else adev_freq(run, m))
            if nums:
                n = len(nums)
                # Phase ADEV divides the mean square by 2*tau^2; the frequency
                # form already carries its own 1/2 and the tau is inside the
                # means.
                var = (0.5 if not args.phase else 1.0 / (2.0 * tau * tau)) * statistics.fmean(nums)
                sigma = math.sqrt(var) * scale
                band = "quantised" if tau < 60.0 else "fit window" if tau < FIT_WINDOW_S else ""
                print(f"{tau:10.0f} {n:8d} {sigma:12.3g} {sigma / math.sqrt(n):10.3g}  {band:>10}")
        m *= 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
