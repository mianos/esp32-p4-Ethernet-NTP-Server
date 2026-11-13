# Plan: PTP, metrology, and a published quality surface

Written 2026-11-14, on the state left by `8ad1d86`. A survey is locked in from the
previous day's work; everything below assumes that is either done or is Phase 4's
prerequisite rather than a competing task.

## 0. Where this actually is

The honest baseline, because every number in the later phases is relative to it.

| | now |
|---|---|
| reference | u-blox LEA-M8T, `CFG-TP5` 1 Hz asserted, `NAV-TIM-TP` refused, position surveyable |
| discipline | PPS edge → anchor + least-squares rate over 300 anchors, open loop, no PID |
| serving | NTPv4 stratum 1, root delay 2 µs, dispersion 2 µs + 1 ppm/s |
| timestamps | **software**, taken after `recvfrom()` returns |
| internal offset | ~30 µs between `CLOCK_REALTIME` and the anchor |
| PPS jitter | published as `tp_jitter_ns`, from the servo's own residuals |
| sky | 22 satellites, median C/N0 26 dBHz, site wander 26–30 m, epochs 0.6 m |
| holdover | anchor + measured rate, demotion after 10 min, no oscillator model |

That is a good stratum-1 NTP server and it is not metrology. Three things separate
them: the timestamps are software, nothing states an uncertainty budget, and
nothing independent checks the output.

## 1. What "metrology level" means here, in numbers

Metrology is not a feature, it is a claim you can defend. The claim is made by an
acceptance ladder and a documented budget behind it.

| rung | claim | measured by | passes when |
|---|---|---|---|
| A | PTP master is synchronised to GNSS | PTP offset vs a known-good master | p95 < 500 ns over 1 h, quiet LAN |
| B | NTP is accurate to its stated dispersion | offset vs the same reference | p95 < 5 µs, dispersion honest |
| C | stability is known, not assumed | TDEV at 1 s … 10 000 s | published curve, TDEV(1 s) < 100 ns |
| D | holdover is bounded | drift after PPS loss, temperature logged | stated ppm/h, uncertainty growth published |
| E | the chain is traceable | cable delay, TP5 delay, antenna position, firmware revision | every term has a value and a source |

Rungs A–C are where PTP and hardware timestamps buy something. D and E are where a
consumer timing module and a stray crystal cost something, and no amount of code
fixes them — only saying so honestly.

## 2. Phase 0 — a way to measure, before anything is claimed

Nothing above can be shown without an independent reference. Roughly a day.

- Second M8T (or any timing receiver) on the same antenna splitter, its PPS into a
  time-interval counter or scope against this board's PPS; a laptop capture for the
  numbers. This board becomes measurable rather than asserted.
- `tools/tie.py`: TIE/ADEV/TDEV/MDEV from the counter output, and from the board's
  own servo residuals as a cross-check. Publish the formula.
- Temperature: a DS18B20 or the P4's internal sensor, sampled with every stability
  point. Without it the oscillator coefficient cannot be separated from sky.
- Log schema versioned from the first sample, because the website will outlive the
  experiment and grading it later is worse than not starting.

Acceptance: an ADEV curve for this board at two known temperatures, reproducible
across two sessions.

## 3. Phase 1 — hardware timestamps, and what they fix first

The P4's EMAC has an IEEE 1588 unit (`SOC_EMAC_IEEE1588V2_SUPPORTED`) and IDF v6.0.1
exposes more than the README currently claims:

| API | gives |
|---|---|
| `esp_eth_mac_ptp_enable(cfg)` | starts the PHC (fine update method, chosen clock source) |
| `esp_eth_mac_get_ptp_time` / `set_ptp_time` | read and set the PHC |
| `esp_eth_mac_adj_ptp_freq(scale)` / `esp_eth_mac_adj_ptp_time(ppb)` | steer it, ppb granularity |
| `esp_eth_mac_enable_ts4all(true)` | **RX hardware timestamp on every frame**, delivered through `stack_input_info` as `eth_mac_time_t {s, ns}` |
| `esp_eth_transmit_ctrl_vargs(hdl, ts, …)` | **TX hardware timestamp** returned in an `eth_mac_time_t` |
| `esp_eth_mac_set_target_time` + `_cb` | comparator with an ISR callback |
| `esp_eth_mac_set_pps_out_gpio` / `set_pps_out_freq` | PPS *output* from the PHC |

So no driver patching is needed; the hook is `stack_input_info`, and the lwIP glue
uses the plain `stack_input`. Work:

- A thin custom glue that implements `stack_input_info`, passes frames to
  `esp_netif_receive`, and, for PTP frames, queues `(5-tuple, ts)` to the PTP task.
  For UDP/123 it queues `(src, sport, dport, ts)` so `t2`/`t4` become hardware.
- Same for transmit: send with `transmit_ctrl_vargs` and keep `t1`/`t3`.
- Replace the PPS ISR's `esp_timer_get_time()` with a PHC read, so the pulse is
  timestamped in the same time base everything else uses.
- Keep the software path as the fallback when the PHC is off, and report which one
  is in use — a client must be able to tell.

Acceptance: `t2` from a hardware stamp differs from the software one by a measured,
reproducible amount (tens of µs, dominated by the scheduler), and the PHC stays
within 1 µs of the anchor while PPS is present.

This phase improves NTP for every client, which is why it comes before PTP.

## 4. Phase 2 — PTP

Raw 802.3 frames over the MAC handle, bypassing lwIP. Two-step only (`Follow_Up`);
one-step needs TX timestamp prediction the unit cannot do.

- Message layer: Announce, Sync, Follow_Up, Delay_Req, Delay_Resp, and the
  management TLVs the monitoring side needs. Signalling for unicast negotiation is
  optional; default to multicast 224.0.1.129 / `01:1B:19:00:00:00` with an
  optional unicast allow-list.
- **Epoch and offset**: PTP counts TAI. `currentUtcOffset` comes from
  `NAV-TIMELS` (`leap_s` already exists); flags follow the UTC-reasonable bit;
  origin timestamps are the PHC's. Sending GPS time in a PTP header is the classic
  mistake and it is silent.
- **BMCA**: a fixed master (`priority1` low) is enough; a full BMCA is not. Announce
  `clockClass` from lock state — 6 locked, 7 holdover inside the window, 52 free-run
  — plus `clockAccuracy` and `timeSource` 0x20 (GNSS) only while actually locked.
- **Slave side too**, needed for Phase 1's acceptance test and worth having for
  itself: a PTP-capable host can then be the reference over the LAN.
- Servo on the PHC: PPS is the truth; PTP to a peer is a check and a fallback.
  Never let a network peer steer through a PPS edge that is present.

Acceptance: `ptp4l -m -s` on a Linux host on the same switch holds < 500 ns p95
offset for an hour, surviving a PPS pull for the first minute of holdover, and a
GPS-only client sees no regression.

## 5. Phase 3 — one clock, stated properly

- The PHC becomes the served clock for both NTP and PTP; `CLOCK_REALTIME` is slewed
  to it as it already is to the anchor, no longer the source of anything served.
- Steer the PHC in ppb from PPS phase, rate-limited; keep the existing 100 ms
  guard and 200 ppm clamp, in the new time base.
- Uncertainty becomes a first-class published number: base (PPS quantisation from
  `qErr`, cable, position, PHY asymmetry) plus a growth term while unlocked. This
  is what `rootDispersion` should be, and it is currently a fixed guess.
- Holdover estimated growth, not a guessed constant.

Acceptance: with PPS present, dispersion is a believable tens of nanoseconds; with
PPS absent, it grows at the measured rate and the stratum/clockClass drop at the
documented thresholds.

## 6. Phase 4 — the metrology hygiene

Prerequisite: the position survey, locked, since position error divides into all of
this (26 m is ~87 ns along the line of sight and it is currently the largest single
term).

- Cable and antenna delay: `CFG-TP5` user delay, set from a measured loopback or a
  known-length comparison, recorded with its source. One number, worth hundreds of
  ns, currently zero.
- Temperature coefficient: from Phase 0's two temperatures, in ppb/°C, published
  whether or not it is compensated.
- Antenna and feed inventory, mount description, sky obstructions — the parts of a
  traceability statement that live outside the box.
- A short `docs/accuracy.md`: the budget table with every term, its value, its
  source and its date. Rung E is a document, not an endpoint.

Acceptance: the budget's RSS explains the measured TIE within a factor of two. If
it does not, the budget is wrong and that is the finding.

## 7. Phase 5 — the quality surface the website needs

Two endpoints, one schema, published and versioned, with the definitions alongside.

```
GET /metrics          Prometheus text, the same numbers for scrapers
GET /quality.json     one current sample + the last 1440 at one a minute
GET /quality.csv      the same ring buffer as text, for a static build
```

Sample: `offset_ns`, `tie_jitter_ns`, `tdev_1s`…`tdev_10ks`, `dispersion_ns`,
`stratum`, `clock_class`, `holdover_age_s`, `sats_used`, `cn0_median`, `fix`,
`pps_age_ms`, `temp_c`, `tstamp_source` (hw/sw), `fw_rev`, `survey_state`,
`ptp_offset_ns` and `ptp_peer` when slave.

Rules that make it usable rather than decorative:

- **Units in the names**, always nanoseconds, no "quality: 87".
- If a composite score is wanted, define it in the README with the weights and
  publish every input, so the site can recompute it and a change to the weights is
  visible. One number hides which term moved.
- Ring buffer in RAM only (1440 samples of ~200 B is 288 kB — fits), rewritten
  answers nothing; a website that charts history is the point.
- 404 for `/metrics` unless enabled in config, since scanners will find it.
- Alert thresholds as data (`/quality/thresholds`), not compiled in.

Acceptance: a static page can chart 24 h of TDEV and offset from the device alone,
with no database, and each chart's axis label matches the schema's units.

## 8. Risks, and what is not worth doing

- IDF marks all of this experimental, and `required_accuracy_ns`/`clk_src` need a
  real value from the schematic, not a default.
- **PHY asymmetry** (IP101) is a fixed, uncalibratable term unless measured; it may
  be a few hundred ns and it lands directly in the PTP budget.
- The M8T is a consumer timing receiver on a poor sky. Sub-100 ns PTP is achievable
  because PPS is good even when C/N0 is not; a lab-grade 10 ns claim is not.
- The ESP32-P4 crystal is not an OCXO. Long holdover will be weak and should be
  stated, not disguised.
- Not worth it: one-step PTP, full BMCA, a web UI on the device, saving
  configuration to the module's flash (already avoided), and any score before
  Phase 0 exists to calibrate it.

## 9. Sequence

| phase | effort | depends on | value if it stalls here |
|---|---|---|---|
| 0 measure | ~1 day + a second receiver | — | an honest stability curve |
| 1 hw stamps | 2–3 days | — | better NTP for every client, and it is the shared foundation |
| 2 PTP | 3–5 days | 1 | sub-µs to PTP clients |
| 3 one clock | 1–2 days | 1, 2 | dispersion that means something |
| 4 hygiene | 1 day + hardware | survey locked | a defensible budget |
| 5 surface | 1–2 days | anything to report | the website |

Phases 4 and 5 can run in parallel with 2 once 0 exists. Phase 1 is the one to
start on: it is the foundation for both branches and it improves what already
ships.
