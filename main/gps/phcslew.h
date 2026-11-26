/* The 1588 counter's phase against the PPS edge: stepped, or slewed.
 *
 * The counter carries PTP's timestamps raw, so a step is a discontinuity a
 * slave follows, and it leaves the counter's own read-to-set gap (about 20 us)
 * behind as a landing error. Between kDeadbandNs and kStepOverNs the phase is
 * therefore walked out with a rate bias instead: the counter runs wrong by
 * biasPpb() for as long as it takes, and its second boundary never moves
 * discontinuously. Beyond kStepOverNs a slew at kMaxBiasPpb would take minutes,
 * so the counter is stepped there.
 *
 * The bias is a proportional term on one edge's phase, so with a drift of r
 * ns/s the loop settles at about r * kSlewSeconds and cannot overshoot: each
 * edge takes off a fixed fraction of what is left. Below the deadband the phase
 * is inside the stamping spread and the bias would only track noise.
 *
 * Plain values only, no ESP-IDF and no state, so test/phcslew_test.cpp can
 * exercise it on a host. */
#pragma once

#include <cstdint>

namespace phcslew {

/* Inside this the phase is not acted on: the tie is 20-100 ns. */
constexpr int64_t kDeadbandNs = 200;

/* Further out than this a slew is minutes long, so the counter is stepped. */
constexpr int64_t kStepOverNs = 1000000; /* 1 ms */

/* Seconds the phase takes to fall to 1/e, and so the bias per ns of phase. */
constexpr int32_t kSlewSeconds = 10;

/* Caps the rate excursion while a large phase is walked out. */
constexpr int32_t kMaxBiasPpb = 20000; /* 20 ppm */

/* Phase is the counter's reading at the edge minus the edge's second, so a
 * positive value means the counter is ahead and has to run slow: the caller
 * subtracts this from the rate it applies. */
inline int32_t biasPpb(int64_t phaseNs)
{
    if (phaseNs <= kDeadbandNs && phaseNs >= -kDeadbandNs) {
        return 0;
    }
    const int64_t bias = phaseNs / kSlewSeconds;
    if (bias > kMaxBiasPpb) {
        return kMaxBiasPpb;
    }
    if (bias < -kMaxBiasPpb) {
        return -kMaxBiasPpb;
    }
    return static_cast<int32_t>(bias);
}

/* Whether this phase is too far to slew, and so has to be stepped. At or beyond
 * kStepOverNs, since that is where the clamp would still be running after a
 * minute. */
inline bool stepInstead(int64_t phaseNs)
{
    return phaseNs >= kStepOverNs || phaseNs <= -kStepOverNs;
}

} // namespace phcslew
