/* Host tests for the 1588 counter's phase slew: the bias that replaces a step
 * when the counter's second boundary is near the PPS edge's.
 *
 * This compiles main/gps/phcslew.h itself -- not a twin of it -- because what
 * matters here is the loop's behaviour, not one value: whether a phase walks in
 * without overshooting, whether it settles rather than oscillating against a
 * drifting crystal, and whether the rate excursion stays inside what a PTP
 * slave can follow. A sign error here would push the served time away from the
 * edge instead of onto it, and a loop that overshoots would turn one 10 us
 * phase into a 20 us one.
 *
 *     ./test/run.sh
 */
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "gps/phcslew.h"

namespace {

int gChecks = 0;
int gFailed = 0;

void expectEq(const char *what, long want, long got, int line)
{
    ++gChecks;
    if (want != got) {
        ++gFailed;
        std::printf("  FAIL %s:%d  %s: want %ld, got %ld\n", __FILE__, line, what, want, got);
    }
}

void expectWithin(const char *what, long limit, long got, int line)
{
    ++gChecks;
    if (got > limit || -got > limit) {
        ++gFailed;
        std::printf("  FAIL %s:%d  %s: |%ld| over %ld\n", __FILE__, line, what, got, limit);
    }
}

#define EXPECT(what, want, got) \
    expectEq((what), static_cast<long>(want), static_cast<long>(got), __LINE__)
#define EXPECT_WITHIN(what, limit, got) \
    expectWithin((what), static_cast<long>(limit), static_cast<long>(got), __LINE__)

/* One second of the loop as the firmware runs it. The phase is read at the
 * edge, biasPpb() biases the applied rate, and over the second that follows the
 * counter gains its own drift less the bias: a positive phase (counter ahead)
 * falls, a negative one rises. Returns the phase after `seconds`. */
int64_t simulate(int64_t phaseNs, int64_t driftNsPerS, int seconds, int64_t *worstRate = nullptr)
{
    int64_t worst = 0;
    for (int i = 0; i < seconds; ++i) {
        const int64_t bias = phcslew::biasPpb(phaseNs);
        worst              = bias > worst ? bias : (-bias > worst ? -bias : worst);
        phaseNs += driftNsPerS - bias;
    }
    if (worstRate != nullptr) {
        *worstRate = worst;
    }
    return phaseNs;
}

void testBiasValues()
{
    EXPECT("inside the deadband: zero", 0, phcslew::biasPpb(0));
    EXPECT("deadband edge positive: zero", 0, phcslew::biasPpb(phcslew::kDeadbandNs));
    EXPECT("deadband edge negative: zero", 0, phcslew::biasPpb(-phcslew::kDeadbandNs));
    /* Positive phase is a counter ahead, which has to run slow. */
    EXPECT("1 us ahead: slows", 100, phcslew::biasPpb(1000));
    EXPECT("1 us behind: speeds up", -100, phcslew::biasPpb(-1000));
    EXPECT("10 us ahead", 1000, phcslew::biasPpb(10000));
    /* Truncates toward zero, so it never exceeds the proportion. */
    EXPECT("201 ns ahead", 20, phcslew::biasPpb(201));
    EXPECT("saturates", phcslew::kMaxBiasPpb, phcslew::biasPpb(phcslew::kStepOverNs));
    EXPECT("saturates negative", -phcslew::kMaxBiasPpb, phcslew::biasPpb(-phcslew::kStepOverNs));
    EXPECT("far out saturates too", phcslew::kMaxBiasPpb, phcslew::biasPpb(5000000000LL));
}

void testStepOver()
{
    EXPECT("1 ms: a step", true, phcslew::stepInstead(phcslew::kStepOverNs));
    EXPECT("1 ms negative: a step", true, phcslew::stepInstead(-phcslew::kStepOverNs));
    EXPECT("just inside: a slew", false, phcslew::stepInstead(phcslew::kStepOverNs - 1));
    EXPECT("just inside negative: a slew", false, phcslew::stepInstead(-phcslew::kStepOverNs + 1));
    EXPECT("zero: a slew", false, phcslew::stepInstead(0));
}


/* Every phase this box produces on its own: 20-100 ns of tie, a few us of
 * crystal wander, and the ~29 us the counter starts out with. */
void testConvergesWithoutOvershoot()
{
    for (int64_t start : {10000LL, -10000LL, 28915LL, -28915LL, 500000LL, -500000LL}) {
        int64_t phase     = start;
        bool    crossed   = false;
        int64_t worstRate = 0;
        int     settledAt = -1;
        for (int i = 0; i < 400; ++i) {
            const int64_t bias = phcslew::biasPpb(phase);
            worstRate = bias > worstRate ? bias : (-bias > worstRate ? -bias : worstRate);
            phase += -bias;
            if ((start > 0 && phase < 0) || (start < 0 && phase > 0)) {
                crossed = true;
            }
            if (settledAt < 0 && phase <= phcslew::kDeadbandNs && phase >= -phcslew::kDeadbandNs) {
                settledAt = i + 1;
            }
        }
        EXPECT("never overshoots the edge", false, crossed);
        EXPECT_WITHIN("stays inside the deadband", phcslew::kDeadbandNs, phase);
        EXPECT("settles inside 400 s", true, settledAt > 0);
        EXPECT_WITHIN("rate excursion within the cap", phcslew::kMaxBiasPpb, worstRate);
    }

    /* 10 us of phase is gone in about 38 edges at 1/10 per edge. */
    EXPECT("10 us settles within 60 s", true, simulate(10000, 0, 60) <= phcslew::kDeadbandNs);
    /* 0.5 ms is a clamped slew: 20 us/s, so about 25 s. */
    EXPECT("0.5 ms settles within 200 s", true, simulate(500000, 0, 200) <= phcslew::kDeadbandNs);
    EXPECT("0.5 ms is still a slew", false, phcslew::stepInstead(500000));
}

/* With the crystal drifting, the loop parks the phase at about drift * the slew
 * constant, and the bias it holds there is the drift: tens of ppb, not the
 * thousands a step would leave behind. */
void testHoldsAgainstDrift()
{
    for (int64_t drift : {1LL, 2LL, 10LL, 60LL, -60LL}) {
        int64_t worstPhase = 0;
        int64_t worstRate  = 0;
        int64_t phase      = 0;
        for (int i = 0; i < 2000; ++i) {
            const int64_t bias = phcslew::biasPpb(phase);
            worstRate = bias > worstRate ? bias : (-bias > worstRate ? -bias : worstRate);
            phase += drift - bias;
            if (i > 1000) {
                worstPhase =
                    phase > worstPhase ? phase : (-phase > worstPhase ? -phase : worstPhase);
            }
        }
        /* The bias is proportional, so the loop parks where it equals the
         * drift: drift * kSlewSeconds, or, when the drift is small, a limit
         * cycle that tops out one edge's drift above the deadband. */
        const int64_t driftMag = drift < 0 ? -drift : drift;
        const int64_t hold     = phcslew::kSlewSeconds * driftMag;
        const int64_t cycle    = phcslew::kDeadbandNs + driftMag;
        EXPECT_WITHIN("settled phase", hold > cycle ? hold : cycle, worstPhase);
        /* At the deadband the bias is (deadband + drift)/kSlewSeconds. */
        EXPECT_WITHIN("settled rate excursion", 100, worstRate);
    }
}

/* 20 ppm is 20 us/s, so the full 1 ms takes 50 s clamped and about 68 more to
 * fall from 200 us to the deadband: inside two minutes, and never past zero. */
void testWorstCaseSlew()
{
    int64_t       worstRate = 0;
    const int64_t after     = simulate(phcslew::kStepOverNs - 1, 0, 150, &worstRate);
    EXPECT_WITHIN("1 ms is inside the deadband after 150 s", phcslew::kDeadbandNs, after);
    EXPECT("1 ms uses the full cap", phcslew::kMaxBiasPpb, worstRate);
    EXPECT("1 ms never crosses the edge", true, after >= 0);
}

} // namespace

int main()
{
    testBiasValues();
    testStepOver();
    testConvergesWithoutOvershoot();
    testHoldsAgainstDrift();
    testWorstCaseSlew();

    std::printf("%d checks, %d failed\n", gChecks, gFailed);
    return gFailed == 0 ? 0 : 1;
}
