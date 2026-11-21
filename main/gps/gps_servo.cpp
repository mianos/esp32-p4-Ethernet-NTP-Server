/*
 * The clock itself: pairing a frame with an edge into an anchor, the rate fit,
 * the served time, the uncertainty budget, and the slew of CLOCK_REALTIME.
 */
#include "gps/gps_internal.h"

#include "gps/pps_latency.h"

using namespace gps_detail;

void GpsTime::anchor(time_t second)
{
    int64_t  ppsMonoUs = 0;
    int64_t  ppsMonoNs = 0;
    int64_t  ppsPtpNs  = 0;
    uint32_t ppsCount  = 0;

    portENTER_CRITICAL(&m_mux);
    ppsMonoUs = m_ppsMonoUs;
    ppsMonoNs = m_ppsMonoNs;
    ppsPtpNs  = m_ppsPtpNs;
    ppsCount  = m_ppsCount;
    /* Before qErr moves it: the latency probe holds the stamp the handler took,
     * not the corrected edge. */
    const int64_t rawPpsMonoUs = ppsMonoUs;
    /* The edge's own quantisation, from the TIM-TP that described this pulse.
     * Convention is corrected = captured - qErr (gnssdo, qerr.rs); the sawtooth
     * is the receiver only being able to place an edge on its own clock's grid.
     * TIM-TP describes the *next* pulse, so a correction received in the last
     * 1.2 s belongs to this edge; older than that and it describes the one
     * before, younger than -50 ms and it arrived after this one and belongs to
     * the next.
     *
     * Requires the UTC base with UTC available. gpsd also withholds the figure
     * while raim is active; this module reports raim active continuously
     * (flags 0x0b), so following that would disable the correction outright --
     * and raim says nothing about the pulse generator's grid. */
    if (m_qErrValid && (m_timTpFlags & 0x03) == 0x03 && m_qErrAtUs != 0) {
        const int64_t ageUs = ppsMonoUs - m_qErrAtUs;
        if (ageUs >= -50000 && ageUs <= 1200000) {
            /* Nanoseconds off a microsecond stamp: without the division a
             * 10 ns sawtooth moved the edge 10 us, and the clamp allowed 5 ms. */
            ppsMonoUs -= m_qErrNs / 1000;
            if (ppsMonoNs != 0) {
                ppsMonoNs -= m_qErrNs;
            }
            /* The same correction in the counter the reply is stamped in, or
             * the two anchors would differ by the sawtooth. */
            if (ppsPtpNs != 0) {
                ppsPtpNs -= m_qErrNs;
            }
        }
    }
    portEXIT_CRITICAL(&m_mux);

    /* No edge yet, or this second's edge is already anchored: the frame carries
     * nothing new for the pair. */
    if (ppsCount == 0 || ppsCount == m_pairedCount) {
        return;
    }

    const int64_t ageUs = esp_timer_get_time() - ppsMonoUs;
    if (ageUs < 0 || ageUs > kPairWindowUs) {
        /* The frame arrived too long after the last edge to be describing it --
         * the receiver was late, or the edge was missed. Pairing anyway would
         * put every timestamp off by a whole second, so this second is dropped;
         * if it keeps happening, lock is lost, which is the correct answer. */
        return;
    }

    m_pairedCount = ppsCount;

    /* Accepted, so this is a real measurement of the sentence latency rather
     * than of a sentence that arrived too late to trust. Recorded here and not
     * in the ISR: it is a property of the pairing, and the pairing is what the
     * window above bounds. */
    {
        const int64_t atUs = esp_timer_get_time();
        portENTER_CRITICAL(&m_mux);
        m_pairAgeUs    = ageUs;
        m_pairAgeMaxUs = ageUs > m_pairAgeMaxUs ? ageUs : m_pairAgeMaxUs;
        m_pairAgeAtUs  = atUs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* What the servo predicted for this instant before it knew. Across an
     * ordinary second this is a check on the rate; across a holdover it is the
     * error a client was served while the edge was missing, and the only
     * honest way to quote what a holdover was worth. */
    int64_t predictedNs = 0;
    int64_t gapUs       = 0;
    bool    hadAnchor   = false;
    bool    rateKnown   = false;
    {
        portENTER_CRITICAL(&m_mux);
        rateKnown = m_rateSamples >= kRateMinSamples;
        if (m_anchorValid && ppsMonoUs > m_anchorMonoUs) {
            hadAnchor   = true;
            gapUs       = ppsMonoUs - m_anchorMonoUs;
            predictedNs = m_anchorUtcNs + gapUs * 1000 - gapUs * m_ratePpb / 1000000;
        }
        portEXIT_CRITICAL(&m_mux);
    }

    /* Classify the edge before it is allowed to move anything, on two
     * independent questions that both have to be answered yes:
     *
     *  - regularity: did about a second pass since the last edge? The ISR has
     *    already refused anything under half a second, so what is left is a
     *    pulse that was lost (1.4 s) or one emitted twice (0.7 s);
     *  - continuity: does the module's own date agree with the time that
     *    actually passed? A receiver that reconfigured its pulse, or that has
     *    just been stepped, emits an edge a second early or late while the
     *    spacing looks perfect -- and a sample whose second is wrong is a whole
     *    second wrong, not a little wrong.
     *
     * An edge failing either is counted and dropped. It does not become the
     * anchor, does not enter the fit and does not steer: holding the anchor
     * still lets the servo's own rate carry this second, which is what the fit
     * is for, and costs one second of phase correction instead of poisoning
     * five minutes of rate. */
    /* Spacing is measured edge to edge, not from the anchor. The two differ
     * exactly when it matters: a refused edge leaves the anchor behind, so the
     * next gap measured from the anchor is two seconds while only one second of
     * edges went by -- and testing against that refuses every edge after the
     * first and never recovers. */
    const int64_t edgeGapUs = m_lastEdgeMonoSet ? ppsMonoUs - m_lastEdgeMonoUs : 0;
    const bool    regular =
        !m_lastEdgeMonoSet || (edgeGapUs >= kEdgeRegularMinUs && edgeGapUs <= kEdgeRegularMaxUs);
    /* Continuity is tested against the seconds that actually elapsed rather than
     * against a flat one, or a returning edge after an outage would always fail
     * it and the anchor would never come back. Half a second of rounding, and
     * nothing beyond what the pulse allowed: a date that jumps further than the
     * edges did is the pairing being wrong. */
    const int64_t expectedS = m_lastEdgeMonoSet ? (edgeGapUs + 500000) / 1000000 : 1;
    const bool    advance =
        m_lastEdgeSecondSet &&
        static_cast<int64_t>(second) == static_cast<int64_t>(m_lastEdgeSecond) + expectedS;
    /* An edge that ends a holdover is outside a second's spacing by definition,
     * and has to be usable: it is the one the holdover error is measured
     * against. From kRateGapNs up, the fit is dropped by the gap path in
     * pushRateSample() anyway, so admitting it costs the fit nothing. Between
     * one missed edge and that, the spanning sample would still be fitted and
     * the edge is refused instead. */
    const bool resumed = m_lastEdgeMonoSet && edgeGapUs >= kRateGapNs / 1000;
    const bool spaced  = advance && (regular || resumed);

    /* And a third question, once there is a fit to ask it of: is the edge where
     * the fit said it would be? Only for ordinary edges, as with the spike
     * count below -- across a holdover the prediction has been running on the
     * crystal and its error is the thing being measured. See kEdgeOutlierNs. */
    const int64_t errorNs = static_cast<int64_t>(second) * 1000000000 - predictedNs;

    /* The edge's wait for the interrupt next to its distance from the fit, and
     * only when one of the two is worth looking at. An error that tracks the
     * wait is core 1 having been held off; one without it is the receiver
     * having moved the pulse. The overhead is logged so it can be subtracted. */
    if (rateKnown) {
        int64_t latencyNs = 0;
        if (pps_latency::forEdge(rawPpsMonoUs, &latencyNs) &&
            (latencyNs >= 2000 || (hadAnchor && (errorNs >= 4000 || errorNs <= -4000)))) {
            pps_latency::Stats probe      = {};
            const int64_t      overheadNs = pps_latency::stats(&probe) ? probe.minNs : 0;
            ESP_LOGW(TAG, "edge latency %lld ns (overhead %lld), %+lld ns from the fit",
                     static_cast<long long>(latencyNs), static_cast<long long>(overheadNs),
                     static_cast<long long>(errorNs));
        }
    }

    const bool outlier = spaced && hadAnchor && rateKnown && gapUs < kLockWindowUs &&
                         (errorNs > kEdgeOutlierNs || errorNs < -kEdgeOutlierNs);
    const bool trusted = spaced && !outlier;

    /* A pulse arriving a millisecond or more from where the fit and the anchor
     * put it is being corrupted rather than merely late. Measured against the
     * prediction in the monotonic domain, because that is the domain the servo
     * works in -- against the realtime clock the figure would carry whatever
     * slew is in flight. Only for ordinary edges: across a holdover the
     * prediction has been running on the crystal for seconds and its error is
     * expected, so testing there would count every return from a gap. */
    if (spaced && hadAnchor && gapUs < kLockWindowUs &&
        (errorNs > kEdgeSpikeNs || errorNs < -kEdgeSpikeNs)) {
        portENTER_CRITICAL(&m_mux);
        ++m_edgeSpikes;
        portEXIT_CRITICAL(&m_mux);
    }
    if (outlier) {
        portENTER_CRITICAL(&m_mux);
        ++m_edgeOutliers;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG, "edge %+lld us from the fit: not anchored",
                 static_cast<long long>(errorNs / 1000));
    }

    m_lastEdgeSecond    = second;
    m_lastEdgeSecondSet = true;
    m_lastEdgeMonoUs    = ppsMonoUs;
    m_lastEdgeMonoSet   = true;

    portENTER_CRITICAL(&m_mux);
    m_lastEdgeTrusted = trusted;
    uint32_t streak   = trusted ? 0 : ++m_edgeRejects;
    if (trusted) {
        m_edgeRejects = 0;
    }
    portEXIT_CRITICAL(&m_mux);

    if (!trusted) {
        /* One refused edge is one second of unconsumed correction. Forty-five
         * of them in a row is not a coincidence: either the pulse or the
         * module's idea of the second is wrong, and a fit built across that is
         * worth less than the minute it takes to build a new one. */
        if (streak == kRejectStreakReacquire) {
            resetRateWindow();
            ESP_LOGW(TAG,
                     "servo: %u edges in a row refused (spacing or second continuity) "
                     "- refitting the rate",
                     static_cast<unsigned>(streak));
        }
        return;
    }

    portENTER_CRITICAL(&m_mux);
    m_anchorUtcNs  = static_cast<int64_t>(second) * 1000000000;
    m_anchorMonoUs = ppsMonoUs;
    /* The same anchor in the counter a reply's arrival is stamped by, so the
     * served time and the stamp need no conversion. Zero when the PHC is not
     * running, and now() then falls back to the monotonic clock. */
    m_anchorPhcNs = ppsPtpNs;
    m_anchorValid = true;
    m_lastFix     = second;
    portEXIT_CRITICAL(&m_mux);

    /* The counter is set from CLOCK_REALTIME at start-up, when that may still
     * be 1970; the step to the first fix leaves the two milliseconds apart and
     * every PTP stamp then carries the difference. Correct it here, and drop the
     * PHC anchor when it moves so the served time falls back for one edge. */
    if (ptp::correctPhc()) {
        portENTER_CRITICAL(&m_mux);
        m_anchorPhcNs = 0;
        portEXIT_CRITICAL(&m_mux);
    }

    if (gapUs >= kLockWindowUs) {
        /* The edge had gone stale and the servo carried the clock: this is what
         * that cost, and it is logged rather than served in the hope that
         * nobody looks. */
        const int64_t errorNs = static_cast<int64_t>(second) * 1000000000 - predictedNs;
        portENTER_CRITICAL(&m_mux);
        m_lastHoldoverUs    = gapUs;
        m_lastHoldoverErrNs = errorNs;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG,
                 "PPS back after %.1f s: the servo was %+lld us out across the gap "
                 "(%+.3f ppm residual)",
                 static_cast<double>(gapUs) / 1e6, static_cast<long long>(errorNs / 1000),
                 static_cast<double>(errorNs) / static_cast<double>(gapUs) * 1e3);
    }

    /* The same pair is one sample of the crystal against the GPS: the anchor
     * says where the second started, this says how fast our own counter got
     * here. */
    pushRateSample(ppsMonoNs != 0 ? ppsMonoNs : ppsMonoUs * 1000,
                   static_cast<int64_t>(second) * 1000000000);

    steer();
}

int64_t GpsTime::rootDelayNs() const
{
    /* Zero, because there is no round trip to measure. The reference is a
     * receiver on the board, so the field that means "path to the reference" has
     * nothing to say, and ntpd reports 0 for a local refclock for exactly that
     * reason.
     *
     * What the 283 us once published here actually is: our own stamping error,
     * which belongs in dispersion. Keeping it in the delay is not neutral --
     * a client halves the delay into its bound, so a local error advertised
     * there is charged at half rate while the client *also* measures a real
     * delay to us that already includes the processing. It has been charged
     * twice. */
    return 0;
}

int64_t GpsTime::sampleErrorNs() const
{
    /* The fixed part of the budget: the interrupt latency of the pulse capture,
     * and half the receiver's stated pulse quantisation -- half because the
     * sawtooth is uniform over the step and a bound is about the worst case
     * from the middle of it. The moving part, the drift of the served time, is
     * measured on the wire and added in dispersionNs(). */
    return kIsrLatencyNs + (m_qErrValid ? std::abs(m_qErrNs) / 2 : 0);
}

int64_t GpsTime::unobservedNs() const
{
    /* A pulse nobody is watching. While TIM-TP is configured and has stopped
     * arriving, the position of the edge against its own second is unobserved
     * -- the servo cannot see it either, because the anchor is taken from the
     * same edge and moves with it. Unobserved is not known, so the stated
     * uncertainty grows with how long it has been silent.
     *
     * 10 us a second covers the measured growth (4.2 us/s to 133 ms) with
     * margin, and the 30 s grace keeps a mode change from widening it. Capped
     * at 200 ms, six times the worst error seen and still finite. */
    if (m_timTpCfg != 1) {
        return 0;
    }
    /* In fixed mode the receiver stops sending TIM-TP: it is a survey-time
     * message, and a held position has no survey to describe. The edge itself is
     * still captured and still trusted, so what is missing is only the
     * receiver's statement of where its own pulse sits -- bounded by the
     * generator's step, not unknowable. Escalating here demoted a box whose
     * clock was being disciplined by the same receiver, and since the term only
     * grows and never comes back the box never promoted again: the upstream kept
     * the clock while 40 µs of honest dispersion had been the truth. The
     * quantisation clamp is the bound to state instead, and only while the edge
     * is trusted -- an untrusted edge below returns to the escalation. */
    /* The edge has to be current as well as trusted. m_lastEdgeTrusted is only
     * written when an edge is processed, so it keeps its last value across an
     * outage -- and a free-running box claiming the quantisation clamp while
     * nothing is arriving is the underclaim the escalation was added for. */
    if (m_tmode == 2 && m_lastEdgeTrusted && m_lastEdgeUs > 0 &&
        esp_timer_get_time() - m_lastEdgeUs < 2000000) {
        return kMaxQErrNs;
    }
    const int64_t silentS = (esp_timer_get_time() - m_qErrAtUs) / 1000000;
    if (silentS <= 30) {
        return 0;
    }
    return std::min<int64_t>((silentS - 30) * 10000, 200000000);
}

int64_t GpsTime::dispersionNs(int64_t ageUs, int64_t unobservedNs) const
{
    const int64_t ageS = ageUs > 0 ? ageUs / 1000000 : 0;
    return sampleErrorNs() + ServePath::instance().boundNs() + m_jitterNs + ageS * kWanderNsPerS +
           unobservedNs;
}

void GpsTime::recordSample(uint32_t epoch, int64_t offsetNs)
{
    /* Missing epochs first. A holdover produces no call at all until the next
     * edge, so the holes are only visible here, and one record per skipped
     * second is what lets a client tell "nothing was measured" from "nothing
     * arrived". */
    if (m_lastSampleSet && epoch > m_lastSampleEpoch + 1) {
        uint32_t missing = epoch - m_lastSampleEpoch - 1;
        if (missing > kSampleRing) {
            missing = kSampleRing;
        }
        for (uint32_t i = 0; i < missing; ++i) {
            /* The row has no measurement, but the seconds it covers were still
             * served: their bound is the one in force at the time, which is the
             * current budget plus the wander over the age of that particular
             * second. Leaving it zero said "no uncertainty here" about seconds
             * that had nothing but the crystal behind them. Worked out before
             * the lock, like the measured row's below. */
            const int64_t  ageS     = static_cast<int64_t>(missing - i);
            const uint32_t gapBound = static_cast<uint32_t>(
                clamp(dispersionNs(ageS * 1000000, unobservedNs()), 0, UINT32_MAX));

            portENTER_CRITICAL(&m_mux);
            Sample &slot      = m_samples[m_sampleSeq % kSampleRing];
            slot.seq          = m_sampleSeq;
            slot.epoch        = epoch - missing + i;
            slot.kind         = 1;
            slot.offsetNs     = 0;
            slot.ratePpb      = m_ratePpb;
            slot.dispersionNs = gapBound;
            slot.jitterNs     = 0;
            slot.flags        = 0;
            slot.sats         = 0;
            slot.fix          = 0;
            ++m_sampleSeq;
            if (m_sampleSeq - m_sampleOldest > kSampleRing) {
                m_sampleOldest = m_sampleSeq - kSampleRing;
            }
            portEXIT_CRITICAL(&m_mux);
        }
    }

    /* The bound this reference states about the packet it would have built for
     * this second, from the same terms status() uses. Recorded at the instant
     * of the sample, so the age term is zero and the wander arrives later, in
     * the rows a holdover generates. Worked out before the lock: it reads a
     * clock and takes the serve path's own lock on the way. */
    const uint32_t boundNs =
        static_cast<uint32_t>(clamp(dispersionNs(0, unobservedNs()), 0, UINT32_MAX));

    portENTER_CRITICAL(&m_mux);
    Sample &slot      = m_samples[m_sampleSeq % kSampleRing];
    slot.seq          = m_sampleSeq;
    slot.epoch        = epoch;
    slot.kind         = 0;
    slot.offsetNs     = static_cast<int32_t>(clamp(offsetNs, INT32_MIN, INT32_MAX));
    slot.ratePpb      = m_ratePpb;
    slot.dispersionNs = boundNs;
    slot.jitterNs     = static_cast<uint16_t>(clamp(m_jitterNs, 0, UINT16_MAX));
    slot.flags        = m_timTpFlags;
    slot.sats         = m_satCount;
    slot.fix          = m_fixType;
    ++m_sampleSeq;
    if (m_sampleSeq - m_sampleOldest > kSampleRing) {
        m_sampleOldest = m_sampleSeq - kSampleRing;
    }
    m_lastSampleEpoch = epoch;
    m_lastSampleSet   = true;
    portEXIT_CRITICAL(&m_mux);
}

size_t GpsTime::drainSamples(uint32_t since, Sample *out, size_t max, uint32_t &first,
                             uint32_t &last, uint32_t &lost) const
{
    /* No lock. The GPS task is the only writer and moves the sequence on after
     * a record is complete, so everything below `newest` is whole; a record is
     * only overwritten 900 s after it was written, and each copy is checked
     * against its own sequence number in case this reader was away that long. */
    const uint32_t newest = m_sampleSeq.load(std::memory_order_acquire);
    const uint32_t oldest = m_sampleOldest.load(std::memory_order_acquire);
    first                 = oldest;
    last                  = newest == 0 ? 0 : newest - 1;
    if (since == 0 || since < oldest) {
        lost  = since == 0 ? 0 : oldest - since;
        since = oldest;
    } else {
        lost = 0;
    }
    size_t copied = 0;
    for (uint32_t seq = since; seq < newest && copied < max; ++seq) {
        out[copied] = m_samples[seq % kSampleRing];
        if (out[copied].seq == seq) {
            ++copied;
        }
    }
    return copied;
}

void GpsTime::resetRateWindow()
{
    m_rateCount = 0;
    m_rateNext  = 0;
    /* The first sample after this seeds the window instead of being differenced
     * against one from the window that was just thrown away. */
    m_lastRateOffsetSet = false;
    /* The published deviation and sample count have to go with the window they
     * were taken over, or the panel keeps quoting both for a sequence that no
     * longer exists. rateSamples is normally only refreshed inside fitRate(),
     * which early-returns below its minimum, so left alone it would sit at the
     * old number for a minute. */
    portENTER_CRITICAL(&m_mux);
    m_rateSamples = 0;
    portEXIT_CRITICAL(&m_mux);
}

void GpsTime::pushRateSample(int64_t monoNs, int64_t utcNs)
{
    const int64_t offsetNs = utcNs - monoNs;

    /* A gap in the reference: neither the fit below nor a second difference
     * wants a sample with a hole in front of it. Drop the window, drop the
     * spanning sample, and let the first post-gap second seed it again. */
    if (m_lastRateMonoSet && monoNs - m_lastRateMonoNs > kRateGapNs) {
        const int64_t gapMs = (monoNs - m_lastRateMonoNs) / 1000000;
        resetRateWindow();
        m_lastRateMonoNs  = monoNs;
        m_lastRateMonoSet = true;
        ESP_LOGW(TAG, "servo: %lld s without a reference, dropping the rate window",
                 static_cast<long long>(gapMs / 1000));
        return;
    }

    if (m_lastRateOffsetSet && (offsetNs - m_lastRateOffsetNs > kRateResetNs ||
                                m_lastRateOffsetNs - offsetNs > kRateResetNs)) {
        /* The module moved the clock under us, or a pairing landed a second out.
         * A slope fitted across that step is about nothing, so the window starts
         * again -- and a second out is worth saying out loud, because it is the
         * one failure that would otherwise look like a healthy clock. */
        const int64_t stepMs = (offsetNs - m_lastRateOffsetNs) / 1000000;
        resetRateWindow();
        ESP_LOGW(TAG, "servo: %+lld ms step in the reference, refitting the rate",
                 static_cast<long long>(stepMs));
    }
    m_lastRateOffsetNs  = offsetNs;
    m_lastRateOffsetSet = true;
    m_lastRateMonoNs    = monoNs;
    m_lastRateMonoSet   = true;

    m_rate[m_rateNext] = RateSample{monoNs, utcNs};
    m_rateNext         = (m_rateNext + 1) % kRateWindow;
    if (m_rateCount < kRateWindow) {
        ++m_rateCount;
    }

    fitRate();
}

void GpsTime::fitRate()
{
    if (m_rateCount < kRateMinSamples) {
        return;
    }
    const int64_t t0 = esp_timer_get_time();

    /* Least squares of UTC nanoseconds against the edge stamp in nanoseconds,
     * both
     * taken relative to the newest sample first: UTC nanoseconds are 1.8e18 at
     * the moment, which is past the point where a double holds every integer,
     * and what is wanted here is a slope of 1e-8. Relative, the numbers are
     * hundreds of seconds in size and the fit is exact enough that the residual
     * below is the receiver's jitter and nothing else.
     *
     * The window order does not matter to the sums, so the ring is read as a
     * flat array of however many slots are filled. */
    const RateSample &base = m_rate[(m_rateNext + kRateWindow - 1) % kRateWindow];

    int64_t sumX = 0;
    int64_t sumY = 0;
    for (int i = 0; i < m_rateCount; ++i) {
        sumX += m_rate[i].monoNs - base.monoNs;
        sumY += m_rate[i].utcNs - base.utcNs;
    }
    const double meanX = static_cast<double>(sumX) / m_rateCount;
    const double meanY = static_cast<double>(sumY) / m_rateCount;

    double sxy = 0.0;
    double sxx = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoNs - base.monoNs) - meanX;
        double       dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY;
        sxy += dx * dy;
        sxx += dx * dx;
    }
    if (sxx <= 0.0) {
        return;
    }

    /* Both axes are nanoseconds now, so a perfect counter gives a slope of
     * exactly 1 and the rate error is how far from 1 it sits, in parts per
     * billion. */
    const double slope   = sxy / sxx;
    int64_t      ppb     = static_cast<int64_t>(std::llround((1.0 - slope) * 1e9));
    bool         clamped = false;
    if (ppb > kRateLimitPpb) {
        ppb     = kRateLimitPpb;
        clamped = true;
    } else if (ppb < -kRateLimitPpb) {
        ppb     = -kRateLimitPpb;
        clamped = true;
    }

    portENTER_CRITICAL(&m_mux);
    if (clamped) {
        ++m_servoClamps;
    }
    m_ratePpb     = static_cast<int32_t>(ppb);
    m_rateSamples = static_cast<uint16_t>(m_rateCount);
    portEXIT_CRITICAL(&m_mux);

    /* The counter the PTP stamps come from is not the one the servo steers by
     * reading, and left alone it drifts at exactly the rate just fitted -- 30 ppm
     * is 120 ms in an hour. Push the fit into it, throttled: only when it moves
     * materially, and otherwise once every thirty seconds so a lost push (a MAC
     * restart re-initialises the block) comes back. */
    {
        static int32_t lastPpb = 0;
        static int64_t lastUs  = 0;
        const int64_t  nowUs   = esp_timer_get_time();
        if (ptp::hardware() && m_rateSamples >= kRateMinSamples &&
            (std::abs(static_cast<int32_t>(ppb) - lastPpb) >= 500 || nowUs - lastUs > 30000000)) {
            if (ptp::setRatePpb(static_cast<int32_t>(-ppb)) == ESP_OK) {
                lastPpb = static_cast<int32_t>(ppb);
                lastUs  = nowUs;
            }
        }
    }

    /* Residual: the thing that says the samples are actually samples of a
     * clock and not of the receiver's mood -- and, while the servo is running,
     * the bound this reference states. Each edge's distance from the line
     * through all of them is the stamping jitter, measured, and RMS is the
     * figure a root dispersion wants. TIM-TP would describe the same jitter
     * directly, and this receiver refuses TIM-TP, so the residuals carry it. */
    double sse    = 0.0;
    double tieMax = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoNs - base.monoNs) - meanX;
        const double dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY - slope * dx;
        sse += dy * dy;
        tieMax = std::fabs(dy) > tieMax ? std::fabs(dy) : tieMax;
    }
    m_fitUs                 = esp_timer_get_time() - t0;
    const double residualNs = std::sqrt(sse / m_rateCount);
    {
        const int64_t jitterNs = residualNs > static_cast<double>(kMaxJitterNs)
                                     ? kMaxJitterNs
                                     : static_cast<int64_t>(residualNs);
        portENTER_CRITICAL(&m_mux);
        m_jitterNs = jitterNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* The worst excursion, from the same residuals: one bad edge shows here
     * while an RMS barely moves. The RMS is the residual already computed. */
    {
        portENTER_CRITICAL(&m_mux);
        m_tieMaxNs = tieMax;
        m_tieRmsNs = residualNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* Once when it is first known, and then only when it moves by enough to be
     * worth a line: a servo that logs its estimate every second has made the
     * console unusable for everything else. */
    if (!m_rateLogged || std::llabs(ppb - m_lastLoggedPpb) > 500) {
        ESP_LOGI(TAG, "servo: %+lld ppb (%+.3f ppm) over %d anchors, residual %.1f us",
                 static_cast<long long>(ppb), static_cast<double>(ppb) / 1000.0, m_rateCount,
                 residualNs / 1000.0);
        m_rateLogged    = true;
        m_lastLoggedPpb = ppb;
    }
}

void GpsTime::steer()
{
    struct timespec gps = {};
    if (!now(gps)) {
        return;
    }
    struct timespec sys = {};
    clock_gettime(CLOCK_REALTIME, &sys);

    const int64_t offsetNs =
        (static_cast<int64_t>(gps.tv_sec) - static_cast<int64_t>(sys.tv_sec)) * 1000000000 +
        (gps.tv_nsec - sys.tv_nsec);
    {
        portENTER_CRITICAL(&m_mux);
        m_offsetUs = offsetNs / 1000;
        m_offsetNs = offsetNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* The record carries what a client reads, not the raw counter phase. The
     * monotonic counter is never steered -- every correction is arithmetic in
     * now() -- so its phase against GPS sits at milliseconds and wanders with
     * the crystal, while the offset here is against the steered realtime clock
     * and is the number the servo is actually holding. */
    recordSample(static_cast<uint32_t>(gps.tv_sec), offsetNs);

    /* Who may write CLOCK_REALTIME: this source, or the fallback's upstream
     * client. The measurement above happens either way -- the offset is
     * published and charted through a fallback too, and it is what shows the two
     * references pulling apart -- but only the owner corrects anything. Two
     * owners is worse than either alone: the client would step, this would
     * measure the step as an error, and the two would undo each other. */
    if (!ClockSource::instance().gpsSteers()) {
        return;
    }

    if (!m_stepped || offsetNs > kStepThresholdNs || offsetNs < -kStepThresholdNs) {
        struct timeval tv = {};
        tv.tv_sec         = gps.tv_sec;
        tv.tv_usec        = gps.tv_nsec / 1000;
        if (settimeofday(&tv, nullptr) != 0) {
            ESP_LOGW(TAG, "settimeofday: errno %d", errno);
            return;
        }
        if (!m_stepped) {
            ESP_LOGI(TAG,
                     "clock set from the GPS (%s): CLOCK_REALTIME now agrees with what "
                     "the responder serves",
                     isoTimestamp(gps.tv_sec).c_str());
        } else {
            ESP_LOGW(TAG, "clock stepped %+lld ms: further than a slew can fix quickly",
                     static_cast<long long>(offsetNs / 1000000));
        }
        m_stepped = true;
        portENTER_CRITICAL(&m_mux);
        ++m_servoSteps;
        /* The counter has to move with the clock, and the anchor's stamp in it
         * is now stale by the step: dropping it sends now() back to the
         * monotonic path until the next edge pairs. */
        m_anchorPhcNs = 0;
        portEXIT_CRITICAL(&m_mux);
        if (ptp::hardware()) {
            (void)ptp::stepToRealtime();
        }

        /* A step is a phase discontinuity the fit cannot see coming and the
         * step detector below cannot catch: that one compares samples and
         * trips at kRateResetNs (100 ms), while a step here is anything over
         * 10 ms. Left in the window, the samples on the far side of it are a
         * constant offset away and *tieMaxNs* -- the lock's own worst-residual
         * test -- sits above its bound for the five minutes the window takes
         * to flush. That is five minutes of stratum 16 immediately after every
         * hand-back from a fallback, which is the opposite of the point. */
        resetRateWindow();

        /* The set() reached the timebase after the instant gps was read, so the
         * clock is now late by the syscall path -- tens of microseconds that
         * would otherwise sit there until the next edge noticed. Measure it
         * again and take it out with a slew, which is the one kind of
         * correction that does not add latency of its own. */
        struct timespec after = {};
        if (now(after)) {
            struct timespec sysAfter = {};
            clock_gettime(CLOCK_REALTIME, &sysAfter);
            const int64_t residueNs =
                (static_cast<int64_t>(after.tv_sec) - static_cast<int64_t>(sysAfter.tv_sec)) *
                    1000000000 +
                (after.tv_nsec - sysAfter.tv_nsec);
            if (residueNs <= kPostStepSlewNs && residueNs >= -kPostStepSlewNs) {
                slew(residueNs);
            }
        }
        return;
    }

    /* Everything smaller than the step threshold is slewed, so a client polling
     * across this moment never sees the clock jump -- and unlike a step the
     * correction leaves no residue of its own behind. */
    slew(offsetNs);
}

void GpsTime::slew(int64_t offsetNs)
{
    /* IDF's adjtime() applies about 1.5% of elapsed time per call, so this
     * clears in seconds rather than minutes. A slew already in flight is not
     * lost: adjtime() replaces the remaining adjustment rather than queueing
     * behind it. */
    int64_t sec  = offsetNs / 1000000000;
    int64_t usec = (offsetNs % 1000000000) / 1000;
    if (usec < 0) {
        usec += 1000000;
        --sec;
    }
    struct timeval delta = {};
    delta.tv_sec         = static_cast<time_t>(sec);
    delta.tv_usec        = static_cast<suseconds_t>(usec);
    if (adjtime(&delta, nullptr) != 0) {
        ESP_LOGW(TAG, "adjtime: errno %d", errno);
    }
}

bool GpsTime::now(struct timespec &ts) const
{
    int64_t utcNs     = 0;
    int64_t monoUs    = 0;
    int64_t anchorPhc = 0;
    int32_t ratePpb   = 0;

    portENTER_CRITICAL(&m_mux);
    const bool valid = m_anchorValid;
    utcNs            = m_anchorUtcNs;
    monoUs           = m_anchorMonoUs;
    anchorPhc        = m_anchorPhcNs;
    ratePpb          = m_ratePpb;
    portEXIT_CRITICAL(&m_mux);

    if (!valid) {
        return false;
    }

    /* The 1588 counter is the served base while it runs: a reply's arrival
     * stamp is taken by hardware in that base, and a served time computed in
     * another one would have to be converted back on every packet. A stale
     * anchor stamp -- the pair and the anchor come from the same edge, so this
     * is only the first second of a boot -- falls through to the monotonic
     * path. */
    if (ptp::hardware() && anchorPhc != 0) {
        const int64_t now       = ptp::nowNs();
        const int64_t elapsedNs = now - anchorPhc;
        if (elapsedNs >= 0 && elapsedNs < 2000000000LL) {
            static bool told = false;
            if (!told) {
                told = true;
                ESP_LOGI(TAG, "phc anchor: phc elapsed %lld ns, mono elapsed %lld ns",
                         static_cast<long long>(elapsedNs),
                         static_cast<long long>(esp_timer_get_time() * 1000 - monoUs * 1000));
            }
            /* Once the counter itself is being steered by this same fit, the
             * elapsed part is already in real seconds and scaling it again
             * would take the correction out twice. Until the first push lands
             * it is not, and the scaling is what makes the served time rate
             * right. Nanoseconds here, so the divisor is 1e9 and not the 1e6
             * of the microsecond path below. */
            const int64_t totalNs =
                ptp::steered() ? utcNs + elapsedNs
                               : utcNs + elapsedNs - elapsedNs * ratePpb / 1000000000;
            ts.tv_sec             = static_cast<time_t>(totalNs / 1000000000);
            ts.tv_nsec            = static_cast<long>(totalNs % 1000000000);
            return true;
        }
    }

    /* The anchor plus elapsed monotonic time, with the servo's rate taken out of
     * the elapsed part: positive ppb means our counter runs fast, so a GPS
     * second takes slightly fewer of its microseconds than it claims and the
     * elapsed time has to be handed back a little. Reading the counter (1 us)
     * rather than CLOCK_REALTIME also means a slew in flight cannot make a
     * timestamp go backwards between two replies.
     *
     * The unit of the correction is the easy thing to get wrong by a thousand:
     * elapsed_us * 1000 is nanoseconds, times ppb * 1e-9 is parts, so the
     * divisor is 1e6 and not 1e3.
     *
     * With no rate yet -- the first minute -- this is exactly what it was
     * before: the anchor plus raw elapsed time. */
    const int64_t elapsedUs = esp_timer_get_time() - monoUs;
    const int64_t totalNs   = utcNs + elapsedUs * 1000 - elapsedUs * ratePpb / 1000000;
    ts.tv_sec               = static_cast<time_t>(totalNs / 1000000000);
    ts.tv_nsec              = static_cast<long>(totalNs % 1000000000);
    return true;
}
