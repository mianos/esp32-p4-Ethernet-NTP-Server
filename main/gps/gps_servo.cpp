/* Compiles this file's ESP_LOGD lines in; they stay hidden until raised with
 * the console's `log_level debug <tag>`. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

/* Anchoring, the rate fit, served time, the error budget and the system clock
 * slew. */
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
    /* pps_latency keys edges by the stamp before the qErr correction. */
    const int64_t rawPpsMonoUs = ppsMonoUs;
    /* Quantisation correction: edge -= qErr. TIM-TP describes the next pulse, so
     * a qErr from 50 ms after to 1.2 s before this edge applies to it. Needs the
     * UTC time base with UTC available. RAIM is ignored (gpsd checks it); this
     * module reports it active all the time. */
    if (m_qErrValid && (m_timTpFlags & 0x03) == 0x03 && m_qErrAtUs != 0) {
        const int64_t ageUs = ppsMonoUs - m_qErrAtUs;
        if (ageUs >= -50000 && ageUs <= 1200000) {
            ppsMonoUs -= m_qErrNs / 1000;
            if (ppsMonoNs != 0) {
                ppsMonoNs -= m_qErrNs;
            }
            /* And on the 1588 counter, so the two anchors agree. */
            if (ppsPtpNs != 0) {
                ppsPtpNs -= m_qErrNs;
            }
        }
    }
    portEXIT_CRITICAL(&m_mux);

    /* No edge, or this edge is already anchored. */
    if (ppsCount == 0 || ppsCount == m_pairedCount) {
        return;
    }

    const int64_t ageUs = esp_timer_get_time() - ppsMonoUs;
    if (ageUs < 0 || ageUs > kPairWindowUs) {
        /* Too long after the edge to be about it; pairing could be a second
         * out. */
        return;
    }

    m_pairedCount = ppsCount;

    /* Edge-to-frame delay for accepted pairs. */
    {
        const int64_t atUs = esp_timer_get_time();
        portENTER_CRITICAL(&m_mux);
        m_pairAgeUs    = ageUs;
        m_pairAgeMaxUs = ageUs > m_pairAgeMaxUs ? ageUs : m_pairAgeMaxUs;
        m_pairAgeAtUs  = atUs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* The servo's prediction for this edge: a rate check, and after a holdover
     * the error that was served during it. */
    int64_t predictedNs = 0;
    int64_t gapUs       = 0;
    bool    hadAnchor   = false;
    bool    rateKnown   = false;
    int64_t anchorUtcNs = 0;
    int32_t ratePpb     = 0;
    {
        portENTER_CRITICAL(&m_mux);
        rateKnown = m_rateSamples >= kRateMinSamples;
        if (m_anchorValid && ppsMonoUs > m_anchorMonoUs) {
            hadAnchor   = true;
            gapUs       = ppsMonoUs - m_anchorMonoUs;
            anchorUtcNs = m_anchorUtcNs;
            ratePpb     = m_ratePpb;
        }
        portEXIT_CRITICAL(&m_mux);
    }
    /* The rate term divides 64 bits, which is a libgcc call (__divdi3):
     * kept out of the section the PPS interrupt waits on. */
    if (hadAnchor) {
        predictedNs = anchorUtcNs + gapUs * 1000 - gapUs * ratePpb / 1000000;
    }

    /* An edge is used only if it is regular (about a second after the previous
     * edge) and continuous (the module's second advanced by the seconds that
     * passed). A refused edge is counted and does not anchor, fit or steer.
     * Spacing is from the previous edge, not the anchor, which a refused edge
     * leaves behind. */
    const int64_t edgeGapUs = m_lastEdgeMonoSet ? ppsMonoUs - m_lastEdgeMonoUs : 0;
    const bool    regular =
        !m_lastEdgeMonoSet || (edgeGapUs >= kEdgeRegularMinUs && edgeGapUs <= kEdgeRegularMaxUs);
    /* Against the seconds elapsed, so an edge after an outage can pass. */
    const int64_t expectedS = m_lastEdgeMonoSet ? (edgeGapUs + 500000) / 1000000 : 1;
    const bool    advance =
        m_lastEdgeSecondSet &&
        static_cast<int64_t>(second) == static_cast<int64_t>(m_lastEdgeSecond) + expectedS;
    /* An edge ending a gap of kRateGapNs or more is accepted despite its
     * spacing; pushRateSample() restarts the fit across it. */
    const bool resumed = m_lastEdgeMonoSet && edgeGapUs >= kRateGapNs / 1000;
    const bool spaced  = advance && (regular || resumed);

    /* Distance from the fit, for the outlier test (kEdgeOutlierNs). Not applied
     * after a holdover, where this error is what is being measured. */
    const int64_t errorNs = static_cast<int64_t>(second) * 1000000000 - predictedNs;

    /* Log a late interrupt or a large fit error side by side: an error that
     * follows the latency is the CPU, one without it is the pulse. Latency is
     * shown against the floor, since the fixed part is absorbed by the fit. */
    if (rateKnown) {
        int64_t            latencyNs = 0;
        pps_latency::Stats probe     = {};
        if (pps_latency::forEdge(rawPpsMonoUs, &latencyNs) && pps_latency::stats(&probe) &&
            (latencyNs - probe.minNs >= 2000 ||
             (hadAnchor && (errorNs >= 4000 || errorNs <= -4000)))) {
            ESP_LOGW(TAG, "edge latency %lld ns (floor %lld), %+lld ns from the fit",
                     static_cast<long long>(latencyNs), static_cast<long long>(probe.minNs),
                     static_cast<long long>(errorNs));
        }
    }

    const bool outlier = spaced && hadAnchor && rateKnown && gapUs < kLockWindowUs &&
                         (errorNs > kEdgeOutlierNs || errorNs < -kEdgeOutlierNs);
    const bool trusted = spaced && !outlier;

    /* Count spikes (kEdgeSpikeNs) outside holdover. */
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
    m_anchorMonoNs = ppsMonoNs;
    /* The anchor on the 1588 counter, which receive stamps also use; zero when
     * it is off, and now() falls back to esp_timer. */
    m_anchorPhcNs = ppsPtpNs;
    m_anchorValid = true;
    m_lastFix     = second;
    portEXIT_CRITICAL(&m_mux);

    /* Step the 1588 counter onto the edge when its phase is off. The PHC anchor
     * is dropped after a step, so served time uses esp_timer for one edge. */
    if (ppsPtpNs != 0) {
        m_phcPhaseNs = ppsPtpNs - static_cast<int64_t>(second) * 1000000000;
        /* The edge after a step shows what the read-to-set gap lost (about
         * 20 us). A landing beyond kPhcStepLagMaxNs is not learned from. */
        if (m_phcJustStepped) {
            m_phcJustStepped = false;
            if (m_phcPhaseNs > -kPhcStepLagMaxNs && m_phcPhaseNs < kPhcStepLagMaxNs) {
                m_phcStepLagNs = std::clamp<int64_t>(m_phcStepLagNs + m_phcPhaseNs,
                                                     -kPhcStepLagMaxNs, kPhcStepLagMaxNs);
            }
        }
        m_phcOffEdges = m_phcPhaseNs > kPhcPhaseStepNs || m_phcPhaseNs < -kPhcPhaseStepNs
                            ? m_phcOffEdges + 1
                            : 0;
        /* Only once the rate is pushed; unsteered, the counter drifts ~30 ppm. */
        const int64_t stepNs = -m_phcPhaseNs - m_phcStepLagNs;
        if (m_phcOffEdges >= kPhcPhaseEdges && ptp::steered() && ptp::stepBy(stepNs) == ESP_OK) {
            ESP_LOGI(TAG, "phc stepped %+lld ns onto the edge (%+lld ns of it the set's lag)",
                     static_cast<long long>(stepNs), static_cast<long long>(-m_phcStepLagNs));
            m_phcOffEdges    = 0;
            m_phcJustStepped = true;
            ++m_phcSteps;
            portENTER_CRITICAL(&m_mux);
            m_anchorPhcNs = 0;
            portEXIT_CRITICAL(&m_mux);
        }
    }

    if (gapUs >= kLockWindowUs) {
        /* Record what the holdover cost. */
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

    pushRateSample(ppsMonoNs != 0 ? ppsMonoNs : ppsMonoUs * 1000,
                   static_cast<int64_t>(second) * 1000000000);

    steer();
}

int64_t GpsTime::rootDelayNs() const
{
    /* A local refclock has no path to the reference (ntpd also reports 0).
     * Stamping error goes in dispersion. */
    return 0;
}

int64_t GpsTime::sampleErrorNs() const
{
    /* Half the quantisation: the sawtooth is uniform about its centre. */
    return kIsrLatencyNs + (m_qErrValid ? std::abs(m_qErrNs) / 2 : 0);
}

int64_t GpsTime::unobservedNs() const
{
    /* While TIM-TP is configured but silent, the pulse's offset from its second
     * is unobserved (the anchor moves with it), so dispersion grows: 10 us/s
     * after 30 s, capped at 200 ms. Measured drift was 4.2 us/s. */
    if (m_timTpCfg != 1) {
        return 0;
    }
    /* In fixed mode TIM-TP stops, but a current, trusted edge is still bounded
     * by the quantisation clamp. Growing here would demote a working box for
     * good. */
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
    /* One "gap" record per missed second first. */
    if (m_lastSampleSet && epoch > m_lastSampleEpoch + 1) {
        uint32_t missing = epoch - m_lastSampleEpoch - 1;
        if (missing > kSampleRing) {
            missing = kSampleRing;
        }
        for (uint32_t i = 0; i < missing; ++i) {
            /* Unmeasured but served: record the bound in force then. Computed
             * outside the lock. */
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

    /* Computed outside the lock: it takes ServePath's. */
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
    /* No lock: the GPS task advances the sequence after a record is complete,
     * and each copy is checked against its own sequence in case it was
     * overwritten meanwhile. */
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
    m_lastRateOffsetSet = false;
    /* fitRate() does not update this until it has kRateMinSamples. */
    portENTER_CRITICAL(&m_mux);
    m_rateSamples = 0;
    portEXIT_CRITICAL(&m_mux);
}

void GpsTime::pushRateSample(int64_t monoNs, int64_t utcNs)
{
    const int64_t offsetNs = utcNs - monoNs;

    /* After a gap, drop the window and the spanning sample. */
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

    /* Least squares of UTC ns against edge ns, relative to the newest sample:
     * absolute UTC ns (~1.8e18) is beyond a double's integer precision. Order
     * does not matter, so the ring is read flat. */
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

    /* Push the rate into the 1588 counter when it moves by 500 ppb, and every
     * 30 s regardless, since a MAC restart loses it. */
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

    /* RMS residual from the line is the stamping jitter used in dispersion;
     * the worst residual is the TIE maximum. */
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

    {
        portENTER_CRITICAL(&m_mux);
        m_tieMaxNs = tieMax;
        m_tieRmsNs = residualNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* Logged when first known and when it moves by 500 ppb. */
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
        const int64_t offsetUs = offsetNs / 1000;
        portENTER_CRITICAL(&m_mux);
        m_offsetUs = offsetUs;
        m_offsetNs = offsetNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* Recorded against the system clock, which is what the servo holds. */
    recordSample(static_cast<uint32_t>(gps.tv_sec), offsetNs);

    /* Measured either way, but only the owner of CLOCK_REALTIME corrects it. */
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
        /* The 1588 counter is stepped too, so its anchor is stale until the
         * next edge. */
        m_anchorPhcNs = 0;
        portEXIT_CRITICAL(&m_mux);
        if (ptp::hardware()) {
            (void)ptp::stepToRealtime();
        }

        /* Steps below kRateResetNs do not trip pushRateSample()'s detector, and
         * would hold tieMaxNs over the lock bound until the window flushed. */
        resetRateWindow();

        /* The set lands tens of us late; slew the residue out now. */
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

    slew(offsetNs);
}

void GpsTime::slew(int64_t offsetNs)
{
    /* IDF's adjtime() slews at 1/64 of elapsed time, and a new call replaces
     * the remaining adjustment. */
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

    /* Served from the 1588 counter, the base hardware receive stamps use. With
     * no PHC anchor (boot, or just after a step) esp_timer is used. */
    if (ptp::hardware() && anchorPhc != 0) {
        const int64_t now       = ptp::nowNs();
        const int64_t elapsedNs = now - anchorPhc;
        if (elapsedNs >= 0 && elapsedNs < 2000000000LL) {
            static bool told = false;
            if (!told) {
                told = true;
                ESP_LOGD(TAG, "phc anchor: phc elapsed %lld ns, mono elapsed %lld ns",
                         static_cast<long long>(elapsedNs),
                         static_cast<long long>(esp_timer_get_time() * 1000 - monoUs * 1000));
            }
            /* Once the counter is steered its elapsed time is already rate
             * corrected; before that, correct it here (ns, so / 1e9). */
            const int64_t totalNs =
                ptp::steered() ? utcNs + elapsedNs
                               : utcNs + elapsedNs - elapsedNs * ratePpb / 1000000000;
            ts.tv_sec             = static_cast<time_t>(totalNs / 1000000000);
            ts.tv_nsec            = static_cast<long>(totalNs % 1000000000);
            return true;
        }
    }

    /* Anchor plus esp_timer elapsed, less the rate error (positive ppb runs
     * fast). elapsed_us * ppb / 1e6 is ns. Not CLOCK_REALTIME, so a slew cannot
     * make replies go backwards. */
    const int64_t elapsedUs = esp_timer_get_time() - monoUs;
    const int64_t totalNs   = utcNs + elapsedUs * 1000 - elapsedUs * ratePpb / 1000000;
    ts.tv_sec               = static_cast<time_t>(totalNs / 1000000000);
    ts.tv_nsec              = static_cast<long>(totalNs % 1000000000);
    return true;
}
