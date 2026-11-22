/*
 * What this reference says about itself: the lock decision, the status the
 * HTTP side reads, the facts a reply is built from, and the TimeRef adapter.
 */
#include "gps/gps_internal.h"

using namespace gps_detail;

uint8_t GpsTime::copySats(Sat *out, uint8_t max) const
{
    /* The rows are rewritten by the GPS task on a 2 s message, bracketed by two
     * bumps of m_satsSeq, so a copy taken while the count was odd or moving is
     * a mix of two skies and is taken again. No lock: /gps/sats is the caller,
     * and it has no business holding the PPS interrupt off. */
    uint8_t n = 0;
    for (int attempt = 0; attempt < 10; ++attempt) {
        const uint32_t before = m_satsSeq.load(std::memory_order_acquire);
        if ((before & 1u) == 0) {
            n = m_satCount < max ? m_satCount : max;
            for (uint8_t i = 0; i < n; ++i) {
                out[i] = m_sats[i];
            }
            if (m_satsSeq.load(std::memory_order_acquire) == before) {
                return n;
            }
        }
        vTaskDelay(1);
    }
    return n;
}

bool GpsTime::evaluateLock(const LockInputs &in) const
{
    /* Lock needs all three: a fix to trust the date, a valid NAV-PVT to know
     * which second, and an edge recent enough that the clock is still being
     * corrected. Fix type 2 is the floor -- "time-only" (5) dates the second but
     * does not put the receiver anywhere, and this reference does not want to
     * explain that difference in its stratum. */
    if (!(in.anchored && in.timeValid && in.fixType >= 2 && in.ppsAgeUs >= 0 &&
          in.ppsAgeUs < in.lockWindowUs)) {
        return false;
    }

    /* Age is not enough on its own: it says an edge arrived, not that the clock
     * is being held where it belongs. Further tests, each of which demotes to
     * stratum 16 rather than serving on a claim that has stopped being true:
     *
     *  - the newest edge passed the trust test. A pulse that was corrupt, or a
     *    second that did not advance, means this second's anchor is a guess;
     *  - the receiver is not reporting critical jamming. Not gated on freshness:
     *    a jammed front end can hold a fix and keep emitting a pulse while the
     *    solution underneath it drifts, so the pulse still looks regular. MON-HW
     *    carries the state and this module does not send it, so on this board
     *    the test is dormant -- taken from Baumart's server, which demotes on it
     *    unconditionally;
     *  - while the edge is fresh, the phase sits inside the band. A servo still
     *    ringing out from a step has edges arriving perfectly and an offset in
     *    the hundreds of milliseconds;
     *  - while the edge is fresh and a rate is known, the fit's worst residual
     *    is inside the jitter bound. That is the slope test: a window assembled
     *    across a fault fits a line through noise, and the residual is what
     *    gives it away. Skipped in a holdover, where the phase is not measured
     *    at all and the residual is by definition the gap.
     *
     * The offset and residual tests wait until a rate exists, so the first
     * minute still locks on a fresh, trusted edge as it always did. */
    const bool fresh = in.ppsAgeUs <= kLockWindowUs;
    const bool bad   = !in.trusted || in.jamState == 3 ||
                     (fresh && (in.offsetNs > kLockOffsetNs || in.offsetNs < -kLockOffsetNs)) ||
                     (fresh && in.rateKnown && in.tieMaxNs > kLockTieMaxNs);

    /* Debounced per edge, not per call: this runs on every reply and every
     * health poll, so a counter advanced per call would saturate inside a
     * second. the count kept in m_gate makes it one vote per edge. A single glitched pulse,
     * or one sample outside the band on its way back from a step, is then not
     * worth a demotion and a relock line -- three consecutive seconds is. */
    /* One word, so the vote needs no lock: the edge count in the low 24 bits
     * and the streak above it. Only a count newer than the last one votes -- a
     * status read working from a snapshot must not vote a second time for an
     * edge a reply has already counted. */
    uint32_t badFor = 0;
    uint32_t cur    = m_gate.load(std::memory_order_relaxed);
    for (;;) {
        const uint32_t streak = cur >> 24;
        const uint32_t ahead  = (in.ppsCount - cur) & 0xFFFFFFu;
        if (ahead == 0 || ahead >= 0x800000u) {
            badFor = streak;
            break;
        }
        const uint32_t voted = bad ? std::min<uint32_t>(streak + 1, 255) : 0;
        if (m_gate.compare_exchange_weak(cur, (voted << 24) | (in.ppsCount & 0xFFFFFFu),
                                         std::memory_order_relaxed)) {
            badFor = voted;
            break;
        }
    }
    return badFor < kGateStreak;
}

void GpsTime::readEdge(EdgeWords &out) const
{
    /* The interrupt is the only writer and brackets its update with two bumps
     * of m_edgeSeq, so an odd value or a changed one means it ran meanwhile and
     * the read is taken again. No lock: this is what lets the status be read
     * without ever holding the interrupt off. */
    for (;;) {
        const uint32_t before = m_edgeSeq.load(std::memory_order_acquire);
        if ((before & 1u) == 0) {
            out.count  = m_ppsCount;
            out.monoUs = m_ppsMonoUs;
            out.ptpNs  = m_ppsPtpNs;
            out.lastUs = m_lastEdgeUs;
            if (m_edgeSeq.load(std::memory_order_acquire) == before) {
                return;
            }
        }
    }
}

void GpsTime::publishStatus()
{
    /* One complete status, written into the next slot of a small ring and then
     * named as the newest. A reader copies the slot that is named and never
     * waits: the slot it is reading is not written again for kStatusSlots - 1
     * seconds, and nothing here or there takes the PPS interrupt's lock.
     *
     * Called from the GPS task only, just after a NAV-PVT has been handled --
     * a couple of hundred milliseconds past the edge, with the next one most
     * of a second away. Nearly every field is this task's own; the few that are
     * not are read by the two means below. */
    const uint32_t next = (m_statusHead.load(std::memory_order_relaxed) + 1) % kStatusSlots;
    StatusSlot    &slot = m_statusRing[next];
    const uint32_t seq  = slot.seq.load(std::memory_order_relaxed);
    slot.seq.store(seq + 1, std::memory_order_release); /* odd: being written */

    StatusData   &st = slot.data;
    StatusExtras &ex = slot.extra;
    st               = StatusData{};

    {
        const uint32_t lo = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        const uint32_t hi =
            systimer_ll_get_counter_value_high(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        st.ticksPerUs = static_cast<uint32_t>(systimer_us_to_ticks(1));
        st.tickUsDelta =
            static_cast<int64_t>(systimer_ticks_to_us((static_cast<uint64_t>(hi) << 32) | lo)) -
            esp_timer_get_time();
    }

    EdgeWords edge;
    readEdge(edge);
    st.ppsCount   = edge.count;
    st.ppsPtpNs   = edge.ptpNs;
    ex.ppsMonoUs  = edge.monoUs;
    ex.lastEdgeUs = edge.lastUs;
    /* Written by whoever arms the holdover test, as two words: read until two
     * reads agree rather than locked. */
    do {
        ex.suppressUntil = m_ppsSuppressUntilUs;
    } while (ex.suppressUntil != m_ppsSuppressUntilUs);

    ex.anchorPhcNs       = m_anchorPhcNs;
    st.timeValid         = m_timeValid;
    st.fixType           = m_fixType;
    st.satellites        = m_satellites;
    st.anchored          = m_anchorValid;
    st.lastFix           = m_lastFix;
    st.offsetUs          = m_offsetUs;
    st.offsetNs          = m_offsetNs;
    st.frames            = m_decoder.frames();
    st.droppedChecksum   = m_decoder.droppedChecksum();
    st.droppedLength     = m_decoder.droppedLength();
    st.rescans           = m_rescans;
    st.bridgeDroppedCrc  = m_bridgeDroppedCrc;
    st.ratePpb           = m_ratePpb;
    st.rateSamples       = m_rateSamples;
    st.leapSeconds       = m_leapSeconds;
    st.leapChange        = m_leapChange;
    st.leapInS           = m_leapInS;
    st.leapValid         = m_leapValid;
    st.lastHoldoverUs    = m_lastHoldoverUs;
    st.lastHoldoverErrNs = m_lastHoldoverErrNs;
    st.timTpCount        = m_timTpCount;
    st.qErrNs            = std::abs(m_qErrNs);
    st.qErrValid         = m_qErrValid;
    st.jitterNs          = m_jitterNs;
    st.pairAgeUs         = m_pairAgeUs;
    st.pairAgeMaxUs      = m_pairAgeMaxUs;
    ex.pairAgeAtUs       = m_pairAgeAtUs;
    st.pulseOnSecond     = m_pulseOnSecond;
    st.pulseOffsetNs     = m_pulseOffsetNs;
    st.pvtNanoNs         = m_pvtNanoNs;
    st.pvtItowMs         = m_pvtItowMs;
    st.timTpFlags        = m_timTpFlags;
    st.tosFlags          = m_tosFlags;
    st.tosUtcOffsetNs    = m_tosUtcOffset;
    st.tosUtcUncNs       = m_tosUtcUnc;
    st.tosCfg            = static_cast<uint8_t>(m_tosCfg);
    st.timTpTimN         = m_timTpTimN;
    st.timTpNavN         = m_timTpNavN;
    st.rateCfg           = static_cast<uint8_t>(m_rateCfg);
    st.nav5Cfg           = static_cast<uint8_t>(m_nav5Cfg);
    st.tp5CableNs        = m_tp5CableNs;
    st.tp5UserNs         = m_tp5UserNs;
    st.tp5Flags          = m_tp5Flags;
    st.tp5PeriodUs       = m_tp5Period;
    st.tp5Ratio          = m_tp5Ratio;
    st.tp5Rejected       = m_tp5Naked;
    st.timTpRejected     = m_timTpNaked;
    st.timTpAcked        = m_timTpAcked;
    st.antBiasOn         = m_antBiasOn;
    st.antShort          = m_antShort;
    st.antLatched        = m_antLatched;
    st.antCfgAcked       = m_antCfgAcked;
    st.antCfgRejected    = m_antCfgNaked;
    st.antStatus         = m_antStatus;
    st.antPower          = m_antPower;
    st.noisePerMs        = m_noisePerMs;
    st.agcCnt            = m_agcCnt;
    st.jamInd            = m_jamInd;
    st.jamState          = m_jamState;
    st.monHwValid        = m_monHwValid;
    st.fitUs             = m_fitUs;
    st.gDop              = m_gDop;
    st.pDop              = m_pDop;
    st.hDop              = m_hDop;
    st.vDop              = m_vDop;
    st.clkBiasNs         = m_clkBiasNs;
    st.clkDriftNsPerS    = m_clkDriftNsPerS;
    st.tAccNs            = m_tAccNs;
    st.fAccPsPerS        = m_fAccPsPerS;
    st.navStatusFlags    = m_navStatusFlags;
    st.navStatusFlags2   = m_navStatusFlags2;
    st.ttffMs            = m_ttffMs;
    st.rfBands           = m_rfBands;
    st.rfJamWorst        = m_rfJamWorst;
    st.rfAntWorst        = m_rfAntWorst;
    st.rfAgcMin          = m_rfAgcMin;
    st.rfSeen            = m_rfSeen;
    st.rfCfg             = static_cast<uint8_t>(m_rfCfg);
    st.tieMaxNs          = m_tieMaxNs;
    st.tieRmsNs          = m_tieRmsNs;
    st.ioRxBytes         = m_ioRxBytes;
    st.ioTxBytes         = m_ioTxBytes;
    st.ioParity          = m_ioParity;
    st.ioFraming         = m_ioFraming;
    st.ioOverrun         = m_ioOverrun;
    st.rbufPending       = m_rbufPending;
    st.rbufPeak          = m_rbufPeak;
    st.tbufPending       = m_tbufPending;
    st.tbufPeak          = m_tbufPeak;
    st.tbufErrors        = m_tbufErrors;
    st.msgUbx            = m_msgUbx;
    ex.recentAt          = m_recentAt;
    st.msgSkipped        = m_msgSkipped;
    st.ioSeen            = m_ioSeen;
    st.baud              = m_baud;
    ex.ioAtUs            = m_ioAtUs;
    st.servoSteps        = m_servoSteps;
    st.phcPhaseNs        = m_phcPhaseNs;
    st.phcSteps          = m_phcSteps;
    st.servoClamps       = m_servoClamps;
    st.edgeSpikes        = m_edgeSpikes;
    st.edgeOutliers      = m_edgeOutliers;
    st.edgeRejects       = m_edgeRejects;
    st.lastEdgeTrusted   = m_lastEdgeTrusted;
    ex.dopAtUs           = m_dopAtUs;
    ex.clockAtUs         = m_clockAtUs;
    ex.navStatusAtUs     = m_navStatusAtUs;
    ex.rfAtUs            = m_rfAtUs;
    st.bridgeSessions    = m_bridgeSessions;
    st.bridgeRefused     = m_bridgeRefused;
    st.bridgeActive      = m_bridgeActive;
    st.bridgeClients     = m_bridgeClients;
    st.bridgeRx          = m_bridgeRx;
    st.bridgeTx          = m_bridgeTx;
    /* Named rather than numeric: the interesting thing is which constellations
     * are being solved, and a reader should not have to hold the gnssId table
     * in their head to find out. */
    ex.gnssMask       = m_gnssMask;
    ex.gnssNaked      = m_gnssNaked;
    ex.gnssAcked      = m_gnssAcked;
    ex.tmode          = m_tmode;
    ex.svinDur        = m_svinDur;
    ex.svinAcc        = m_svinAcc;
    ex.svinObs        = m_svinObs;
    st.svinMeanCm[0]  = m_svinMean[0];
    st.svinMeanCm[1]  = m_svinMean[1];
    st.svinMeanCm[2]  = m_svinMean[2];
    ex.svinDurS       = m_svinDurS;
    ex.svinAccMm      = m_svinAccMm;
    st.svinValid      = m_svinValid;
    st.holdValidated  = m_holdValidated;
    st.holdWanted     = m_holdWanted;
    ex.holdAssertAtUs = m_holdAssertAtUs;
    ex.satCount       = m_satCount;
    ex.svinAtUs       = m_svinAtUs;
    st.pinSel         = m_pinSel;
    st.pinDir         = m_pinDir;
    st.pinVal         = m_pinVal;
    st.cn0Max         = m_cn0Max;
    st.cn0Mean        = m_cn0Mean;
    st.cn0Used        = m_cn0Used;
    ex.navSatAtUs     = m_navSatAtUs;
    ex.monHwAtUs      = m_monHwAtUs;
    ex.anchorMonoUs   = m_anchorMonoUs;

    /* The two clocks against the anchor. The reads have to be adjacent, so the
     * counter is read either side of the timer and the pair is kept only if
     * nothing got in between. */
    if (st.anchored && ex.anchorPhcNs != 0 && ptp::hardware()) {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const int64_t phcBefore = ptp::nowNs();
            const int64_t monoNow   = esp_timer_get_time();
            const int64_t phcAfter  = ptp::nowNs();
            if (phcAfter - phcBefore < 2000) {
                st.phcMonoDeltaNs = ((phcBefore + phcAfter) / 2 - ex.anchorPhcNs) -
                                    (monoNow - ex.anchorMonoUs) * 1000;
                break;
            }
        }
    }

    for (int i = 0; i < 16; ++i) {
        ex.recentFrames[i] = m_recentFrames[i];
    }
    /* The bridge task writes two of these; the worst a copy can be is a line
     * caught mid-update, and each buffer keeps its terminator. */
    std::memcpy(ex.msgpp, m_msgppHex.data(), sizeof(ex.msgpp));
    std::memcpy(ex.io, m_ioHex.data(), sizeof(ex.io));
    std::memcpy(ex.module, m_moduleId.data(), sizeof(ex.module));
    std::memcpy(ex.bridgeEnd, m_bridgeEnd.data(), sizeof(ex.bridgeEnd));
    std::memcpy(ex.bridgeLast, m_bridgeLast.data(), sizeof(ex.bridgeLast));

    ex.qErrAtUs    = m_qErrAtUs;
    ex.publishedUs = esp_timer_get_time();
    ex.number      = ++m_statusNumber;

    slot.seq.store(seq + 2, std::memory_order_release);
    m_statusHead.store(next, std::memory_order_release);
    m_statusPublishedUs = ex.publishedUs;
    m_statusDue         = false;
}

GpsTime::Status GpsTime::status() const
{
    Status       st;
    StatusExtras ex;

    /* The newest finished snapshot, copied with no lock. The check afterwards
     * is for a reader descheduled for seconds in the middle of the copy, which
     * is the only way the writer can come round to the same slot. */
    for (int attempt = 0; attempt < 4; ++attempt) {
        const StatusSlot &slot        = m_statusRing[m_statusHead.load(std::memory_order_acquire)];
        const uint32_t    before      = slot.seq.load(std::memory_order_acquire);
        static_cast<StatusData &>(st) = slot.data;
        ex                            = slot.extra;
        if ((before & 1u) == 0 && slot.seq.load(std::memory_order_acquire) == before) {
            break;
        }
    }

    /* The edge itself is read live, by the same lock-free means the snapshot
     * used: the snapshot is up to a second old, and an edge that has arrived
     * since should not read as a second of age. */
    EdgeWords edge;
    readEdge(edge);
    st.ppsCount   = edge.count;
    st.ppsPtpNs   = edge.ptpNs;
    ex.ppsMonoUs  = edge.monoUs;
    ex.lastEdgeUs = edge.lastUs;

    const int64_t ppsMonoUs      = ex.ppsMonoUs;
    const int64_t anchorMonoUs   = ex.anchorMonoUs;
    const int64_t suppressUntil  = ex.suppressUntil;
    const int64_t navSatAtUs     = ex.navSatAtUs;
    const int64_t pairAgeAtUs    = ex.pairAgeAtUs;
    const int64_t monHwAtUs      = ex.monHwAtUs;
    const int64_t dopAtUs        = ex.dopAtUs;
    const int64_t clockAtUs      = ex.clockAtUs;
    const int64_t navStatusAtUs  = ex.navStatusAtUs;
    const int64_t rfAtUs         = ex.rfAtUs;
    const int64_t ioAtUs         = ex.ioAtUs;
    const int64_t holdAssertAtUs = ex.holdAssertAtUs;
    const int64_t lastEdgeUs     = ex.lastEdgeUs;
    const int64_t svinAtUs       = ex.svinAtUs;
    const uint8_t recentAt       = ex.recentAt;
    const uint8_t gnssMask       = ex.gnssMask;
    const bool    gnssNaked      = ex.gnssNaked;
    const bool    gnssAcked      = ex.gnssAcked;
    const uint8_t tmode          = ex.tmode;
    const uint8_t satCount       = ex.satCount;

    text::getList(Settings::instance().antDelayNs(), ',', st.antDelayNs);
    text::getList(Settings::instance().gpsUserDelayNs(), ',', st.userDelayNs);

    st.svinDurS    = static_cast<double>(ex.svinDur);
    st.svinAccMm   = static_cast<double>(isqrt(ex.svinAcc));
    st.svinVarMm2  = static_cast<double>(ex.svinAcc);
    st.svinObs     = static_cast<double>(ex.svinObs);
    st.svinTargetS = static_cast<double>(ex.svinDurS);
    st.svinLimitMm = static_cast<double>(ex.svinAccMm);

    st.msgppHex.assign(ex.msgpp, ::strnlen(ex.msgpp, sizeof(ex.msgpp) - 1));
    st.ioHex.assign(ex.io, ::strnlen(ex.io, sizeof(ex.io) - 1));
    st.moduleId.assign(ex.module, ::strnlen(ex.module, sizeof(ex.module) - 1));
    st.bridgeEnd.assign(ex.bridgeEnd, ::strnlen(ex.bridgeEnd, sizeof(ex.bridgeEnd) - 1));
    st.bridgeLast.assign(ex.bridgeLast, ::strnlen(ex.bridgeLast, sizeof(ex.bridgeLast) - 1));
    {
        static constexpr const char *kNames[] = {"GPS",  "SBAS", "Galileo", "BeiDou",
                                                 "IMES", "QZSS", "GLONASS"};
        for (int i = 0; i < 7; ++i) {
            if ((gnssMask >> i) & 1u) {
                if (!st.gnss.empty()) {
                    st.gnss += ',';
                }
                st.gnss += kNames[i];
            }
        }
    }
    st.gnssCfg  = gnssNaked ? "rejected" : gnssAcked ? "accepted" : "silent";
    st.timeMode = tmode == 1 ? "survey" : tmode == 2 ? "fixed" : "moving";

    /* Sixteen std::format calls are sixteen heap operations, so they happen
     * here, on the reader's copy. */
    st.recentFrames.clear();
    for (uint8_t i = 0; i < 16; ++i) {
        const uint8_t at = static_cast<uint8_t>((recentAt + 15 - i) % 16);
        if (ex.recentFrames[at] == 0) {
            continue;
        }
        st.recentFrames +=
            std::format("{:02x}:{:02x} ", ex.recentFrames[at] >> 8, ex.recentFrames[at] & 0xFF);
    }

    const int64_t nowUs = esp_timer_get_time();
    if (st.ppsCount != 0) {
        st.ppsAgeUs = nowUs - ppsMonoUs;
    }
    if (st.anchored) {
        st.anchorAgeUs = nowUs - anchorMonoUs;
    }
    st.snapshotSeq   = ex.number;
    st.snapshotAgeUs = ex.publishedUs != 0 ? nowUs - ex.publishedUs : -1;
    st.holdAssertInS =
        holdAssertAtUs == 0 ? -1 : (static_cast<double>(holdAssertAtUs - nowUs) / 1e6);
    st.lastEdgeAgeMs = lastEdgeUs < 0 ? -1 : (static_cast<double>(nowUs - lastEdgeUs) / 1e3);
    st.holdReady =
        st.fixType >= 2 && satCount >= 4 && lastEdgeUs > 0 && nowUs - lastEdgeUs < 3000000;
    st.svinAgeS = svinAtUs != 0 ? static_cast<double>(nowUs - svinAtUs) / 1e6 : -1;
    if (suppressUntil > nowUs) {
        st.ppsSuppressUs = suppressUntil - nowUs;
    }
    if (navSatAtUs != 0) {
        st.navSatAgeUs = nowUs - navSatAtUs;
    }
    if (monHwAtUs != 0) {
        st.monHwAgeUs = nowUs - monHwAtUs;
    }
    if (dopAtUs != 0) {
        st.dopAgeUs = nowUs - dopAtUs;
    }
    if (clockAtUs != 0) {
        st.clockAgeUs = nowUs - clockAtUs;
    }
    if (navStatusAtUs != 0) {
        st.navStatusAgeUs = nowUs - navStatusAtUs;
    }
    if (rfAtUs != 0) {
        st.rfAgeUs = nowUs - rfAtUs;
    }
    if (ioAtUs != 0) {
        st.ioAgeUs = nowUs - ioAtUs;
    }

    st.pairAgeS          = pairAgeAtUs != 0 ? static_cast<double>(nowUs - pairAgeAtUs) / 1e6 : -1;
    const bool rateKnown = st.rateSamples >= kRateMinSamples;

    /* Two windows, and the difference between them is the servo. Without a rate
     * there is nothing to hold over with, so three missed edges demote; with one
     * the crystal's known error is taken out of every timestamp and ten minutes
     * of holdover is worth serving, with the claim decaying as it ages. */
    st.lockWindowUs =
        (rateKnown && st.timeValid && st.fixType >= 2) ? kHoldoverWindowUs : kLockWindowUs;

    /* The gate itself, and why each test is there, lives in evaluateLock(); it
     * carries the per-edge debounce too, which is why replyFacts() shares it
     * rather than repeating the predicates. */
    LockInputs in;
    in.anchored     = st.anchored;
    in.timeValid    = st.timeValid;
    in.fixType      = st.fixType;
    in.ppsAgeUs     = st.anchorAgeUs;
    in.trusted      = st.lastEdgeTrusted;
    in.offsetNs     = st.offsetNs;
    in.rateKnown    = rateKnown;
    in.tieMaxNs     = st.tieMaxNs;
    in.jamState     = st.jamState;
    in.ppsCount     = st.ppsCount;
    in.lockWindowUs = st.lockWindowUs;
    st.locked       = evaluateLock(in);

    if (st.locked) {
        /* What this reference states about itself, and where each term comes
         * from. The delay is the measurement path: the named interrupt latency
         * plus half the receiver's own stated pulse quantisation, which is what
         * it admits it does not know about where the edge is. The dispersion is
         * that, plus the jitter the stamps actually show, plus one ppm of the
         * anchor's age.
         *
         * Both used to be worse: delay was 0, which read as "the reference is
         * wired, so there is no path" while quietly ignoring the interrupt, and
         * dispersion was a flat ten microseconds no measurement stood behind.
         * This module rejects NAV-TIM-TP, so the quantisation term is absent --
         * but the jitter term is not, because it comes from the servo and not
         * from the message. */
        st.rootDelayNs = rootDelayNs();

        /* A pulse nobody is watching. While TIM-TP is agreeing to be sent and
         * has stopped arriving, the position of the edge against its own second
         * is unobserved -- the servo cannot see it either, because the anchor is
         * taken from the same edge and moves with it. Unobserved is not known:
         * the 133 ms failure of 18:43 was exactly this state, on a clock quoting
         * 2.4 us of dispersion, and chrony had to work it out for itself.
         *
         * So the stated uncertainty grows with how long the pulse has been
         * unobserved. 10 us a second covers the measured growth (4.2 us/s to
         * 133 ms) with margin, and the 30 s grace keeps a mode change from
         * widening it. Capped at 200 ms, which is six times the worst error seen
         * and still finite. */
        const int64_t lastTpUs = ex.qErrAtUs;
        /* Published inside the budget below as well as on its own, so a wide
         * bound can be told from a bad clock. */
        st.pulseUnobservedNs = unobservedNs();
        st.dispersionNs      = dispersionNs(st.anchorAgeUs, st.pulseUnobservedNs);
        if (m_timTpCfg == 1) {
            const int64_t silentS = (esp_timer_get_time() - lastTpUs) / 1000000;
            if (silentS > 30) {
                if (!m_pulseUnobservedLogged) {
                    m_pulseUnobservedLogged = true;
                    ESP_LOGW(TAG,
                             "TIM-TP silent for %lld s: the pulse position is unobserved, "
                             "widening dispersion from here",
                             static_cast<long long>(silentS));
                }
            } else if (m_pulseUnobservedLogged) {
                m_pulseUnobservedLogged = false;
                ESP_LOGI(TAG, "TIM-TP flowing again: dispersion back to its measured terms");
            }
        }
    }

    return st;
}

uint8_t GpsTime::leapIndicator() const
{
    int8_t  change = 0;
    int32_t inS    = 0;
    bool    valid  = false;

    portENTER_CRITICAL(&m_mux);
    change = m_leapChange;
    inS    = m_leapInS;
    valid  = m_leapValid;
    portEXIT_CRITICAL(&m_mux);

    /* RFC 5905 7.3: the indicator describes the last minute of the UTC day.
     * TIMELS counts down to the event, so "0 < timeToLsEvent <= 60" is that
     * minute, and a larger number means a leap is known about but is not yet
     * the client's problem. */
    if (!valid || change == 0 || inS <= 0 || inS > 60) {
        return 0;
    }
    return change > 0 ? 1 : 2;
}

std::string GpsTime::describe() const
{
    const Status st = status();

    if (st.locked) {
        if (st.anchorAgeUs > kLockWindowUs) {
            /* Still servable, and worth saying why in a line somebody reads
             * later: this is the holdover the servo bought. */
            return std::format("GPS/PPS, {} fix, {} satellites, holding over {:.1f} s",
                               fixText(st.fixType), st.satellites,
                               static_cast<double>(st.anchorAgeUs) / 1e6);
        }
        return std::format("GPS/PPS, {} fix, {} satellites", fixText(st.fixType), st.satellites);
    }
    if (st.fixType >= 2 && st.timeValid) {
        /* A fix but no edge we can trust: the receiver is fine and the pulse is
         * not, which is a different fault from "no fix" and worth saying
         * differently in the boot log. The refused case is not the same as the
         * missing one either -- edges are arriving, they just are not seconds --
         * and reading "no fresh edge" on a box whose LED is blinking would send
         * the reader to the wrong wire. */
        if (!st.lastEdgeTrusted) {
            return std::format("GPS/PPS, {} fix, {} edges refused in a row (spacing or "
                               "second continuity)",
                               fixText(st.fixType), st.edgeRejects);
        }
        return std::format("GPS/PPS, {} fix but no fresh PPS edge", fixText(st.fixType));
    }
    return "GPS/PPS (no fix yet)";
}

void GpsTime::logLockChange()
{
    const Status st = status();

    /* The fallback arbiter, once a second and from boot: a device whose antenna
     * is missing has to be able to hand over without anybody asking it to.
     *
     * What it is told is "the pulse is back and trustworthy", which is
     * deliberately *not* the lock. Lock also needs the phase inside
     * kLockOffsetNs, and while a fallback holds the clock the phase is the
     * distance between the upstream's clock and the receiver's -- it cannot come
     * back into the band until this servo steers again, and the servo is gated
     * off until the arbiter hands back. Judging the return on the lock alone
     * therefore never returns: each waits for the other. */
    const bool holding   = st.locked && st.anchorAgeUs > kLockWindowUs;
    const bool pulseGood = !holding && st.anchored && st.timeValid && st.fixType >= 2 &&
                           st.anchorAgeUs >= 0 && st.anchorAgeUs <= kLockWindowUs &&
                           st.lastEdgeTrusted;
    ClockSource::instance().observe(pulseGood);

    if (st.locked != m_wasLocked) {
        m_wasLocked = st.locked;
        if (st.locked) {
            ESP_LOGI(TAG, "locked: stratum 1 from here on (PPS edges so far: %u, rate %+ld ppb)",
                     static_cast<unsigned>(st.ppsCount), static_cast<long>(st.ratePpb));
        } else if (st.ppsCount == 0) {
            ESP_LOGW(TAG, "no PPS edges: is the module's timepulse wired to GPIO%d?",
                     static_cast<int>(kPpsPin));
        } else if (!st.lastEdgeTrusted) {
            /* Edges arriving and being refused is the one PPS fault that looks
             * healthy from a distance: the counter climbs and nothing works. */
            ESP_LOGW(TAG,
                     "not locked: %u PPS edges in a row refused (wrong spacing, or the "
                     "module's second did not advance with them)",
                     st.edgeRejects);
        } else {
            ESP_LOGW(TAG, "not locked (fix %s, last edge %lld ms ago): stratum 16 until it returns",
                     fixText(st.fixType), static_cast<long long>(st.ppsAgeUs / 1000));
        }
    }

    /* Inside a holdover nothing above changes for ten minutes, which would
     * otherwise look like a healthy clock that has quietly lost its PPS. Say it
     * when the edge goes stale, and say it again when it comes back. */
    if (holding != m_holdoverLogged) {
        m_holdoverLogged = holding;
        if (holding) {
            ESP_LOGW(TAG,
                     "holding over: no PPS for %.1f s, serving on the servo's rate (%+ld ppb) "
                     "with dispersion %lld us",
                     static_cast<double>(st.ppsAgeUs) / 1e6, static_cast<long>(st.ratePpb),
                     static_cast<long long>(st.dispersionNs / 1000));
        } else if (st.locked) {
            ESP_LOGI(TAG, "PPS back: the anchor is fresh again");
        }
    }

    /* A module that was narrating its pulse and has stopped. Not the same fault
     * as one that never did, and nothing on the wire shows it: the quantisation
     * term silently disappears from the budget. */
    const int64_t nowUs = esp_timer_get_time();
    if (st.timTpCount != m_timTpLastCount) {
        m_timTpLastCount  = st.timTpCount;
        m_timTpLastUs     = nowUs;
        m_timTpGoneLogged = false;
    } else if (st.timTpCount != 0 && !m_timTpGoneLogged && nowUs - m_timTpLastUs > 10 * 1000000) {
        m_timTpGoneLogged = true;
        /* The last qErr stays in the status for what it is worth, but stops
         * being counted: a figure from a message that has stopped arriving is a
         * stale claim. The delay drops back to the named latency; the jitter
         * does not, because it is measured from the anchors themselves and the
         * anchors are still coming. */
        portENTER_CRITICAL(&m_mux);
        m_qErrValid = false;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG,
                 "NAV-TIM-TP stopped 10 s ago: the pulse's quantisation is no longer stated, "
                 "root delay falls back to the named %lld us",
                 static_cast<long long>(kIsrLatencyNs / 1000));
    }
}

/* The most uncertainty this reference will serve through while still calling
 * itself locked.
 *
 * A healthy box states about 130 us of dispersion against a serve bound near
 * 40 us, so 400 us is ten times the bound and three times the healthy floor --
 * far enough out that only a real fault reaches it. Past it the reply becomes
 * stratum 16 / INIT / leap 3, which every client already reads as "ignore me".
 *
 * The alternative, which this box ran until now, is a stratum-1 badge held
 * through a 258 ms error with 19 ms of dispersion underneath it. chrony had to
 * work the falseticker out for itself; a client that trusts stratum alone would
 * have taken the 258 ms. */
constexpr int64_t kServeHonestNs = 400000;

/* What clients report about the path may widen the claim, never close the
 * gate: only the reference's own terms get to say it is no longer one. */
static_assert(ServePath::kCapNs <= kServeHonestNs / 2,
              "the client-measured path term must leave the gate to the reference");

GpsTime::ReplyFacts GpsTime::replyFacts() const
{
    ReplyFacts rf;
    LockInputs in;

    portENTER_CRITICAL(&m_mux);
    in.anchored  = m_anchorValid;
    in.timeValid = m_timeValid;
    in.fixType   = m_fixType;
    /* Since the last edge that became an anchor, not since the last edge. The
     * interrupt stamps every pulse whether or not a frame ever dated it, so
     * with the receiver's frames gone and its pulse still arriving the raw age
     * stayed under a second: "locked", fresh, and no wander charged, on an
     * anchor that was minutes old. */
    in.ppsAgeUs  = !m_anchorValid ? -1 : esp_timer_get_time() - m_anchorMonoUs;
    in.trusted   = m_lastEdgeTrusted;
    in.offsetNs  = m_offsetNs;
    in.rateKnown = m_rateSamples >= kRateMinSamples;
    in.tieMaxNs  = m_tieMaxNs;
    in.jamState  = m_jamState;
    in.ppsCount  = m_ppsCount;
    rf.lastSync  = m_lastFix;
    portEXIT_CRITICAL(&m_mux);

    in.lockWindowUs =
        (in.rateKnown && in.timeValid && in.fixType >= 2) ? kHoldoverWindowUs : kLockWindowUs;
    rf.locked  = evaluateLock(in);
    rf.stratum = rf.locked ? 1 : 16;
    /* "GPS" while it is a stratum-1 reference, "INIT" once it is not: a client
     * reading stratum 16 needs the refid to look like a refusal, not a
     * reference. */
    rf.refId = rf.locked ? kRefIdGps : kRefIdInit;
    /* The receiver's own leap notice, straight into the packet: only it knows
     * whether the end of the UTC day has 59, 60 or 61 seconds. */
    rf.leap = leapIndicator();
    if (rf.locked) {
        const int64_t budget = dispersionNs(in.ppsAgeUs, unobservedNs());
        if (budget > 0) {
            rf.rootDelayNs  = static_cast<uint32_t>(rootDelayNs());
            rf.dispersionNs = static_cast<uint32_t>(budget);
        }
        /* The gate. Dispersion is the one term the client cannot check for
         * itself, so it is also the one that decides whether this is a
         * reference or a guess. */
        if (budget > kServeHonestNs) {
            rf.locked  = false;
            rf.stratum = 16;
            rf.refId   = kRefIdInit;
            rf.demoted = true;
            if (!m_gateLogged) {
                m_gateLogged = true;
                ESP_LOGW(TAG,
                         "demoting: %lld us of dispersion is past the %lld us a reply may "
                         "claim -- serving stratum 16 until it closes",
                         static_cast<long long>(budget / 1000),
                         static_cast<long long>(kServeHonestNs / 1000));
            }
        } else if (m_gateLogged) {
            m_gateLogged = false;
            ESP_LOGI(TAG, "promoting: dispersion back to %lld us, serving stratum 1 again",
                     static_cast<long long>(budget / 1000));
        }
    }
    return rf;
}

namespace {

/* The GPS as the responder sees it. Stateless -- everything it answers comes
 * from the GpsTime singleton, which is older than any responder task. */
class GpsTimeRef : public TimeRef {
public:
    /* replyFacts(), not status(): status() assembles nine std::strings for the
     * HTTP side, and the responder was paying for every one of them between its
     * two stamps -- 250 us of allocation, measured, inside the window that
     * decides the T2/T3 pair. The budget's terms are the same ones status()
     * uses. */
    TimeReference reference() const override
    {
        const GpsTime::ReplyFacts rf = GpsTime::instance().replyFacts();

        TimeReference ref;
        ref.lastSync     = rf.lastSync;
        ref.locked       = rf.locked;
        ref.stratum      = rf.stratum;
        ref.refId        = rf.refId;
        ref.leap         = rf.leap;
        ref.rootDelayNs  = rf.rootDelayNs;
        ref.dispersionNs = rf.dispersionNs;
        ref.demoted      = rf.demoted;
        return ref;
    }

    void now(struct timespec &ts) const override
    {
        /* The anchor, not CLOCK_REALTIME. The OS clock is slewed towards the
         * same anchor so the log and the HTTP Date header agree with it, but a
         * correction in flight there must never reach a reply. */
        if (!GpsTime::instance().now(ts)) {
            clock_gettime(CLOCK_REALTIME, &ts);
        }
    }

    std::string describe() const override { return GpsTime::instance().describe(); }
};

}  // namespace

std::unique_ptr<TimeRef> makeGpsTimeRef()
{
    return std::unique_ptr<TimeRef>(new GpsTimeRef());
}
