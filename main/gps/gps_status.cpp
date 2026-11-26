/* The lock decision, the published status, reply facts and the TimeRef
 * adapter. */
#include "gps/gps_internal.h"

#include "gps/leap.h"
#include "gps/mono.h"

namespace {

/* The status ring holds fixed char arrays so a reader can take one with no
 * lock, and the task holds a text::Buf of the same size next to it. Pairing
 * them by hand had no check, so widening one side would have over-read the
 * other silently; this makes the pair part of the type. */
template <std::size_t N, std::size_t M> void copyBuf(char (&dst)[N], const text::Buf<M> &src)
{
    static_assert(N == M, "a status text field and its working copy differ in size");
    std::memcpy(dst, src.data(), N);
}

}  // namespace

using namespace gps_detail;

uint8_t GpsTime::copySats(Sat *out, uint8_t max) const
{
    /* Retry while m_satsSeq is odd or changes; no lock, so the PPS interrupt
     * is never held off. */
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
    /* Needs an anchor, a valid NAV-PVT date, fix type 2 or above (which
     * includes 5, time only) and an edge inside the lock window. */
    if (!(in.anchored && in.timeValid && in.fixType >= 2 && in.ppsAgeUs >= 0 &&
          in.ppsAgeUs < in.lockWindowUs)) {
        return false;
    }

    /* A bad vote is any of: the newest edge was refused; MON-HW reports
     * critical jamming (a jammed receiver can keep a regular pulse); or, while
     * the edge is fresh, the offset is outside kLockOffsetNs or, once a rate is
     * known, the fit's worst residual is above kLockTieMaxNs. */
    const bool fresh = in.ppsAgeUs <= kLockWindowUs;
    const bool bad   = !in.trusted || in.jamState == 3 ||
                     (fresh && (in.offsetNs > kLockOffsetNs || in.offsetNs < -kLockOffsetNs)) ||
                     (fresh && in.rateKnown && in.tieMaxNs > kLockTieMaxNs);

    /* One vote per edge, not per call, since replies and status reads both
     * call this; kGateStreak bad edges in a row demote. m_gate holds the last
     * voted edge count (low 24 bits) and the streak, so no lock is needed. */
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
    /* Retry while m_edgeSeq is odd or changes; no lock. */
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
    /* Fill the next ring slot, then name it newest. GPS task only; the edge
     * words and the suppress time are read lock-free below. */
    const uint32_t next = (m_statusHead.load(std::memory_order_relaxed) + 1) % kStatusSlots;
    StatusSlot    &slot = m_statusRing[next];
    const uint32_t seq  = slot.seq.load(std::memory_order_relaxed);
    slot.seq.store(seq + 1, std::memory_order_release); /* odd: being written */

    StatusData   &st = slot.data;
    StatusExtras &ex = slot.extra;
    st               = StatusData{};

    st.ticksPerUs = static_cast<uint32_t>(systimer_us_to_ticks(1));

    EdgeWords edge;
    readEdge(edge);
    st.ppsCount   = edge.count;
    st.ppsPtpNs   = edge.ptpNs;
    ex.ppsMonoUs  = edge.monoUs;
    ex.lastEdgeUs = edge.lastUs;
    /* 64-bit, written by another task: read until two reads agree. */
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
    st.phcSlewPpb        = m_phcSlewPpb;
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
    ex.anchorMonoNs   = m_anchorMonoNs;

    /* 1588 counter against esp_timer since the anchor; the counter is read
     * either side of the timer and kept only if the reads are close. */
    if (st.anchored && ex.anchorPhcNs != 0 && ptp::hardware()) {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const int64_t phcBefore = ptp::nowNs();
            /* On the counter's own nanoseconds: the point of this reading is
             * a residual well under the microsecond esp_timer reports. */
            const int64_t monoNs    = mono::nowNs();
            const int64_t phcAfter  = ptp::nowNs();
            if (phcAfter - phcBefore < 2000) {
                st.phcMonoDeltaNs = ((phcBefore + phcAfter) / 2 - ex.anchorPhcNs) -
                                    (monoNs - ex.anchorMonoNs);
                break;
            }
        }
    }

    for (int i = 0; i < 16; ++i) {
        ex.recentFrames[i] = m_recentFrames[i];
    }
    /* The bridge task writes two of these unlocked; at worst a copy is torn
     * text, still terminated. */
    copyBuf(ex.msgpp, m_msgppHex);
    copyBuf(ex.io, m_ioHex);
    copyBuf(ex.module, m_moduleId);
    copyBuf(ex.bridgeEnd, m_bridgeEnd);
    copyBuf(ex.bridgeLast, m_bridgeLast);

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

    /* Copy the newest slot without a lock; the check catches a reader stalled
     * long enough for the writer to come round again. */
    for (int attempt = 0; attempt < 4; ++attempt) {
        const StatusSlot &slot        = m_statusRing[m_statusHead.load(std::memory_order_acquire)];
        const uint32_t    before      = slot.seq.load(std::memory_order_acquire);
        static_cast<StatusData &>(st) = slot.data;
        ex                            = slot.extra;
        if ((before & 1u) == 0 && slot.seq.load(std::memory_order_acquire) == before) {
            break;
        }
    }

    /* The pair the anchor was taken from, so a reader can tell an anchor that
     * is merely old from one whose two readings disagree. */
    st.anchorMonoNs = ex.anchorMonoNs;
    st.anchorPhcNs  = ex.anchorPhcNs;

    /* The edge is read live; the snapshot can be a second old. */
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

    /* Formatted on the reader's side, not in the GPS task. */
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

    /* Holdover window once a rate is known, otherwise the short one. */
    st.lockWindowUs =
        (rateKnown && st.timeValid && st.fixType >= 2) ? kHoldoverWindowUs : kLockWindowUs;

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
        /* See dispersionNs() for the terms. */
        st.rootDelayNs = rootDelayNs();

        const int64_t lastTpUs = ex.qErrAtUs;
        /* Also published on its own; see unobservedNs(). */
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

    /* The stored change is 0 when the receiver announced nothing, so it doubles
     * as "an event is known". The window and the sign become the indicator in
     * gps/leap.h, where they are tested. */
    leap::Announcement announced;
    announced.offsetKnown = valid;
    announced.eventKnown  = change != 0;
    announced.changeS     = change;
    announced.inS         = inS;
    return leap::indicator(announced);
}

std::string GpsTime::describe() const
{
    const Status st = status();

    if (st.locked) {
        if (st.anchorAgeUs > kLockWindowUs) {
            return std::format("GPS/PPS, {} fix, {} satellites, holding over {:.1f} s",
                               fixText(st.fixType), st.satellites,
                               static_cast<double>(st.anchorAgeUs) / 1e6);
        }
        return std::format("GPS/PPS, {} fix, {} satellites", fixText(st.fixType), st.satellites);
    }
    if (st.fixType >= 2 && st.timeValid) {
        /* A fix but no usable edge; refused edges and missing edges point at
         * different faults. */
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

    /* ClockSource is told whether the pulse is good, not whether we are locked:
     * while the upstream holds the clock the offset is outside the lock band,
     * and it cannot return until this servo steers again. */
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
            ESP_LOGW(TAG,
                     "not locked: %u PPS edges in a row refused (wrong spacing, or the "
                     "module's second did not advance with them)",
                     st.edgeRejects);
        } else {
            ESP_LOGW(TAG, "not locked (fix %s, last edge %lld ms ago): stratum 16 until it returns",
                     fixText(st.fixType), static_cast<long long>(st.ppsAgeUs / 1000));
        }
    }

    /* Holdover keeps the lock, so log its start and end separately. */
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

    /* TIM-TP that was arriving and has stopped. */
    const int64_t nowUs = esp_timer_get_time();
    if (st.timTpCount != m_timTpLastCount) {
        m_timTpLastCount  = st.timTpCount;
        m_timTpLastUs     = nowUs;
        m_timTpGoneLogged = false;
    } else if (st.timTpCount != 0 && !m_timTpGoneLogged && nowUs - m_timTpLastUs > 10 * 1000000) {
        m_timTpGoneLogged = true;
        /* Stop using a stale qErr; dispersion keeps kIsrLatencyNs and the
         * fit jitter. */
        portENTER_CRITICAL(&m_mux);
        m_qErrValid = false;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG, "TIM-TP stopped 10 s ago: qErr no longer applied");
    }
}

/* The most dispersion served as stratum 1; beyond it replies are stratum 16,
 * refid INIT. A healthy box states about 40 us. */
constexpr int64_t kServeHonestNs = 400000;

/* The client-measured ServePath term alone must not be able to demote. */
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
    /* Age of the anchor, not of the last edge: edges keep arriving when frames
     * stop. */
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
    rf.refId = rf.locked ? kRefIdGps : kRefIdInit;
    rf.leap = leapIndicator();
    if (rf.locked) {
        const int64_t budget = dispersionNs(in.ppsAgeUs, unobservedNs());
        if (budget > 0) {
            rf.rootDelayNs  = static_cast<uint32_t>(rootDelayNs());
            rf.dispersionNs = static_cast<uint32_t>(budget);
        }

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

/* Stateless adapter over the GpsTime singleton. */
class GpsTimeRef : public TimeRef {
public:
    /* replyFacts(), which avoids status()'s string building. */
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
        /* The anchor, not CLOCK_REALTIME, which may be mid-slew. */
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
