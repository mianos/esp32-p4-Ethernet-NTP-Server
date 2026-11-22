/* Compiles this file's ESP_LOGD lines in; they stay hidden until raised with
 * the console's `loglevel` command. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

/* One handler per UBX message, called once a frame has passed its checksum
 * and length checks. */
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::onCfgTmode(const ubx::Frame &f)
{
    /* Only arrives as a poll reply, so it is the receiver's actual state. */
    const uint8_t mode = f.u8(0);
    int32_t       mean[3];
    for (int i = 0; i < 3; ++i) {
        mean[i] = f.i4(4 + 4 * i);
    }
    const uint32_t acc = f.u4(16);

    m_tmode = mode > 2 ? 0 : mode;
    if (mode == 2 && (mean[0] != 0 || mean[1] != 0 || mean[2] != 0)) {
        for (int i = 0; i < 3; ++i) {
            m_fixedMean[i] = mean[i];
        }
        m_fixedAcc   = acc;
        m_fixedValid = true;
        /* A fixed position set by anyone is kept. A "moving" reply does not
         * clear it, since that is what a reset looks like; only a stop does. */
        m_holdWanted = true;
    }
}

/* Messages in ubx::kDefs arrive length-checked; the handlers for the others
 * check their own length. */

void GpsTime::onDrop(ubx::Drop why, const ubx::Frame &frame)
{
    /* The decoder counts these; log the first few of each. */
    if (why == ubx::Drop::Checksum && m_badChecksumLogged < 5) {
        ++m_badChecksumLogged;
        ESP_LOGW(TAG, "frame %02x/%02x len %u: bad checksum", frame.cls, frame.id,
                 static_cast<unsigned>(frame.len));
    } else if (why == ubx::Drop::Length && m_badLengthLogged < 5) {
        ++m_badLengthLogged;
        ESP_LOGW(TAG, "frame %02x/%02x len %u: outside its definition", frame.cls, frame.id,
                 static_cast<unsigned>(frame.len));
    }
}

void GpsTime::onFrame(const ubx::Frame &frame)
{
    const uint8_t cls = frame.cls;
    const uint8_t id  = frame.id;
    m_lastFrameUs     = esp_timer_get_time();
    ++m_secFrames;
    m_recentFrames[m_recentAt++ % 16] = static_cast<uint16_t>((cls << 8) | id);
    if (!m_frameSeen) {
        m_frameSeen    = true;
        m_scanSteps    = 0;
        m_silentPasses = 0;
        ESP_LOGI(TAG, "UBX frames arriving at %u baud", static_cast<unsigned>(m_baud));
        /* Remember the working rate for the next boot, writing NVS only on a
         * change. */
        if (Settings::instance().gpsBaud() != std::to_string(m_baud)) {
            Settings::instance().setGpsBaud(std::to_string(m_baud));
            Settings::instance().save();
            ESP_LOGI(TAG, "keeping %u baud as the boot rate", static_cast<unsigned>(m_baud));
        }
    }

    /* The first dozen frames, to see what the receiver actually sends. */
    if (m_framesLogged < 12) {
        ++m_framesLogged;
        ESP_LOGD(TAG, "frame %02x/%02x len %u", cls, id, static_cast<unsigned>(frame.len));
    }

    if (cls == 0x05 && (id == 0x00 || id == 0x01)) { /* UBX-ACK-ACK / UBX-ACK-NAK */
        /* Only for the outstanding message; anything else is a late reply. */
        if (frame.u8(0) == m_awaitCls && frame.u8(1) == m_awaitId && m_awaitResult == -1) {
            m_awaitResult = (id == 0x01) ? 1 : 0;
        }
        if (!m_ackLogged && id == 0x01) {
            m_ackLogged = true;
            ESP_LOGI(TAG, "module accepted the 1 Hz message set");
        }
    } else if (cls == 0x01 && id == 0x07) { /* UBX-NAV-PVT */
        onNavPvt(frame);
    } else if (cls == 0x01 && id == 0x26) { /* UBX-NAV-TIMELS */
        onNavTimels(frame);
    } else if (cls == 0x06 && id == 0x31) { /* UBX-CFG-TP5, polled back */
        onCfgTp5(frame);
    } else if (cls == 0x0D && id == 0x12) { /* UBX-TIM-TOS */
        onTimTos(frame);
    } else if ((cls == 0x0D && id == 0x01) || (cls == 0x01 && id == 0x27)) {
        /* TIM-TP and its NAV-class spelling share a payload; counted apart to
         * show which one the module answers. */
        if (cls == 0x0D) {
            ++m_timTpTimN;
        } else {
            ++m_timTpNavN;
        }
        onNavTimTp(frame);
    } else if (cls == 0x06 && id == 0x3D) { /* UBX-CFG-TMODE2 */
        onCfgTmode(frame);
    } else if (cls == 0x0A && id == 0x04) { /* UBX-MON-VER */
        onMonVer(frame);
    } else if (cls == 0x0A && id == 0x09) { /* UBX-MON-HW */
        onMonHw(frame);
    } else if (cls == 0x01 && id == 0x35) { /* UBX-NAV-SAT */
        onNavSat(frame);
    } else if (cls == 0x0D && id == 0x04) { /* UBX-TIM-SVIN */
        onNavSvin(frame);
    } else if (cls == 0x01 && id == 0x04) { /* UBX-NAV-DOP */
        onNavDop(frame);
    } else if (cls == 0x01 && id == 0x22) { /* UBX-NAV-CLOCK */
        onNavClock(frame);
    } else if (cls == 0x01 && id == 0x03) { /* UBX-NAV-STATUS */
        onNavStatus(frame);
    } else if (cls == 0x0A && id == 0x38) { /* UBX-MON-RF */
        onMonRf(frame);
    } else if (cls == 0x0A && id == 0x02) { /* UBX-MON-IO */
        onMonIo(frame);
    } else if (cls == 0x0A && id == 0x07) { /* UBX-MON-RXBUF */
        onMonRxbuf(frame);
    } else if (cls == 0x0A && id == 0x08) { /* UBX-MON-TXBUF */
        onMonTxbuf(frame);
    } else if (cls == 0x0A && id == 0x06) { /* UBX-MON-MSGPP */
        onMonMsgpp(frame);
    }
}

void GpsTime::hexPreview(const ubx::Frame &f, text::Buf<241> &out)
{
    static constexpr char kDigits[] = "0123456789abcdef";
    const std::size_t     bytes     = f.len < 120 ? f.len : 120;
    for (std::size_t i = 0; i < bytes; ++i) {
        const uint8_t byte = f.u8(i);
        out[i * 2]         = kDigits[byte >> 4];
        out[i * 2 + 1]     = kDigits[byte & 0x0F];
    }
    out[bytes * 2] = '\0';
}

/* MON-IO: 20 bytes per port; UART1 is block 1. */
void GpsTime::onMonIo(const ubx::Frame &f)
{
    constexpr std::size_t kBlock = 20;
    constexpr std::size_t kPort  = 1;
    if (f.len < kBlock * (kPort + 1)) {
        return;
    }
    const std::size_t at  = kBlock * kPort;
    const int64_t     now = esp_timer_get_time();
    text::Buf<241>    hex = {};
    hexPreview(f, hex);
    portENTER_CRITICAL(&m_mux);
    std::memcpy(m_ioHex.data(), hex.data(), sizeof(m_ioHex));
    m_ioRxBytes = f.u4(at);
    m_ioTxBytes = f.u4(at + 4);
    m_ioParity  = f.u2(at + 8);
    m_ioFraming = f.u2(at + 10);
    m_ioOverrun = f.u2(at + 12);
    m_ioSeen    = true;
    m_ioAtUs    = now;
    portEXIT_CRITICAL(&m_mux);
}

/* MON-RXBUF: pending u2[6] at 0, usage u1[6] at 12, peakUsage u1[6] at 18. */
void GpsTime::onMonRxbuf(const ubx::Frame &f)
{
    constexpr std::size_t kPort = 1;
    if (f.len < 12 + 2 * kPort + 2) {
        return;
    }
    portENTER_CRITICAL(&m_mux);
    m_rbufPending = f.u2(2 * kPort);
    m_rbufPeak    = f.u8(12 + kPort);
    portEXIT_CRITICAL(&m_mux);
}

/* MON-TXBUF: as MON-RXBUF, then tUsage, tPeakusage and errors (X1 at 26; bit 0
 * limit reached, bit 1 allocation failure). */
void GpsTime::onMonTxbuf(const ubx::Frame &f)
{
    constexpr std::size_t kPort = 1;
    if (f.len < 12 + 2 * kPort + 3) {
        return;
    }
    portENTER_CRITICAL(&m_mux);
    m_tbufPending = f.u2(2 * kPort);
    m_tbufPeak    = f.u8(12 + kPort);
    m_tbufErrors  = f.u8(26);
    portEXIT_CRITICAL(&m_mux);
}

/* MON-MSGPP: msg1..msg6, U2[8] per port indexed by protocol (0 is UBX), then
 * skipped U4[6]. msg2 is UART1. */
void GpsTime::onMonMsgpp(const ubx::Frame &f)
{
    if (f.len < 120) {
        return;
    }
    constexpr std::size_t kPort1 = 16; /* msg2 */
    text::Buf<241>        hex    = {};
    hexPreview(f, hex);
    portENTER_CRITICAL(&m_mux);
    m_msgUbx     = f.u2(kPort1);
    m_msgSkipped = f.u4(96 + 4 * 1);
    std::memcpy(m_msgppHex.data(), hex.data(), sizeof(m_msgppHex));
    portEXIT_CRITICAL(&m_mux);
}

/* NAV-DOP: iTOW, then seven u2 DOPs in hundredths; north/east are not kept. */
void GpsTime::onNavDop(const ubx::Frame &f)
{
    if (f.len < 18) {
        return;
    }
    /* 0xFFFF means unknown. */
    const auto dop = [&f](std::size_t at) {
        const uint16_t raw = f.u2(at);
        return raw == 0xFFFF ? 0.0F : static_cast<float>(raw) / 100.0F;
    };
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&m_mux);
    m_gDop    = dop(4);
    m_pDop    = dop(6);
    m_vDop    = dop(10);
    m_hDop    = dop(12);
    m_dopAtUs = nowUs;
    portEXIT_CRITICAL(&m_mux);
}

/* NAV-CLOCK: iTOW, clkBias (ns, i4), clkDrift (ns/s, i4), tAcc (ns, u4),
 * fAcc (ps/s, u4). */
void GpsTime::onNavClock(const ubx::Frame &f)
{
    if (f.len < 20) {
        return;
    }
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&m_mux);
    m_clkBiasNs      = static_cast<int32_t>(f.u4(4));
    m_clkDriftNsPerS = static_cast<int32_t>(f.u4(8));
    m_tAccNs         = f.u4(12);
    m_fAccPsPerS     = f.u4(16);
    m_clockAtUs      = nowUs;
    portEXIT_CRITICAL(&m_mux);
}

/* NAV-STATUS: iTOW, gpsFix, flags, fixStat, flags2, ttff (ms), msss. */
void GpsTime::onNavStatus(const ubx::Frame &f)
{
    if (f.len < 16) {
        return;
    }
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&m_mux);
    m_navStatusFlags  = f.u8(5);
    m_navStatusFlags2 = f.u8(7);
    m_ttffMs          = f.u4(8);
    m_navStatusAtUs   = nowUs;
    portEXIT_CRITICAL(&m_mux);
}

/* MON-RF: version, nBlocks, 2 reserved, then 24 bytes per band; the worst
 * band is kept. */
void GpsTime::onMonRf(const ubx::Frame &f)
{
    const uint8_t declared = f.u8(1);
    const uint8_t fits     = static_cast<uint8_t>((f.len - 4) / 24);
    const uint8_t walk     = declared < fits ? declared : fits;

    uint8_t  worstJam = 0;
    uint8_t  worstAnt = 0;
    uint16_t minAgc   = 0xFFFF;
    for (uint8_t i = 0; i < walk; ++i) {
        const std::size_t base = 4 + 24 * static_cast<std::size_t>(i);
        const uint8_t     jam  = f.u8(base + 12);
        const uint8_t     ant  = f.u8(base + 2);
        const uint16_t    agc  = f.u2(base + 10);
        worstJam               = jam > worstJam ? jam : worstJam;
        worstAnt               = ant > worstAnt ? ant : worstAnt;
        minAgc                 = agc < minAgc ? agc : minAgc;
    }

    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&m_mux);
    m_rfBands    = walk;
    m_rfJamWorst = worstJam;
    m_rfAntWorst = worstAnt;
    m_rfAgcMin   = minAgc == 0xFFFF ? 0 : minAgc;
    m_rfSeen     = true;
    m_rfAtUs     = nowUs;
    portEXIT_CRITICAL(&m_mux);
}

void GpsTime::onMonVer(const ubx::Frame &f)
{
    if (m_verLogged) {
        return;
    }
    m_verLogged = true;

    /* NUL-separated strings (software, hardware, then extensions such as
     * PROTVER and MOD), kept with NULs as spaces. */
    std::string text;
    text.reserve(f.len);
    for (const uint8_t byte : f.payload) {
        const char c = static_cast<char>(byte);
        text += c == '\0' ? ' ' : (c >= 0x20 && c < 0x7F ? c : '.');
    }

    /* put() truncates to the fixed buffer. */
    portENTER_CRITICAL(&m_mux);
    text::put(m_moduleId, "{}", text);
    portEXIT_CRITICAL(&m_mux);

    ESP_LOGI(TAG, "module: %.48s", text.c_str());
}

void GpsTime::onMonHw(const ubx::Frame &f)
{
    /* UBX-MON-HW, 60 bytes:
     *   16 noisePerMS (u2)   18 agcCnt (u2)
     *   20 aStatus (u1: 0 init, 1 dontknow, 2 ok, 3 short, 4 open)
     *   21 aPower  (u1: 0 off, 1 on, 2 dontknow)
     *   22 flags   (X1, bits 2:3 jamming state 0 unknown / 1 ok / 2 warn / 3 crit)
     *   45 jamInd  (u1)
     * aStatus and aPower need the supervisor enabled by CFG-ANT. */
    const uint16_t noise  = f.u2(16);
    const uint16_t agc    = f.u2(18);
    const uint8_t  status = f.u8(20);
    const uint8_t  power  = f.u8(21);
    const uint8_t  jstate = (f.u8(22) >> 2) & 0x03;
    const uint8_t  jam    = f.u8(45);

    const uint32_t pinSel = f.u4(0);
    const uint32_t pinDir = f.u4(8);
    const uint32_t pinVal = f.u4(12);

    uint8_t before      = 0;
    uint8_t beforePower = 0;
    uint8_t beforeJam   = 0;

    portENTER_CRITICAL(&m_mux);
    before       = m_antStatus;
    beforePower  = m_antPower;
    beforeJam    = m_jamState;
    m_antStatus  = status;
    m_antPower   = power;
    m_noisePerMs = noise;
    m_agcCnt     = agc;
    m_jamInd     = jam;
    m_jamState   = jstate;
    m_monHwValid = true;
    m_monHwAtUs  = esp_timer_get_time();
    m_pinSel     = pinSel;
    m_pinDir     = pinDir;
    m_pinVal     = pinVal;
    portEXIT_CRITICAL(&m_mux);

    /* Logged when it changes; CFG-ANT names these pins. */
    if (pinSel != m_pinsLogged) {
        m_pinsLogged = pinSel;
        ESP_LOGI(TAG, "receiver pins: sel 0x%08lx dir 0x%08lx val 0x%08lx",
                 static_cast<unsigned long>(pinSel), static_cast<unsigned long>(pinDir),
                 static_cast<unsigned long>(pinVal));
    }

    /* Logged on change only. */
    if (status != before) {
        switch (status) {
        case 3:
            ESP_LOGW(TAG, "antenna SHORT: bias powered down, check the lead and the connector");
            break;
        case 4:
            ESP_LOGW(TAG, "antenna OPEN: nothing drawing bias, check the lead and the connector");
            break;
        case 2:  ESP_LOGI(TAG, "antenna status OK"); break;
        default: ESP_LOGI(TAG, "antenna status %u (no supervisor reading)", status); break;
        }
    }
    if (power != beforePower) {
        ESP_LOGI(TAG, "antenna bias %s", power == 1 ? "ON" : power == 0 ? "OFF" : "DONTKNOW");
    }
    if (jstate != beforeJam && jstate >= 2) {
        ESP_LOGW(TAG, "jamming state %s: check for overload from a preamp or a nearby transmitter",
                 jstate == 3 ? "CRITICAL" : "warning");
    }
}

void GpsTime::onNavSat(const ubx::Frame &f)
{
    /* UBX-NAV-SAT: 8-byte header (iTOW, version, numSvs, reserved), then 12
     * bytes per satellite. Every row is kept for the sky view; C/N0 mean and
     * max count only used satellites. The walk stops at the payload's end if
     * numSvs claims more. */
    const uint8_t num  = f.u8(5);
    const uint8_t fits = static_cast<uint8_t>((f.len - 8) / 12);
    const uint8_t walk = num < fits ? num : fits;

    uint16_t sum  = 0;
    uint8_t  max  = 0;
    uint8_t  used = 0;
    uint8_t  kept = 0;

    /* One short critical section per row: a critical section masks interrupts,
     * and holding one across the whole walk delayed the PPS edge. */
    m_satsSeq.fetch_add(1, std::memory_order_release); /* odd: rows in flux */
    for (uint8_t i = 0; i < walk; ++i) {
        const std::size_t sv  = 8 + 12 * static_cast<std::size_t>(i);
        const uint8_t     cno = f.u8(sv + 2);
        const uint32_t    fl  = f.u4(sv + 8);

        if (kept < kMaxSats) {
            portENTER_CRITICAL(&m_mux);
            Sat &s  = m_sats[kept++];
            s.gnss  = f.u8(sv + 0);
            s.sv    = f.u8(sv + 1);
            s.cno   = cno;
            s.elev  = static_cast<int8_t>(f.u8(sv + 3));
            s.azim  = static_cast<int16_t>(f.u2(sv + 4));
            s.flags = fl;
            /* prRes, signed; 0x8000 means not available. */
            const int16_t pr = static_cast<int16_t>(f.u2(sv + 6));
            s.prRes          = (pr == INT16_MIN) ? 0 : pr;
            s.prResValid     = pr != INT16_MIN;
            portEXIT_CRITICAL(&m_mux);
        }
        if ((fl & (1u << 3)) == 0 || cno == 0) {
            continue;
        }
        sum = static_cast<uint16_t>(sum + cno);
        max = cno > max ? cno : max;
        ++used;
    }

    portENTER_CRITICAL(&m_mux);
    m_cn0Max     = max;
    m_cn0Mean    = used != 0 ? static_cast<uint8_t>(sum / used) : 0;
    m_cn0Used    = used;
    m_satCount   = kept;
    m_navSatAtUs = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);
    m_satsSeq.fetch_add(1, std::memory_order_release);
}

void GpsTime::onNavSvin(const ubx::Frame &f)
{
    /* UBX-TIM-SVIN, 28 bytes: dur U4, meanX/Y/Z I4 (cm), meanV U4 (variance,
     * mm^2), obs U4, valid, active, reserved U2. The mean is kept after the
     * survey ends so it can be held. */
    const uint32_t dur = f.u4(0);
    const uint32_t var = f.u4(16);
    const uint32_t obs = f.u4(20);
    int32_t        mean[3];
    for (int i = 0; i < 3; ++i) {
        mean[i] = f.i4(4 + 4 * i);
    }

    portENTER_CRITICAL(&m_mux);
    for (int i = 0; i < 3; ++i) {
        m_svinMean[i] = mean[i];
    }
    m_svinDur    = dur;
    m_svinAcc    = var;
    m_svinObs    = obs;
    m_svinValid  = f.u8(24) != 0;
    m_svinActive = f.u8(25) != 0;
    m_svinAtUs   = esp_timer_get_time();
    if (m_svinActive) {
        /* Also catches a survey started from u-center. */
        m_tmode = 1;
    }
    portEXIT_CRITICAL(&m_mux);

    /* Announce the end; holding the position is left to the user. */
    static bool wasActive = false;
    if (wasActive && !m_svinActive && m_tmode == 1) {
        ESP_LOGW(TAG,
                 "survey-in finished after %u s at %u mm: "
                 "POST /gps/survey {\"action\":\"fix\"} to lock it",
                 static_cast<unsigned>(dur), static_cast<unsigned>(isqrt(var)));
    }
    wasActive = m_svinActive;
}

void GpsTime::onNavPvt(const ubx::Frame &f)
{
    m_statusDue = true; /* this second's facts are in: publish after the read */
    const uint8_t valid = f.u8(11);
    time_t        stamp = 0;
    /* validDate | validTime; before both are set the date is a default. */
    const bool haveTime = (valid & 0x03) == 0x03 && civilToEpoch(f.u2(4), f.u8(6), f.u8(7), f.u8(8),
                                                                 f.u8(9), f.u8(10), stamp);
    const int32_t nano  = f.i4(16); /* offset into the second, ~0 at 1 Hz */
    const uint8_t fix   = f.u8(20);
    const uint8_t sats  = f.u8(23);

    portENTER_CRITICAL(&m_mux);
    m_timeValid  = haveTime;
    m_fixType    = fix;
    m_satellites = sats;
    m_pvtNanoNs  = nano;
    m_pvtItowMs  = f.u4(0);
    portEXIT_CRITICAL(&m_mux);

    if (!haveTime) {
        return;
    }

    if (!m_fixLogged) {
        /* nano should be tens of ns if the epoch is the PPS second. */
        m_fixLogged = true;
        ESP_LOGI(TAG, "%s fix, %u satellites, UTC %s (nano %ld)", fixText(fix), sats,
                 isoTimestamp(stamp).c_str(), static_cast<long>(nano));
    }

    anchor(stamp);
}

void GpsTime::onCfgTp5(const ubx::Frame &f)
{
    /* Poll reply, 32 bytes: antCableDelay i2 at 4, userConfigDelay i4 at 24,
     * flags at 28. */
    if (f.len < 32) {
        return;
    }
    const int16_t  cable = static_cast<int16_t>(f.u2(4));
    const int32_t  user  = f.i4(24);
    const uint32_t flags = f.u4(28);
    if (cable == m_tp5CableNs && user == m_tp5UserNs && flags == m_tp5Flags) {
        return;
    }
    m_tp5CableNs = cable;
    m_tp5UserNs  = user;
    m_tp5Flags   = flags;
    m_tp5Period  = f.u4(8);
    m_tp5Ratio   = f.u4(16);
    ESP_LOGW(TAG, "CFG-TP5 reads back: cable %d ns, user delay %ld ns, flags 0x%08lx",
             static_cast<int>(cable), static_cast<long>(user), static_cast<unsigned long>(flags));
}

void GpsTime::onTimTos(const ubx::Frame &f)
{
    /* 56 bytes: flags X4 at 4, utcOffset i4 (ns) at 16, utcUncertainty u4 (ns)
     * at 20. cohPulse and lockedPulse are taken as bits 12 and 13. */
    const uint32_t flags  = f.u4(4);
    const int32_t  utcOff = f.i4(16);
    const uint32_t utcUnc = f.u4(20);

    portENTER_CRITICAL(&m_mux);
    m_tosFlags     = flags;
    m_tosUtcOffset = utcOff;
    m_tosUtcUnc    = utcUnc;
    portEXIT_CRITICAL(&m_mux);

    if (!m_tosLogged) {
        m_tosLogged = true;
        ESP_LOGW(TAG,
                 "TIM-TOS flowing: pulse %s, generation %s, %.3f us off the UTC second "
                 "(+/- %.3f us)",
                 (flags & (1u << 13)) != 0 ? "locked" : "NOT locked",
                 (flags & (1u << 12)) != 0 ? "coherent" : "not coherent",
                 static_cast<double>(utcOff) / 1e3, static_cast<double>(utcUnc) / 1e3);
    }
}

void GpsTime::onNavTimels(const ubx::Frame &f)
{
    /* 24 bytes, version 0:
     *   8  srcOfCurrLs (u1)   9  currLs (i1)
     *   10 srcOfLsChange (u1) 11 lsChange (i1)
     *   12 timeToLsEvent (i4) 16 dateOfLsGpsWn (u2) 18 dateOfLsGpsDn (u2)
     *   23 valid (u1): bit 0 says currLs is known, bit 1 says the event time is */
    const int8_t  current     = static_cast<int8_t>(f.u8(9));
    const int8_t  change      = static_cast<int8_t>(f.u8(11));
    const int32_t inS         = f.i4(12);
    const uint8_t valid       = f.u8(23);
    const bool    haveOffset  = (valid & 0x01) != 0;
    const bool    haveEventAt = (valid & 0x02) != 0;

    portENTER_CRITICAL(&m_mux);
    if (haveOffset) {
        m_leapSeconds = current;
        m_leapValid   = true;
    }
    /* No announcement means nothing pending. */
    m_leapChange = haveEventAt ? change : 0;
    m_leapInS    = haveEventAt ? inS : 0;
    portEXIT_CRITICAL(&m_mux);

    /* Logged on change, not on every countdown tick. */
    if (m_leapLoggedSeconds != m_leapSeconds || m_leapLoggedChange != m_leapChange) {
        m_leapLoggedSeconds = m_leapSeconds;
        m_leapLoggedChange  = m_leapChange;
        if (m_leapChange == 0) {
            ESP_LOGI(TAG, "leap: GPS-UTC offset %+d s, nothing announced", m_leapSeconds);
        } else {
            ESP_LOGW(
                TAG,
                "leap: GPS-UTC offset %+d s, and %+d s in %ld s (leap indicator 1 for that minute)",
                m_leapSeconds, m_leapChange, static_cast<long>(m_leapInS));
        }
    }
}

void GpsTime::onNavTimTp(const ubx::Frame &f)
{
    /* TIM-TP, 16 bytes:
     *    0 towMS (u4, ms of week)   4 towSubMS (u4, 2^-32 ms)
     *    8 qErr (i4, ps)
     *   12 week (u2)   14 flags (X1)   15 refInfo (u1)
     * flags: bit 0 time base (0 GNSS, 1 UTC), bit 1 UTC available, bits 2-3
     * RAIM, bit 4 qErrInvalid. qErr is applied to the anchor in anchor() and
     * half of it is counted in dispersion. */
    const uint32_t towSubMs = f.u4(4);
    const int32_t  qErrPs   = f.i4(8);
    const uint8_t  flags    = f.u8(14);

    const bool gnssBase    = (flags & 0x01) == 0;
    const bool qErrInvalid = (flags & 0x10) != 0;

    /* The pulse's offset into its second, in ns; 999 ms reads as just before
     * the second. */
    int32_t msIntoSecond = f.i4(0) % 1000;
    if (msIntoSecond < 0) {
        msIntoSecond += 1000;
    }
    int64_t offsetNs = static_cast<int64_t>(msIntoSecond) * 1000000 +
                       (static_cast<int64_t>(towSubMs) * 1000000 >> 32);
    if (msIntoSecond == 999) {
        offsetNs -= 1000000000;
    }
    /* Position only: GNSS and UTC bases differ by whole seconds, which pairing
     * already handles. */
    const bool onSecond = offsetNs > -2000000 && offsetNs < 2000000;

    /* Signed, for anchor()'s edge -= qErr. */
    const int32_t qErrNs =
        static_cast<int32_t>(std::clamp<int64_t>(qErrPs / 1000, -kMaxQErrNs, kMaxQErrNs));

    portENTER_CRITICAL(&m_mux);
    ++m_timTpCount;
    m_qErrNs        = qErrNs;
    m_qErrValid     = !qErrInvalid;
    m_pulseOnSecond = onSecond;
    m_pulseOffsetNs = offsetNs;
    m_timTpFlags    = flags;
    m_qErrAtUs      = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);

    /* Logged once. */
    if (!m_timTpLogged) {
        m_timTpLogged = true;
        if (qErrInvalid) {
            ESP_LOGW(TAG, "TIM-TP flowing but qErr is flagged invalid: not applied");
        } else {
            ESP_LOGI(TAG, "TIM-TP flowing: the receiver states %.3f us of pulse quantisation",
                     static_cast<double>(qErrNs) / 1000.0);
        }
    }
    if (!onSecond) {
        /* Logged on change. */
        if (m_pulseOnSecondSeen) {
            ESP_LOGW(TAG,
                     "timepulse is %.3f ms off the UTC second (%s base): every anchor is "
                     "shifted with it",
                     static_cast<double>(offsetNs) / 1e6, gnssBase ? "GNSS" : "UTC");
        }
    } else if (!m_pulseOnSecondSeen) {
        ESP_LOGI(TAG, "timepulse back on the UTC second");
    }
    m_pulseOnSecondSeen = onSecond;
}
