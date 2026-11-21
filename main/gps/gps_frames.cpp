/*
 * What arrives from the receiver: one handler per UBX message, called by the
 * decoder once a frame has passed its checksum and its length.
 */
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::onCfgTmode(const ubx::Frame &f)
{
    /* CFG-TMODE2 has no reason to arrive except in answer to a poll: the ACK
     * acknowledges it separately and carries only class and id. So anything
     * that gets here describes the receiver's actual state, which is the point
     * of asking -- the firmware cannot otherwise tell a fixed position from a
     * moving one across a reboot, and a reset would take one away silently. */
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
        /* Locked by anyone -- a request here, or somebody in u-center -- wants
         * holding. A "moving" reading does not clear the saved position: it is
         * what a reset looks like on the way past, and the position is still
         * worth putting back. Only an explicit stop says otherwise. */
        m_holdWanted = true;
    }
}

/* The listener half of the decoder: a frame that arrived whole and passed its
 * checksum and its length range. The length gate is in the decoder, so the
 * handlers below can read their fields without a guard apiece. */

void GpsTime::onDrop(ubx::Drop why, const ubx::Frame &frame)
{
    /* Both are counted by the decoder; this is where the first few of each are
     * said out loud, because a receiver that has started failing its checksums
     * looks exactly like one that has gone quiet and means something different. */
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
        /* The rate the receiver is actually on, so the next boot starts here
         * instead of scanning for it. Saved only on a change: a rate that keeps
         * working must not rewrite NVS every time it is re-found. */
        if (Settings::instance().gpsBaud() != std::to_string(m_baud)) {
            Settings::instance().setGpsBaud(std::to_string(m_baud));
            Settings::instance().save();
            ESP_LOGI(TAG, "keeping %u baud as the boot rate", static_cast<unsigned>(m_baud));
        }
    }

    /* The first dozen frames by class and id. Short lines, and only at
     * boot: when a message is enabled, acknowledged and then never arrives,
     * knowing what the receiver does send is the whole diagnosis. */
    if (m_framesLogged < 12) {
        ++m_framesLogged;
        ESP_LOGI(TAG, "frame %02x/%02x len %u", cls, id, static_cast<unsigned>(frame.len));
    }

    if (cls == 0x05 && (id == 0x00 || id == 0x01)) { /* UBX-ACK-ACK / UBX-ACK-NAK */
        /* Only the message actually outstanding is recorded. Anything else is a
         * late reply to something already timed out, and guessing which would
         * put the answer on the wrong message. */
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
        /* Counted apart: the two spellings are the same sixteen bytes, and
         * which one a module answers is the whole question on a part that has
         * gone quiet on both rates. */
        if (cls == 0x0D) {
            ++m_timTpTimN;
        } else {
            ++m_timTpNavN;
        }
        /* TIM-TP, and the NAV spelling of it, in case another module answers
         * only that one. Same payload and same handler either way. */
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

/* MON-IO: 20 bytes per port, in the receiver's fixed port order. UART1 is block
 * 1 -- the only port wired here -- and its counts are what say whether the line
 * is coping. Parity and framing errors are the wire's own verdict, separate
 * from anything the decoder later throws away. */
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

/* MON-RXBUF: pending[6] u2, usage[6], peakUsage[6]. Pending is bytes queued on
 * the module for that port: sustained non-zero means the decoder or the host
 * cannot keep up, and it climbs to a peak the receiver remembers. */
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

/* MON-TXBUF: the same shape plus a trailing usage and an error bitfield
 * (bit 0 limit reached, bit 1 allocation failure) that applies to all ports. */
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

/* MON-MSGPP: 120 bytes of U2[8] per port, then skipped[6] U4.
 *
 * Two things here are easy to get wrong, and were. The eight entries inside a
 * port's array are PROTOCOLS, not message classes -- index 0 is UBX, which is
 * all this firmware sends -- and the surrounding array is indexed by PORT, with
 * msg2 being port 1, UART1. Reading it as class-indexed finds the CFG traffic
 * at the wrong offset and reports a flat zero.
 *
 * What is kept is the UBX count (the traffic we generate) and the bytes UART1
 * threw away as unframed, which is the receiver's own view of the line rather
 * than ours.
 */
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

/* NAV-DOP: 18 bytes, iTOW then seven u2 DOPs in hundredths. Only the four that
 * describe the geometry are kept; the northing/easting split is a rotated view
 * of the same thing. */
void GpsTime::onNavDop(const ubx::Frame &f)
{
    if (f.len < 18) {
        return;
    }
    /* 0xFFFF is the message's "not known" and would read as 655.35. */
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
 * fAcc (ps/s, u4). tAcc is the receiver's own accuracy estimate and the reason
 * this message is here: the pulse's placeable precision, stated by the thing
 * that places it. */
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

/* NAV-STATUS: iTOW, gpsFix, flags, fixStat, flags2, ttff (ms), msss. The fix
 * type duplicates NAV-PVT; what is new here is ttff -- how long from reset to
 * first fix -- which is how a cold start is told from a loss of lock. */
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

/* MON-RF: version, nBlocks, 2 reserved, then 24 bytes per band. Only the worst
 * case over the bands is kept, because that is the one that describes the
 * receiver: a clean L1 next to a jammed L2 is still a jammed receiver. */
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

    /* The payload is NUL-terminated strings: software version, hardware
     * version, then extensions such as PROTVER=23.01 and MOD=ZED-M8. NULs
     * become spaces and the result is kept for /healthz, which is where it is
     * actually read -- the serial console drops bytes in exactly this part of
     * the boot, when the Ethernet task is also talking. */
    std::string text;
    text.reserve(f.len);
    for (const uint8_t byte : f.payload) {
        const char c = static_cast<char>(byte);
        text += c == '\0' ? ' ' : (c >= 0x20 && c < 0x7F ? c : '.');
    }

    /* The member is fixed storage because status() copies it under the mux, so
     * the truncation has to be explicit -- put() does it and terminates. */
    portENTER_CRITICAL(&m_mux);
    text::put(m_moduleId, "{}", text);
    portEXIT_CRITICAL(&m_mux);

    ESP_LOGI(TAG, "module: %.48s", text.c_str());
}

void GpsTime::onMonHw(const ubx::Frame &f)
{
    /* UBX-MON-HW is 60 bytes. The fields that matter for an antenna are:
     *   16 noisePerMS (u2, dBm)   18 agcCnt (u2)
     *   20 aStatus (u1: 0 init, 1 dontknow, 2 ok, 3 short, 4 open)
     *   21 aPower  (u1: 0 off, 1 on, 2 dontknow)
     *   22 flags   (X1, bits 2:3 jamming state 0 unknown / 1 ok / 2 warn / 3 crit)
     *   45 jamInd  (u1)
     *
     * aStatus and aPower only mean anything while the antenna supervisor is
     * enabled, which is what CFG-ANT does; without it they stay DONTKNOW and
     * that is reported rather than smoothed over. noisePerMS and agcCnt are
     * always real, and they are the pair that shows a front end being
     * overloaded: a preamp on a high-gain antenna drives agcCnt down and the
     * jamming state up, which looks like a good signal right up to the point
     * where it is a worse one. */
    const uint16_t noise  = f.u2(16);
    const uint16_t agc    = f.u2(18);
    const uint8_t  status = f.u8(20);
    const uint8_t  power  = f.u8(21);
    const uint8_t  jstate = (f.u8(22) >> 2) & 0x03;
    const uint8_t  jam    = f.u8(45);

    /* The pin map, which is the only place the supervisor's pin numbers appear. */
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

    /* The pin map once, and again if it moves: these are the numbers CFG-ANT has
     * to name, so they belong in the log where a bench session looks. */
    if (pinSel != m_pinsLogged) {
        m_pinsLogged = pinSel;
        ESP_LOGW(TAG, "receiver pins: sel 0x%08lx dir 0x%08lx val 0x%08lx",
                 static_cast<unsigned long>(pinSel), static_cast<unsigned long>(pinDir),
                 static_cast<unsigned long>(pinVal));
    }

    /* Only on a change: MON-HW is polled every two seconds and a line each time
     * would bury everything else. */
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
    /* UBX-NAV-SAT: 8 bytes of header (iTOW, version, numSvs, reserved) then 12
     * bytes per satellite. The groups are walked rather than the message being
     * kept, but every group is now retained for the sky view, not just the two
     * fields the means need.
     *
     * C/N0 is the number to compare antennas with. Satellite count is not: a
     * better antenna adds the marginal ones last, and a worse one can hold the
     * count up on the strongest handful while everything near the horizon goes.
     * Mean and max over *used* satellites therefore, not tracked ones. The
     * declared count can exceed what the payload holds -- the decoder gates on
     * 8 + 12n only up to its buffer -- so the walk stops at whichever is
     * shorter.
     *
     * Every walked group is stored, used or not, because the point of the plot
     * is to show what was rejected as well as what was kept; a satellite that
     * is loud but unhealthy is exactly the multipath signature worth seeing.
     * The means still count only used ones. */
    const uint8_t num  = f.u8(5);
    const uint8_t fits = static_cast<uint8_t>((f.len - 8) / 12);
    const uint8_t walk = num < fits ? num : fits;

    uint16_t sum  = 0;
    uint8_t  max  = 0;
    uint8_t  used = 0;
    uint8_t  kept = 0;

    /* One short critical section per row, rather than one around the walk.
     *
     * The whole walk held with interrupts masked delays the PPS edge, and on
     * this port a critical section masks every priority -- so a long one costs
     * the edge stamp no matter what level the PPS interrupt is at. Thirty-two
     * rows is not a "few hundred nanoseconds" once the loop, the offset maths
     * and the sum accumulation are counted; each row now costs a handful of
     * instructions and the sums accumulate outside the lock entirely.
     *
     * The rows are still written straight into m_sats rather than staged on the
     * stack, because this runs on the GPS task and 32 entries is 256 bytes of
     * it. What changed is only how long the lock is held, not where the data
     * lives. */
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
            /* prRes, centimetres, 0x8000 for "not available". Kept signed: a
             * negative residual means the measurement sits short of the
             * solution, and folding that away would hide half of what a
             * reflector does. */
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

    /* The scalars again need the lock, and again only for the stores. */
    portENTER_CRITICAL(&m_mux);
    m_cn0Max     = max;
    m_cn0Mean    = used != 0 ? static_cast<uint8_t>(sum / used) : 0;
    m_cn0Used    = used;
    m_satCount   = kept;
    m_navSatAtUs = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);
}

void GpsTime::onNavSvin(const ubx::Frame &f)
{
    /* UBX-TIM-SVIN: dur U4, meanX/Y/Z I4, meanV U4, obs U4, valid, active,
     * reserved U2 -- 28 bytes, confirmed against live frames while a survey was
     * running (dur, obs and active all read correctly; nothing else in the
     * message moves that way in step with a survey).
     *
     * meanV is a **variance in mm squared**, not an accuracy in mm, so it gets
     * square-rooted for display and the limit is squared when it is set. Note
     * that it is millimetres even though the positions beside it are
     * centimetres: the receiver mixes units inside one message.
     *
     * The mean is kept after the survey ends, because locking the position
     * afterwards is the whole point of having run one and the receiver carries
     * it in this message and nowhere else this firmware can reach. */
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
        /* The receiver is the authority on whether this is still running; a
         * survey started through u-center shows up here too. */
        m_tmode = 1;
    }
    portEXIT_CRITICAL(&m_mux);

    /* Announces the end once, so nobody has to sit and watch it. Logged rather
     * than acted on: locking the position automatically would throw away the
     * chance to look at the accuracy first. */
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
    /* The first 24 bytes carry everything used here; the decoder has already
     * held the frame to its full 92. */
    const uint8_t valid = f.u8(11);
    time_t        stamp = 0;
    /* validDate | validTime. u-blox only sets both once the receiver has an
     * almanac and a position estimate, which is exactly the point at which its
     * UTC can be believed -- before that the fields hold a 1980 default. */
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
        /* nano is logged because it is the one number that says the pairing
         * assumption holds: the receiver's solution epoch is the second
         * boundary the PPS marks, so nano should read a few tens of ns, not
         * hundreds of ms. */
        m_fixLogged = true;
        ESP_LOGI(TAG, "%s fix, %u satellites, UTC %s (nano %ld)", fixText(fix), sats,
                 isoTimestamp(stamp).c_str(), static_cast<long>(nano));
    }

    anchor(stamp);
}

void GpsTime::onCfgTp5(const ubx::Frame &f)
{
    /* Reply to the poll above: 32 bytes, antCableDelay (i2) at 4,
     * userConfigDelay (i4) at 24, flags at 28. Published because a nonzero user
     * delay is a constant shift nobody can see any other way. */
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
    /* 56 bytes: flags (X4) at 4, utcOffset (i4, ns) at 16, utcUncertainty (u4,
     * ns) at 20. The flag order is the graphic's, so cohPulse and lockedPulse
     * are the last two: bits 12 and 13. */
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
    /* 24 bytes, version 0, from the u-blox 8 interface description:
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
    /* A module that has an offset but no announcement is the normal case: it
     * means nothing is pending, which is different from not knowing. */
    m_leapChange = haveEventAt ? change : 0;
    m_leapInS    = haveEventAt ? inS : 0;
    portEXIT_CRITICAL(&m_mux);

    /* Once when it changes, not every second: the countdown ticks, and a line
     * a second about a leap months away is noise in a log that is read on a
     * serial console. */
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
    /* NAV-TIM-TP is 16 bytes, from the u-blox 8 interface description:
     *    0 towMS (u4, ms of week)   4 towSubMS (u4, 2^-32 of a ms)
     *    8 qErr (i4, picoseconds)
     *   12 week (u2)   14 flags (X1)   15 refInfo (u1)
     *
     * The flags byte, bit 0: 0 = GNSS time base, 1 = UTC -- that way round per
     * the u-blox 8/M8 receiver description, and this read it inverted until a
     * timing question forced the manual open.
     *
     * qErrInvalid is bit 4, not bit 3: raim sits between them and is two bits
     * wide (0 unavailable, 1 not active, 2 active). Reading bit 3 therefore
     * tested raim's high bit, which is 1 whenever raim is "not active" -- so
     * this reported the quantisation invalid on a module that was sending a
     * real one several times a second (9.7 ns, -3.8 ns, 4.0 ns ...) and the
     * correction never ran.
     *
     * This is the receiver narrating its own pulse, and it is the only source
     * for two things this reference used to assume: how well the receiver knows
     * where the edge is, and whether the pulse is on the UTC second at all. The
     * first goes into root delay, the second into a log line -- a pulse that has
     * been dragged somewhere in u-center otherwise looks exactly like a healthy
     * one from every other angle.
     *
     * Nothing here corrects the anchor. Subtract the mean of the interrupt
     * latency and the anchor moves by a constant that a later anchor absorbs
     * again (see kIsrLatencyNs); what this adds is the spread, which does not
     * cancel and is what a client needs in order to know how good the number it
     * is handed actually is. */
    const uint32_t towSubMs = f.u4(4);
    const int32_t  qErrPs   = f.i4(8);
    const uint8_t  flags    = f.u8(14);

    const bool gnssBase    = (flags & 0x01) == 0;
    const bool qErrInvalid = (flags & 0x10) != 0;

    /* Where in the second the pulse sits, in nanoseconds: a UTC-aligned 1 Hz
     * pulse is a hair either side of zero. A reported position just under a
     * second is the same place seen from the other side, so it is made
     * negative rather than being read as 999 ms late. */
    int32_t msIntoSecond = f.i4(0) % 1000;
    if (msIntoSecond < 0) {
        msIntoSecond += 1000;
    }
    int64_t offsetNs = static_cast<int64_t>(msIntoSecond) * 1000000 +
                       (static_cast<int64_t>(towSubMs) * 1000000 >> 32);
    if (msIntoSecond == 999) {
        offsetNs -= 1000000000;
    }
    /* About the position alone, not the base: a GNSS-base pulse sits at the
     * same instant as a UTC one -- the labels differ by whole leap seconds, and
     * whole seconds are already handled by pairing the frame's second with the
     * edge. Requiring UTC here marked a perfectly placed pulse as off. */
    const bool onSecond = offsetNs > -2000000 && offsetNs < 2000000;

    /* Signed. The correction in anchor() is "captured - qErr", and with the
     * magnitude stored here it moved every edge the same way: right for one
     * half of the sawtooth and doubling it for the other. */
    const int32_t qErrNs =
        static_cast<int32_t>(std::clamp<int64_t>(qErrPs / 1000, -kMaxQErrNs, kMaxQErrNs));

    /* The position itself, not just whether it is inside the window: this is
     * the receiver stating where its own pulse sits against the second it
     * labels, which is the only sub-millisecond statement of that available
     * (TIM-TP carries it; NAV-TIM-TP did not get as far as being answered). A
     * constant here shifts every anchor and nothing downstream can see it. */
    portENTER_CRITICAL(&m_mux);
    ++m_timTpCount;
    m_qErrNs        = qErrNs;
    m_qErrValid     = !qErrInvalid;
    m_pulseOnSecond = onSecond;
    m_pulseOffsetNs = offsetNs;
    m_timTpFlags    = flags;
    m_qErrAtUs      = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);

    /* Once, so the log says which regime this board is in: a measured
     * quantisation, or none. */
    if (!m_timTpLogged) {
        m_timTpLogged = true;
        if (qErrInvalid) {
            ESP_LOGW(TAG,
                     "NAV-TIM-TP flowing but qErr is flagged invalid: root delay stays "
                     "the named %lld us",
                     static_cast<long long>(kIsrLatencyNs / 1000));
        } else {
            ESP_LOGI(TAG, "NAV-TIM-TP flowing: the receiver states %.3f us of pulse quantisation",
                     static_cast<double>(qErrNs) / 1000.0);
        }
    }
    if (!onSecond) {
        /* On the change, not every second: a pulse off the UTC second shifts
         * every anchor with it, so it is worth saying, and worth saying again
         * only if it goes away. */
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
