/*
 * Talking to the receiver: the configuration burst, the baud scan, the time
 * mode and position hold, and the send-and-wait-for-ACK primitive under them.
 */
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::applyBias(bool on)
{
    m_antBiasOn = on;
    gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
}

/* NMEA PUBX,41 -- the port configuration sentence.
 *
 * A second way to say what CFG-PRT says, and the only one that still gets in
 * when the receiver's UBX *input* mask has been zeroed: NMEA parsing does not
 * depend on it. That is the failure this exists for -- a receiver pulsing PPS
 * and answering nothing, deaf to every UBX message sent at every rate, because
 * its UART1 input mask went to zero and the config was saved that way. A rail
 * power cycle reloads the same broken config, so the repair has to arrive over
 * the wire or not at all.
 *
 * Sent three times at the rate the scan is on, because a receiver whose baud we
 * have guessed wrong reads the tail of one and the head of the next. Harmless
 * when the link is healthy: it restates what is already true.
 *
 *        $PUBX,41,1,0003,0001,<baud>,0*<cs>
 *               |  |    |    |      + autobauding off
 *               |  |    |    + baud rate
 *               |  |    + out: UBX
 *               |  + in: UBX | NMEA
 *               + portID: UART1
 */
void GpsTime::sendPubxPortConfig(uint32_t baud)
{
    char      bodyText[40] = {};
    const int body         = std::snprintf(bodyText, sizeof(bodyText), "PUBX,41,1,0003,0001,%u,0",
                                           static_cast<unsigned>(baud));
    if (body <= 0 || body >= static_cast<int>(sizeof(bodyText))) {
        return;
    }
    uint8_t cs = 0;
    for (int i = 0; i < body; ++i) {
        cs ^= static_cast<uint8_t>(bodyText[i]);
    }
    /* Two buffers: snprintf cannot take its own destination as an argument. */
    char      sentence[48] = {};
    const int total = std::snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", bodyText, cs);
    if (total <= 0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        uart_write_bytes(kUart, sentence, static_cast<size_t>(total));
    }
}

/* CFG-PRT for UART1, before the burst. Sent at whatever rate is currently working, so the module
 * receives it; the caller switches the host afterwards. 8N1, UBX in and out,
 * NMEA off -- the same shape the boot configuration leaves behind. RAM only, so
 * a module power cycle still returns to its own default and the scan finds it. */
/* CFG-CFG, saving only the IO section (bit 0 of the save mask).
 *
 * The port configuration has to outlive a power cut or a baud change cannot
 * survive one: CFG-PRT alone is RAM, and this module stops scheduling output
 * after it until it is power-cycled -- so the change needs a power cycle to
 * become usable, and a power cycle throws away anything not saved.
 *
 * Only IO is saved, not the whole configuration. Saving everything would pin
 * the message rates and the GNSS mask to whatever the firmware happened to send
 * that boot, and the firmware deliberately leaves the module's own flash alone
 * so a later u-center session is not fighting a stale snapshot. */
int8_t GpsTime::saveCfgIo()
{
    uint8_t cfg[12] = {};
    text::putLe(cfg + 0, 0u);          /* clearMask: nothing */
    text::putLe(cfg + 4, 0x00000001u); /* saveMask: IO */
    text::putLe(cfg + 8, 0u);          /* loadMask: nothing */
    return sendAndWait(0x06, 0x09, cfg, sizeof(cfg));
}

int8_t GpsTime::sendCfgPrt(uint32_t baud)
{
    uint8_t cfg[20] = {};
    cfg[0]          = 1;               /* portID: UART1 */
    text::putLe(cfg + 4, 0x000008D0u); /* 8 data bits, no parity, 1 stop */
    text::putLe(cfg + 8, baud);
    /* The masks are BITFIELDS: bit 0 UBX, bit 1 NMEA, bit 2 RTCM, bit 5 RTCM3.
     * Outgoing has to keep bit 0 or the receiver stops sending UBX altogether
     * and the only frames left are the ACKs to these very messages -- which read
     * as a live-but-idle link and strand the scan, because a trickle keeps it
     * from ever advancing. Incoming keeps NMEA too, harmlessly, because a
     * module someone has been at with u-center may still be emitting it. */
    text::putLe(cfg + 12, 0x0003u); /* in: UBX | NMEA */
    text::putLe(cfg + 14, 0x0001u); /* out: UBX */
    return sendAndWait(0x06, 0x00, cfg, sizeof(cfg));
}

void GpsTime::adoptTimeMode()
{
    /* Only the answer matters, and onCfgTmode stores it. */
    sendAndWait(0x06, 0x3D, nullptr, 0);
}

void GpsTime::saveTimeMode()
{
    /* Called on every transition. NVS writes are cheap at this rate and the
     * whole point is that the next boot knows what the last one wanted. */
    Settings &settings = Settings::instance();
    settings.setGpsTimeMode(m_tmode == 1 ? "survey" : m_tmode == 2 ? "fixed" : "moving");
    settings.save();
}

bool GpsTime::sendFixedPosition()
{
    uint8_t tm[28] = {};
    tm[0]          = 2; /* fixed */
    for (int i = 0; i < 3; ++i) {
        text::putLe(tm + 4 + 4 * i, m_fixedMean[i]);
    }
    text::putLe(tm + 16, m_fixedAcc);
    if (sendAndWait(0x06, 0x3D, tm, sizeof(tm)) != 1) {
        return false;
    }
    m_tmode      = 2;
    m_holdWanted = true;
    return true;
}

void GpsTime::restoreTimeMode()
{
    /* The module's RAM died with the supply, so what was wanted is read back
     * from NVS and re-sent. A survey resumes with its own parameters; a locked
     * position goes back as it was. Without this, a power cut (or street works)
     * costs a day's survey and leaves nothing to say it ever existed. */
    Settings          &settings = Settings::instance();
    const std::string &mode     = settings.gpsTimeMode();

    if (mode == "fixed") {
        long long     x   = 0;
        long long     y   = 0;
        long long     z   = 0;
        unsigned long acc = 0;
        unsigned long val = 0;
        /* The validated flag rides on the position as a fifth field: it is a
         * property of that position, and it used to live only in RAM, so every
         * reboot silently threw away a survey that had signed itself off. Four
         * fields is the older form and reads as unvalidated, which is what it
         * was. */
        /* A zero position parses and is not a position: it is the centre of the
         * Earth, and holding it costs microseconds of clock for every kilometre.
         * It is what a lock taken with no survey mean used to store. */
        const bool stored = (text::getList(settings.gpsFixedPos(), ',', x, y, z, acc, val) ||
                             text::getList(settings.gpsFixedPos(), ',', x, y, z, acc)) &&
                            !(x == 0 && y == 0 && z == 0);
        if (stored) {
            adoptFixedPosition(x, y, z, acc, val != 0);
            ESP_LOGW(TAG, "position hold restored from NVS%s",
                     m_holdValidated ? ", survey-validated" : "");
            /* Adopting is not asking. This block used to end here, which left
             * the position in RAM and the receiver in moving mode: the module
             * came up moving, was never told otherwise, and every boot spent the
             * survey for nothing while the box reported a hold it did not have.
             *
             * The frame is not sent this instant, though. Sending TMODE2
             * restarts the pulse generator, and at four seconds in the receiver
             * is still acquiring: done here it cost five seconds of edges, two
             * refusals and the upstream taking the clock. The periodic block
             * sends it once there is a fix and a live edge to send it over. */
            m_holdRetryUs    = kHoldRetryUs;
            m_holdAssertAtUs = esp_timer_get_time() + kHoldSettleUs;
            ESP_LOGW(TAG, "position hold will be asserted once the receiver has settled");
            return;
        }
        /* The mode says fixed but the position did not survive. Falling out of
         * this block used to leave the receiver holding nothing at all, because
         * the survey arm below was gated on the mode being "survey" -- so the
         * box came up moving, with no hold, and said nothing. The survey pair is
         * still in NVS from the run the position was locked from, which is
         * exactly what this case needs. */
        ESP_LOGW(TAG, "time mode is fixed but no position survived: surveying instead");
        if (!startSurveyFromSettings()) {
            ESP_LOGW(TAG, "  and no survey parameters survived either: moving position");
        }
        return;
    }
    if (mode == "survey" && !startSurveyFromSettings()) {
        ESP_LOGW(TAG, "time mode is survey but the parameters did not survive: moving position");
    }
}

bool GpsTime::startSurveyFromSettings()
{
    /* Shared by the boot restore and the reset re-assert, which want the same
     * thing for the same reason: an armed survey that outlives the RAM. The
     * mode decision stays with the callers -- a stale pair must not start a
     * survey on a box that was deliberately left moving. */
    unsigned long hours = 0;
    unsigned long accMm = 0;
    if (!text::getList(Settings::instance().gpsSurvey(), ',', hours, accMm) || hours < 2 ||
        hours > 48 || accMm < 50 || accMm > 60000) {
        return false;
    }
    m_svinDurS  = hours * 3600;
    m_svinAccMm = accMm;
    m_svinCmd   = 1;
    ESP_LOGW(TAG, "survey armed from NVS: %lu h, %lu mm", hours, accMm);
    return true;
}

void GpsTime::reassertFix()
{
    /* Runs after a reset or a clear-to-defaults, when the time mode has just
     * been taken away. The stored record is the authority: if a position
     * survived, it goes back. Only when none survived does a survey resume --
     * the same order, and for the same reason, as restoreTimeMode.
     *
     * m_fixedValid is deliberately not the test. After a reset the receiver
     * cannot yet say what the mode was, so it can be false while the record is
     * perfectly good -- which is how a power cycle used to restart a survey
     * over the top of a validated hold.
     *
     * The record is re-read rather than taken from RAM for the same reason: a
     * reset may be exactly when it was corrected by hand, and applying the
     * stale copy would quietly undo that. */
    long long          x   = 0;
    long long          y   = 0;
    long long          z   = 0;
    unsigned long      acc = 0;
    unsigned long      val = 0;
    const std::string &pos = Settings::instance().gpsFixedPos();
    const bool         stored =
        (text::getList(pos, ',', x, y, z, acc, val) || text::getList(pos, ',', x, y, z, acc)) &&
        !(x == 0 && y == 0 && z == 0);
    if (!stored) {
        /* No hold to put back: a survey may be the thing that was wanted.
         * Restarting it loses its elapsed time, which is still better than a
         * reset silently ending a day's run. */
        const std::string &mode = Settings::instance().gpsTimeMode();
        if ((mode == "survey" || mode == "fixed") && startSurveyFromSettings()) {
            ESP_LOGW(TAG, "survey restarted after the reset");
        }
        return;
    }

    adoptFixedPosition(x, y, z, acc, val != 0);

    if (sendFixedPosition()) {
        ESP_LOGW(TAG, "position hold put back after the reset%s",
                 m_holdValidated ? ", survey-validated" : "");
    } else {
        m_holdRetryUs    = kHoldRetryUs;
        m_holdAssertAtUs = esp_timer_get_time() + m_holdRetryUs;
        ESP_LOGW(TAG, "position hold could not be put back: retrying in %lld s",
                 static_cast<long long>(kHoldRetryUs / 1000000));
    }
}

void GpsTime::advanceBaud()
{
    constexpr int kCount = static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0]));

    /* A pass is the whole list and is counted, not inferred from the index.
     *
     * Wrapping the index back to the preferred rate instead looked equivalent
     * and was not: starting at the preferred rate and stepping forward never
     * visits the entries *before* it. With the stored rate at 9600 the scan went
     * 9600, 115200, 57600, 19200, 4800 and round again, so the receiver's
     * actual 38400 was never tried at all -- one commit after the one that
     * introduced it, and every boot after. */
    if (++m_scanSteps >= kCount) {
        m_scanSteps = 0;
        ++m_silentPasses;
        /* A whole pass with nothing at all. The module can be left in a state
         * where it says nothing on any rate -- a port configuration half
         * written, a burst of line noise taken as a frame -- and then no entry
         * in the list is the right one and the scan can only walk it forever.
         *
         * A reset frame is twelve bytes and is harmless at a wrong rate, so it
         * goes out at every rate in the list, once a minute. The module comes
         * back at its own defaults, NMEA at 9600, and the scan starts from
         * there; the message set is re-asserted when frames appear. */
        const int64_t nowUs = esp_timer_get_time();
        /* Not on the first pass. That one ends before the rate the scan left
         * has been listened to a second time, so a walk-off from a rate that
         * was right all along reset a healthy receiver on the way back to it. */
        if (m_silentPasses >= 2 && nowUs - m_lastSweepUs > 60000000LL) {
            m_lastSweepUs            = nowUs;
            constexpr uint8_t rst[4] = {0x00, 0x00, 0x00, 0x00}; /* hot, hardware */
            for (uint32_t baud : kBauds) {
                (void)uart_set_baudrate(kUart, baud);
                sendUbx(0x06, 0x04, rst, sizeof(rst));
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            ESP_LOGW(TAG, "receiver silent at every rate: reset swept, back to 9600");
            for (int i = 0; i < kCount; ++i) {
                if (kBauds[i] == 9600) {
                    m_baudIndex = i;
                }
            }
        } else {
            m_baudIndex = m_preferredIndex;
        }
        if (!m_noDataLogged) {
            ESP_LOGW(TAG,
                     "no UBX frames at any baud rate: check module TX -> GPIO%d "
                     "(esp RX) and PPS -> GPIO%d",
                     kRxPin, static_cast<int>(kPpsPin));
            m_noDataLogged = true;
        }
    } else {
        m_baudIndex = (m_baudIndex + 1) % kCount;
        if (!m_noDataLogged) {
            ESP_LOGI(TAG, "no UBX frames yet, trying %u baud",
                     static_cast<unsigned>(kBauds[m_baudIndex]));
        }
    }

    m_baud              = kBauds[m_baudIndex];
    const esp_err_t err = uart_set_baudrate(kUart, m_baud);
    /* Half a frame read at the old rate must not be finished with bytes at the
     * new one. */
    m_decoder.reset();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_baudrate(%u): %s", static_cast<unsigned>(m_baud),
                 esp_err_to_name(err));
    }
    /* Before the UBX burst, and on the off chance the port config is what is
     * wrong: NMEA gets in where UBX no longer does. */
    if (!m_frameSeen) {
        sendPubxPortConfig(m_baud);
        esp_rom_delay_us(20000);
    }
    m_poorSecs  = 0;
    m_secFrames = 0;
    configureModule();
}

/* The pulse frame on its own, so the delay can be changed without re-running the
 * whole of configureModule() (which throws the acquisition away with a GNSS mask
 * reset). Split out for that reason only: at boot it is still called from there.
 * Returns sendAndWait's convention -- 1 ACK, 0 NAK, -1 silence. */
int8_t GpsTime::sendTp5()
{
    uint8_t tp5[32] = {};
    /* Version 0. The interface description says "0x00 for this version", and a
     * 1 here is the FTS variant of the same message: it is acknowledged and
     * echoed either way, which is why this went unnoticed while nothing the
     * frame carried took effect. */
    tp5[1] = 0; /* version */
    /* The antenna lead's delay, nanoseconds (offset 4). Positive tells the
     * receiver the signal arrives late so it brings the pulse forward by the
     * same amount; leaving it at zero puts the whole cable's delay into the
     * pulse, where nothing downstream can tell it from a real offset. */
    unsigned long asked = 0;
    text::getList(Settings::instance().antDelayNs(), ',', asked);
    const auto cableNs = static_cast<int32_t>(asked > 200000 ? 200000 : asked);
    /* The calibration term, i4 nanoseconds at offset 24. Separate from the cable
     * delay so a measured correction and a lead length stay distinguishable. */
    long userAsked = 0;
    text::getList(Settings::instance().gpsUserDelayNs(), ',', userAsked);
    /* int32_t, not the long the clamp produces: putLe writes sizeof, and eight
     * bytes at 24 run over the flags at 28. */
    const int32_t userNs = static_cast<int32_t>(userAsked > 10000000L    ? 10000000L
                                                : userAsked < -10000000L ? -10000000L
                                                                         : userAsked);
    text::putLe(tp5 + 24, userNs);
    text::putLe(tp5 + 4, cableNs);
    /* freqPeriod and freqPeriodLock, in microseconds: isFreq (bit 3) is clear. */
    const uint32_t kOneHzUs = m_periodUs;
    /* Pulse width as a fraction of the period, 2^-32 units. Default stays the
     * module's own 100 ms, but it is settable: qErr is a statement about
     * generating the edge, and a 100 ms pulse is an unusual thing to quantify. */
    const uint32_t ratio =
        m_pulseMs >= 1000 ? 4294967295u
                          : static_cast<uint32_t>((static_cast<uint64_t>(m_pulseMs) << 32) / 1000);
    const uint32_t kRatio10 = ratio;
    text::putLe(tp5 + 8, kOneHzUs);
    text::putLe(tp5 + 12, kOneHzUs);
    text::putLe(tp5 + 16, kRatio10);
    text::putLe(tp5 + 20, kRatio10);
    /* active, lockGnssFreq, lockedOtherSet, alignToTow, rising, and no grid
     * bits: from protocol 16 the single gridUtcGps bit became a three-bit
     * gridUtcGnss, 0 = UTC, 1 = GPS, so leaving 7 clear selects UTC. Setting it
     * selects GPS and the TIM-TP time base then reports GNSS -- which is how the
     * wrong meaning was caught. */
    constexpr uint32_t kTp5Flags = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 5) | (1u << 6);
    text::putLe(tp5 + 28, kTp5Flags);

    /* Each result is acted on by the caller rather than by whoever reads the log
     * later: an ACK means the pulse shape is asserted, a NAK means it is not, and
     * a silence means the module does not know the message -- which for CFG-TP5
     * would be a very old receiver indeed. */
    if (cableNs != 0) {
        ESP_LOGW(TAG, "pulse cable delay set to %ld ns", static_cast<long>(cableNs));
    }
    const int8_t r = sendAndWait(0x06, 0x31, tp5, sizeof(tp5));
    /* Read it straight back, every time. The frame has a userConfigDelay at 24
     * that this firmware never writes, so whatever sits there came from
     * somewhere else and shifts every anchor by a constant -- the shape of the
     * milliseconds being chased. An ACK says "stored", never "stored what", so
     * the poll is the only way to know what the module is actually running. */
    sendUbx(0x06, 0x31, nullptr, 0);
    return r;
}

void GpsTime::configureModule()
{
    /* UBX-CFG-MSG (class 0x06, id 0x01): the payload is the message class, its
     * id, then one output rate per port -- DDC, UART1, UART2, USB, SPI. NAV-PVT
     * once a second on UART1, and the NMEA sentences off so the line carries
     * what is read here and nothing else.
     *
     * NAV-TIMELS is the leap-second notice: the current GPS-UTC offset, whether
     * a leap is pending, and how many seconds until it. At 24 bytes a second it
     * is noise on a 38400 line, and it is the only source for the leap indicator
     * a reply puts in its first byte.
     *
     * NAV-TIM-TP is the same trade for the pulse itself: 16 bytes a second
     * carrying where the pulse nominally is and the receiver's own quantisation
     * error in picoseconds. It would turn the root delay from "wired, so zero"
     * into a number measured on this board -- and this unit rejects it, so it is
     * dormant here and the delay is measured from the servo's residuals instead. */
    static constexpr uint8_t kNavPvt[8]    = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kNavTimels[8] = {0x01, 0x26, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSilence[][2] = {
        {0xF0, 0x00}, /* GGA */ {0xF0, 0x01}, /* GLL */ {0xF0, 0x02}, /* GSA */
        {0xF0, 0x03}, /* GSV */ {0xF0, 0x04}, /* RMC */ {0xF0, 0x05}, /* VTG */
        {0xF0, 0x08},                                                 /* ZDA */
    };

    /* UBX-CFG-TP5 (class 0x06, id 0x31) first, because it is the frame that
     * decides whether the anchors mean anything. Until now the pulse was left at
     * the module's default and the README told the reader to check it; a pulse
     * moved in u-center is the one fault that makes this server confidently
     * wrong rather than merely stale, so it is asserted at every boot.
     *
     * 1 Hz, and 1 Hz again for the locked case so a receiver that thinks it is
     * being clever cannot slow the pulse down when it loses the fix. Rising edge
     * (bit 6), UTC grid (bit 7), aligned to the top of the second (bit 5), 100 ms
     * wide as a ratio of the period -- the module's own default width, so this
     * asserts the shape rather than changing it.
     *
     * Offset 4 is the cable delay in nanoseconds, and the module's own reply to
     * a CFG-TP5 poll is 32 bytes with the flags at 28 and this field at 4: it is
     * one antenna-delay slot, not the antCableDelay/rfGroupDelay pair the
     * 36-byte M8 layout has, so a 32-byte frame is the whole message and the
     * delay is the first word of it. */
    /* UBX-CFG-RATE (0x06 0x08): 1 Hz measurement, one measurement per
     * navigation solution, UTC time reference. Never set here before -- the
     * module's own default was left to apply -- and the interface description
     * is explicit that "the rate of UBX-TIM-TP is bound to the measurement
     * rate". In 0-D mode that binding is evidently what starves TIM-TP: the
     * message stops entirely and even a poll goes unanswered, which is what
     * cost a morning of blaming the receiver. Assert it rather than inherit it,
     * and assert it before the pulse. */
    {
        constexpr uint8_t kRate[6] = {0xE8, 0x03, 0, 0, 0, 1}; /* 1000 ms, 1:1, UTC */
        const int8_t      rr       = sendAndWait(0x06, 0x08, kRate, sizeof(kRate));
        m_rateCfg                  = rr == 1 ? 1 : rr == 0 ? 2 : 0;
        if (rr == 1) {
            ESP_LOGI(TAG, "CFG-RATE accepted: 1 Hz measurement, UTC reference");
        } else {
            ESP_LOGW(TAG, "CFG-RATE %s", rr == 0 ? "rejected" : "unanswered");
        }
    }
    /* UBX-CFG-NAV5 (0x06 0x24): dynamic model and UTC standard. Never set here
     * either, and both defaults are wrong for this box.
     *
     * dynModel stays at 0 (portable), which tells a receiver on a mast that it
     * may move; stationary (2) is what the timing products expect and is what
     * stops the navigation filter chasing a solution that is only noise.
     *
     * utcStandard stays at 0 (automatic) while CFG-TP5 below asks for a
     * UTC-aligned pulse. The datasheet is explicit that the standard "should be
     * selected in CFG-NAV5 of which the receiver has knowledge" and that other
     * selections "may result in relatively large timing uncertainties until the
     * offset between GNSS time and the selected UTC becomes available". GPS is
     * the one carried in the signals this box actually tracks, so it is the one
     * that is known.
     *
     * Mask 0x0401 selects exactly those two fields, so a firmware default behind
     * them cannot be clobbered by accident. Asserted before the pulse, so the
     * UTC that the pulse is aligned to is already defined. */
    {
        uint8_t nav5[36] = {};
        text::putLe(nav5 + 0, 0x0401u); /* mask: dynModel | utcStandard */
        nav5[2]         = 2;            /* dynModel: stationary */
        nav5[30]        = 3;            /* utcStandard: UTC(USNO) from GPS */
        const int8_t nr = sendAndWait(0x06, 0x24, nav5, sizeof(nav5));
        m_nav5Cfg       = nr == 1 ? 1 : nr == 0 ? 2 : 0;
        if (nr == 1) {
            ESP_LOGI(TAG, "CFG-NAV5 accepted: stationary platform, UTC from GPS");
        } else {
            ESP_LOGW(TAG, "CFG-NAV5 %s: dynamic model and UTC standard are defaults",
                     nr == 0 ? "rejected" : "unanswered");
        }
    }
    int8_t r   = sendTp5();
    m_tp5Acked = r == 1;
    m_tp5Naked = r == 0;
    if (r == 1) {
        ESP_LOGI(TAG, "CFG-TP5 accepted: pulse is 1 Hz, UTC, rising");
    } else {
        ESP_LOGW(TAG, "CFG-TP5 %s: the pulse is NOT asserted", r == 0 ? "rejected" : "unanswered");
    }

    /* UBX-CFG-ANT (class 0x06, id 0x13). The bias tee on the RF input is not
     * something this firmware used to touch, so whether a preamp was powered
     * depended entirely on whatever the module's saved configuration said -- a
     * user swapping antennas had no way to know from here. It is now asserted:
     * supply voltage control on, short detection on, and power down on a short so
     * a bad lead cannot cook the supply. Open-circuit detection is left out
     * because this firmware refuses it.
     *
     * The detect bits are the point of the exercise: MON-HW reports what they
     * find as aStatus, so "is the antenna plugged in and powered" becomes a
     * number in /healthz instead of a guess. A receiver without a supervisor
     * NAKs this and the whole group stays at DONTKNOW. */
    /* flags 0x0B: svcs | scd | pdwnOnSCD. Bit 2, open-circuit detection, is
     * rejected by this firmware -- setting it NAKs the whole message, which is
     * what made the supervisor look absent for an afternoon. Pins 20 and 27 are
     * the EXTINT pair the board wires to the switch, AADET first. */
    constexpr uint8_t kCfgAnt[4] = {0x0B, 0x00, 20, 27};
    r                            = sendAndWait(0x06, 0x13, kCfgAnt, sizeof(kCfgAnt));
    m_antCfgAcked                = r == 1;
    m_antCfgNaked                = r == 0;
    if (r == 1) {
        ESP_LOGI(TAG, "CFG-ANT accepted: antenna bias on, short detection, power-down on short");
    } else {
        ESP_LOGW(TAG, "CFG-ANT %s: antenna bias is whatever the module was left set to",
                 r == 0 ? "rejected" : "unanswered");
    }

    /* UBX-CFG-GNSS (class 0x06, id 0x3E). Which constellations the receiver
     * solves, and how many of its 32 channels each may have.
     *
     * **BeiDou and GLONASS are mutually exclusive on this receiver**, and that
     * is not documented anywhere obvious -- it took a bisect over every
     * combination here. Enabling BeiDou is NAKed while GLONASS is on, and
     * NAKed while Galileo is on for the same reason. What is accepted is
     * Galileo with GLONASS, or BeiDou without either. The numbers decided the
     * choice: Galileo with GLONASS and QZSS takes the sky from 22 satellites in
     * view to 37 and lifts the median elevation from 35 to 49 degrees, which is
     * the axis a survey cares about. BeiDou would have replaced GLONASS rather
     * than added to it.
     *
     * SBAS and IMES stay off. Nothing here applies SBAS corrections, so its
     * two geostationary satellites would only be two more low-elevation
     * measurements for a channel; IMES is an indoors system.
     *
     * The channel counts are the receiver's own reported defaults, untouched:
     * they are already the split u-blox ships for a five-constellation mask,
     * and inventing a different one is a guess against a working baseline. */
    {
        struct GnssBlock {
            uint8_t  id;
            uint8_t  res;
            uint8_t  max;
            uint32_t flags;
        };
        /* flags: bit 0 enables; the sigCfgMask the receiver reports sits above
         * it and is kept verbatim, since only the enable bit was ever the
         * question. */
        const GnssBlock kBlocks[] = {
            {0, 8, 16, 0x01010000}, /* GPS      */
            {1, 1, 3, 0x01010000},  /* SBAS     */
            {2, 4, 8, 0x01010000},  /* Galileo  */
            {3, 8, 16, 0x01010000}, /* BeiDou   */
            {4, 0, 8, 0x03010000},  /* IMES     */
            {5, 0, 3, 0x05010000},  /* QZSS     */
            {6, 8, 14, 0x01010000}, /* GLONASS  */
        };
        constexpr int  kCount              = static_cast<int>(sizeof(kBlocks) / sizeof(kBlocks[0]));
        uint8_t        cfg[4 + 8 * kCount] = {0x00, 32, 32, kCount};
        const uint32_t mask                = m_gnssMask;
        for (int i = 0; i < kCount; ++i) {
            const uint32_t flags =
                kBlocks[i].flags | (((mask >> kBlocks[i].id) & 1u) != 0 ? 1u : 0u);
            uint8_t *b = cfg + 4 + 8 * i;
            b[0]       = kBlocks[i].id;
            b[1]       = kBlocks[i].res;
            b[2]       = kBlocks[i].max;
            b[3]       = 0;
            text::putLe(b + 4, flags);
        }
        r           = sendAndWait(0x06, 0x3E, cfg, sizeof(cfg));
        m_gnssAcked = r == 1;
        m_gnssNaked = r == 0;
        if (r == 1) {
            ESP_LOGI(TAG, "CFG-GNSS accepted: constellations 0x%02x", static_cast<unsigned>(mask));
        } else {
            ESP_LOGW(TAG, "CFG-GNSS %s: constellation set is whatever the module had",
                     r == 0 ? "rejected" : "unanswered");
        }
    }

    r = sendAndWait(0x06, 0x01, kNavPvt, sizeof(kNavPvt));
    if (r != 1) {
        ESP_LOGW(TAG, "NAV-PVT enable %s", r == 0 ? "rejected" : "unanswered");
    }
    r = sendAndWait(0x06, 0x01, kNavTimels, sizeof(kNavTimels));
    if (r != 1) {
        ESP_LOGW(TAG, "NAV-TIMELS enable %s", r == 0 ? "rejected" : "unanswered");
    }

    /* NAV-TIM-TP needs protocol 15. This module acknowledges the enable and then
     * sends nothing, which is why the answer is recorded in three states rather
     * than two: "accepted" and "on the wire" are not the same thing. */
    const int8_t tos = sendAndWait(0x06, 0x01, kTimTosEnable, sizeof(kTimTosEnable));
    m_tosCfg         = tos == 1 ? 1 : tos == 0 ? 2 : 0;
    /* Rate zero, then poll it: see the poll in task(). The rate is still asked
     * for, because a module that honours it saves the poll. */
    r = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
    /* Armed whether or not it was acknowledged: the answer to the enable has
     * been everything from an ACK to nothing at all on this module, and a poll
     * is the same request with no rate attached. */
    m_timTpCfg   = 1;
    m_timTpAcked = r == 1;
    m_timTpNaked = r == 0;
    if (r == 0) {
        ESP_LOGW(TAG, "NAV-TIM-TP enable rejected: no TIM-TP on this firmware");
    } else if (r == 1) {
        ESP_LOGI(TAG, "NAV-TIM-TP enable accepted");
    }

    /* UBX-NAV-SAT at half rate. Twelve bytes per satellite with a 16-satellite
     * sky is 200 bytes, so once a second would spend a fifth of even the
     * current line on diagnostics; once every two seconds is plenty to compare
     * two antennas and leaves the wire to the messages the clock needs. */
    {
        constexpr uint8_t kNavSat[8] = {0x01, 0x35, 0, 2, 0, 0, 0, 0};
        r                            = sendAndWait(0x06, 0x01, kNavSat, sizeof(kNavSat));
        if (r != 1) {
            ESP_LOGW(TAG, "NAV-SAT enable %s: no C/N0 in /healthz",
                     r == 0 ? "rejected" : "unanswered");
        }
    }

    /* Geometry and clock quality. DOP and STATUS are small and move in minutes,
     * so they go at half rate; CLOCK is the one worth every second because tAcc
     * is what the reference could quote if it ever needed a stated accuracy.
     * MON-RF is a front-end trend and once per five seconds is enough to see a
     * jammer arrive. All four are diagnostics: none of them feeds the clock. */
    {
        struct Extra {
            const char *name;
            uint8_t     cls;
            uint8_t     id;
            uint8_t     div;
        };
        const Extra kExtras[] = {
            {"NAV-DOP", 0x01, 0x04, 2},
            {"NAV-CLOCK", 0x01, 0x22, 1},
            {"NAV-STATUS", 0x01, 0x03, 2},
            {"MON-RF", 0x0A, 0x38, 5},
            /* Link health. Slow on purpose: these measure the line, so they
             * must not be a meaningful part of what loads it. */
            {"MON-IO", 0x0A, 0x02, 10},
            {"MON-RXBUF", 0x0A, 0x07, 5},
            {"MON-TXBUF", 0x0A, 0x08, 5},
            {"MON-MSGPP", 0x0A, 0x06, 10},
        };
        for (const Extra &extra : kExtras) {
            const uint8_t enable[8] = {extra.cls, extra.id, 0, extra.div, 0, 0, 0, 0};
            const int8_t  res       = sendAndWait(0x06, 0x01, enable, sizeof(enable));
            if (extra.id == 0x38) {
                m_rfCfg = res == 1 ? 1 : res == 0 ? 2 : 0;
            }
            if (res != 1) {
                ESP_LOGW(TAG, "%s enable %s: absent from /gps/info", extra.name,
                         res == 0 ? "rejected" : "unanswered");
            }
        }
    }

    /* UBX-TIM-SVIN (0x0D 0x04), once every ten seconds. It is the survey's own
     * report -- duration, running mean position, mean variance, observations --
     * and ten seconds is fresh enough: a survey runs for hours and its numbers
     * move in minutes. NOT UBX-NAV-SVIN, which this firmware NAKs; the message
     * exists in the TIM dialect for exactly this product. */
    {
        constexpr uint8_t kTimSvin[8] = {0x0D, 0x04, 0, 10, 0, 0, 0, 0};
        r                             = sendAndWait(0x06, 0x01, kTimSvin, sizeof(kTimSvin));
        if (r != 1) {
            ESP_LOGW(TAG, "TIM-SVIN enable %s: no survey progress",
                     r == 0 ? "rejected" : "unanswered");
        }
    }

    /* UBX-MON-VER poll. The protocol version decides which of the above exist
     * at all -- NAV-TIM-TP needs protocol 15 or later -- so it is asked for
     * rather than inferred from silence. This receiver never answers it, which
     * is itself worth the log line: absent `gps_module` is a fact about the
     * module, not about the wiring. */
    if (sendAndWait(0x0A, 0x04, nullptr, 0, 1500000) == -2) {
        ESP_LOGI(TAG, "MON-VER unanswered: protocol version unknown");
    }

    for (const auto &msg : kSilence) {
        const uint8_t off[8] = {msg[0], msg[1], 0, 0, 0, 0, 0, 0};
        sendAndWait(0x06, 0x01, off, sizeof(off));
    }

    /* Last, because it describes state this function does not set. Asking the
     * receiver what its time mode is after every reconfigure means a reboot, a
     * reset or somebody in u-center cannot leave the firmware believing the
     * position is moving when it is held -- or the other way round. */
    if (!m_skipAdopt) {
        adoptTimeMode();
    }

    /* Deliberately not followed by UBX-CFG-CFG: saving this to the receiver's
     * flash would outlive the firmware and take away the module's own defaults,
     * which is a rude thing to do to a receiver someone may want to read with
     * u-center later. Sending it at every boot costs eleven short frames. */
}

int8_t GpsTime::sendAndWait(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                            uint32_t timeoutUs)
{
    m_awaitCls    = cls;
    m_awaitId     = id;
    m_awaitResult = -1;
    sendUbx(cls, id, payload, len);

    /* 250 ms is several reply times at 9600 baud for a short answer. A long
     * one needs longer and says so: MON-VER is 250 bytes, which is a quarter of
     * a second on the wire at 9600 all by itself, and a 250 ms wait timed out on
     * it and logged the module as silent when it had in fact answered. */
    pump(timeoutUs);
    return m_awaitResult;
}

void GpsTime::pump(int64_t timeoutUs)
{
    uint8_t       buf[64];
    const int64_t deadline = esp_timer_get_time() + timeoutUs;

    while (m_awaitResult == -1 && esp_timer_get_time() < deadline) {
        /* While a client is attached the bridge task is the one reader, and
         * everything it reads goes through feed(). Reading here as well took
         * bytes out of the client's stream: every antenna request, warm retry
         * and TIM-TP probe swallowed whatever arrived inside its window, so
         * u-center saw a reply-shaped hole where its own poll answer should
         * have been. One reader, and this side only waits for the parser to
         * report the ACK the bridge brought in. */
        if (m_bridgeActive) {
            vTaskDelay(1); /* one tick: 2 ms rounds to none at 100 Hz, a spin */
            continue;
        }
        const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(20));
        m_decoder.feed(std::span(buf, static_cast<std::size_t>(got > 0 ? got : 0)));
    }
    if (m_awaitResult == -1) {
        m_awaitResult = -2;
    }
}

void GpsTime::sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8 + 120];
    if (len > sizeof(frame) - 8) {
        return;
    }

    size_t n   = 0;
    frame[n++] = 0xB5;
    frame[n++] = 0x62;
    frame[n++] = cls;
    frame[n++] = id;
    frame[n++] = static_cast<uint8_t>(len & 0xFF);
    frame[n++] = static_cast<uint8_t>(len >> 8);
    for (uint16_t i = 0; i < len; ++i) {
        frame[n++] = payload[i];
    }

    /* The UBX checksum is the 8-bit Fletcher over class..last payload byte. */
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (size_t i = 2; i < n; ++i) {
        ckA = static_cast<uint8_t>(ckA + frame[i]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
    frame[n++] = ckA;
    frame[n++] = ckB;

    uart_write_bytes(kUart, frame, n);
}

void GpsTime::adoptFixedPosition(long long x, long long y, long long z, unsigned long acc,
                                 bool validated)
{
    for (int i = 0; i < 3; ++i) {
        m_fixedMean[i] = static_cast<int32_t>(i == 0 ? x : i == 1 ? y : z);
    }
    /* Records written before the fix above hold the variance in mm squared. No
     * survey may ask for worse than 60 m, so anything past that is one of
     * those and has its root taken. */
    m_fixedAcc      = acc > 60000 ? isqrt(static_cast<uint32_t>(std::min(acc, 0xFFFFFFFFul))) : acc;
    m_fixedValid    = true;
    m_holdWanted    = true;
    m_holdValidated = validated;
}
