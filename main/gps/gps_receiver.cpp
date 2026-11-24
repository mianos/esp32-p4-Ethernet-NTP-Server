/* Receiver configuration, the baud scan, time mode and position hold, and
 * send-and-wait. */
#include "gps/fixpos.h"
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::applyBias(bool on)
{
    m_antBiasOn = on;
    gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
}

/* NMEA PUBX,41: CFG-PRT as a sentence, which still works when the saved UBX
 * input mask is zero. Sent three times, since a receiver at another rate may
 * only resynchronise partway through.
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
    char      sentence[48] = {};
    const int total = std::snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", bodyText, cs);
    if (total <= 0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        uart_write_bytes(kUart, sentence, static_cast<size_t>(total));
    }
}

/* CFG-CFG saving only the IO section, so a baud change survives a power cycle.
 * Message rates and the GNSS mask are not saved; they are sent every boot. */
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
    /* Protocol masks: bit 0 UBX, bit 1 NMEA, bit 2 RTCM, bit 5 RTCM3. */
    text::putLe(cfg + 12, 0x0003u); /* in: UBX | NMEA */
    text::putLe(cfg + 14, 0x0001u); /* out: UBX */
    return sendAndWait(0x06, 0x00, cfg, sizeof(cfg));
}

void GpsTime::adoptTimeMode()
{
    /* A poll; onCfgTmode stores the answer. */
    sendAndWait(0x06, 0x3D, nullptr, 0);
}

void GpsTime::saveTimeMode()
{
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
    /* The time mode is not saved in the module, so restore it from NVS. */
    Settings          &settings = Settings::instance();
    const std::string &mode     = settings.gpsTimeMode();

    if (mode == "fixed") {
        if (adoptStoredFix()) {
            ESP_LOGW(TAG, "position hold restored from NVS%s",
                     m_holdValidated ? ", survey-validated" : "");
            /* The task loop sends the hold once the receiver has a fix: TMODE2
             * restarts the pulse generator. */
            m_holdRetryUs    = kHoldRetryUs;
            m_holdAssertAtUs = esp_timer_get_time() + kHoldSettleUs;
            ESP_LOGW(TAG, "position hold will be asserted once the receiver has settled");
            return;
        }
        /* Fixed, but no position stored: survey again. */
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
    /* The callers decide whether a survey is wanted. */
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
    /* After a reset: put the stored hold back, or resume a survey if there is
     * none. Reads NVS, not m_fixedValid, which a reset leaves unknown. */
    if (!adoptStoredFix()) {
        /* Restarting a survey loses its elapsed time. */
        const std::string &mode = Settings::instance().gpsTimeMode();
        if ((mode == "survey" || mode == "fixed") && startSurveyFromSettings()) {
            ESP_LOGW(TAG, "survey restarted after the reset");
        }
        return;
    }

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

    /* Passes are counted, not inferred from the index, so a pass starting
     * mid-list still visits every rate. */
    if (++m_scanSteps >= kCount) {
        m_scanSteps = 0;
        ++m_silentPasses;
        /* Silent at every rate: send a hardware reset at each rate, at most
         * once a minute, and resume scanning at 9600. */
        const int64_t nowUs = esp_timer_get_time();
        /* Not after the first pass, which can end before the starting rate has
         * been retried. */
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

    m_decoder.reset();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_baudrate(%u): %s", static_cast<unsigned>(m_baud),
                 esp_err_to_name(err));
    }
    /* In case the UBX input mask is what is wrong. */
    if (!m_frameSeen) {
        sendPubxPortConfig(m_baud);
        esp_rom_delay_us(20000);
    }
    m_poorSecs  = 0;
    m_secFrames = 0;
    configureModule();
}

/* CFG-TP5 on its own, so a delay change does not rerun configureModule() and
 * lose acquisition. Returns 1 ACK, 0 NAK, -1 no answer. */
int8_t GpsTime::sendTp5()
{
    uint8_t tp5[32] = {};
    /* Version 0; 1 is the FTS variant, which is ACKed but has no effect here. */
    tp5[1] = 0; /* version */
    /* Antenna cable delay, ns, at offset 4: the receiver advances the pulse by
     * it. */
    unsigned long asked = 0;
    text::getList(Settings::instance().antDelayNs(), ',', asked);
    const auto cableNs = static_cast<int32_t>(asked > 200000 ? 200000 : asked);
    /* User delay, i4 ns at offset 24, kept apart from the cable delay. */
    long userAsked = 0;
    text::getList(Settings::instance().gpsUserDelayNs(), ',', userAsked);
    /* int32_t: putLe writes sizeof(value). */
    const int32_t userNs = static_cast<int32_t>(userAsked > 10000000L    ? 10000000L
                                                : userAsked < -10000000L ? -10000000L
                                                                         : userAsked);
    text::putLe(tp5 + 24, userNs);
    text::putLe(tp5 + 4, cableNs);
    /* Period in us, since isFreq (bit 3) is clear. */
    const uint32_t kOneHzUs = m_periodUs;
    /* Pulse width as a fraction of the period in 2^-32 units; 100 ms default. */
    const uint32_t ratio =
        m_pulseMs >= 1000 ? 4294967295u
                          : static_cast<uint32_t>((static_cast<uint64_t>(m_pulseMs) << 32) / 1000);
    const uint32_t kRatio10 = ratio;
    text::putLe(tp5 + 8, kOneHzUs);
    text::putLe(tp5 + 12, kOneHzUs);
    text::putLe(tp5 + 16, kRatio10);
    text::putLe(tp5 + 20, kRatio10);
    /* active, lockGnssFreq, lockedOtherSet, alignToTow, polarity rising.
     * gridUtcGnss (bits 7-10) left 0 selects UTC. */
    constexpr uint32_t kTp5Flags = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 5) | (1u << 6);
    text::putLe(tp5 + 28, kTp5Flags);

    if (cableNs != 0) {
        ESP_LOGW(TAG, "pulse cable delay set to %ld ns", static_cast<long>(cableNs));
    }
    const int8_t r = sendAndWait(0x06, 0x31, tp5, sizeof(tp5));
    /* Poll it back: an ACK does not say what was stored. */
    sendUbx(0x06, 0x31, nullptr, 0);
    return r;
}

void GpsTime::configureModule()
{
    /* UBX-CFG-MSG payload: class, id, then a rate per port (DDC, UART1, UART2,
     * USB, SPI). NAV-TIMELS is the only source for the leap indicator. */
    static constexpr uint8_t kNavPvt[8]    = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kNavTimels[8] = {0x01, 0x26, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSilence[][2] = {
        {0xF0, 0x00}, /* GGA */ {0xF0, 0x01}, /* GLL */ {0xF0, 0x02}, /* GSA */
        {0xF0, 0x03}, /* GSV */ {0xF0, 0x04}, /* RMC */ {0xF0, 0x05}, /* VTG */
        {0xF0, 0x08},                                                 /* ZDA */
    };

    /* CFG-RATE: 1 Hz, one measurement per solution, UTC reference. TIM-TP's
     * rate is bound to the measurement rate. */
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
    /* CFG-NAV5, only dynModel and utcStandard (mask 0x0401): stationary, and
     * UTC(USNO) as broadcast by GPS, which this receiver tracks. Sent before
     * CFG-TP5, whose pulse is UTC-aligned. */
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

    /* CFG-ANT flags 0x0B: supply control, short detection, power down on short;
     * MON-HW then reports aStatus. Open-circuit detection (bit 2) makes this
     * firmware NAK the whole message. Pins 20 and 27 are the board's switch
     * pair, AADET first. */
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

    /* CFG-GNSS. This receiver NAKs BeiDou with GLONASS or Galileo enabled;
     * GPS+Galileo+QZSS+GLONASS gave the most satellites (37 against 22). SBAS
     * corrections are not used. Channel counts are the receiver's defaults. */
    {
        struct GnssBlock {
            uint8_t  id;
            uint8_t  res;
            uint8_t  max;
            uint32_t flags;
        };
        /* flags: bit 0 enables; sigCfgMask is as the receiver reported it. */
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

    /* TIM-TOS is optional; its answer is reported as tos_cfg. */
    const int8_t tos = sendAndWait(0x06, 0x01, kTimTosEnable, sizeof(kTimTosEnable));
    m_tosCfg         = tos == 1 ? 1 : tos == 0 ? 2 : 0;
    /* Enabled at 1 Hz and polled as well (see task()). */
    r = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
    /* Polling starts whatever the answer. */
    m_timTpCfg   = 1;
    m_timTpAcked = r == 1;
    m_timTpNaked = r == 0;
    if (r == 0) {
        ESP_LOGW(TAG, "NAV-TIM-TP enable rejected: no TIM-TP on this firmware");
    } else if (r == 1) {
        ESP_LOGI(TAG, "NAV-TIM-TP enable accepted");
    }

    /* NAV-SAT every 2 s: 12 bytes per satellite. */
    {
        constexpr uint8_t kNavSat[8] = {0x01, 0x35, 0, 2, 0, 0, 0, 0};
        r                            = sendAndWait(0x06, 0x01, kNavSat, sizeof(kNavSat));
        if (r != 1) {
            ESP_LOGW(TAG, "NAV-SAT enable %s: no C/N0 in /healthz",
                     r == 0 ? "rejected" : "unanswered");
        }
    }

    /* Diagnostics only; none of these feeds the clock. */
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
            /* Slow, so they do not load the line they measure. */
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

    /* TIM-SVIN every 10 s (this firmware NAKs NAV-SVIN). */
    {
        constexpr uint8_t kTimSvin[8] = {0x0D, 0x04, 0, 10, 0, 0, 0, 0};
        r                             = sendAndWait(0x06, 0x01, kTimSvin, sizeof(kTimSvin));
        if (r != 1) {
            ESP_LOGW(TAG, "TIM-SVIN enable %s: no survey progress",
                     r == 0 ? "rejected" : "unanswered");
        }
    }

    /* MON-VER, for the firmware and protocol version (reported as module). */
    if (sendAndWait(0x0A, 0x04, nullptr, 0, 1500000) == -2) {
        ESP_LOGI(TAG, "MON-VER unanswered: protocol version unknown");
    }

    for (const auto &msg : kSilence) {
        const uint8_t off[8] = {msg[0], msg[1], 0, 0, 0, 0, 0, 0};
        sendAndWait(0x06, 0x01, off, sizeof(off));
    }

    /* Read the time mode back, in case a reset or u-center changed it. */
    if (!m_skipAdopt) {
        adoptTimeMode();
    }

}

int8_t GpsTime::sendAndWait(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                            uint32_t timeoutUs)
{
    m_awaitCls    = cls;
    m_awaitId     = id;
    m_awaitResult = -1;
    sendUbx(cls, id, payload, len);

    /* Long replies such as MON-VER pass a longer timeout. */
    pump(timeoutUs);
    return m_awaitResult;
}

void GpsTime::pump(int64_t timeoutUs)
{
    uint8_t       buf[64];
    const int64_t deadline = esp_timer_get_time() + timeoutUs;

    while (m_awaitResult == -1 && esp_timer_get_time() < deadline) {
        /* With a bridge client the bridge is the only reader; wait for the ACK
         * it feeds through. */
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

bool GpsTime::adoptStoredFix()
{
    /* The record is text in NVS; fixpos.h is the one reader of it. */
    fixpos::Hold hold;
    if (!fixpos::parse(Settings::instance().gpsFixedPos(), hold)) {
        return false;
    }
    adoptFixedPosition(hold.x, hold.y, hold.z, hold.accMm, hold.validated);
    return true;
}

void GpsTime::adoptFixedPosition(long long x, long long y, long long z, unsigned long acc,
                                 bool validated)
{
    for (int i = 0; i < 3; ++i) {
        m_fixedMean[i] = static_cast<int32_t>(i == 0 ? x : i == 1 ? y : z);
    }
    /* Older records stored the variance (mm^2); no valid accuracy exceeds
     * 60 m, so larger values are converted. */
    m_fixedAcc      = acc > 60000 ? isqrt(static_cast<uint32_t>(std::min(acc, 0xFFFFFFFFul))) : acc;
    m_fixedValid    = true;
    m_holdWanted    = true;
    m_holdValidated = validated;
}
