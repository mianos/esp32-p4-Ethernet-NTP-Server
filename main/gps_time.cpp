#include "gps_time.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ntp_time.h"
#include "settings.h"

namespace {

/* ---- Wiring ------------------------------------------------------------ */

/* UART1, not UART0: the console owns UART0 on this board (sdkconfig.defaults),
 * and handing the module the same port as the log would be a bad trade. */
constexpr uart_port_t kUart   = UART_NUM_1;
constexpr int         kTxPin  = 15; /* esp TX -> module RX */
constexpr int         kRxPin  = 16; /* esp RX <- module TX */
constexpr gpio_num_t  kPpsPin = GPIO_NUM_14;

/* The module's factory default is first. The rest are there because a receiver
 * that has been through u-center once usually is not on 9600 any more, and
 * cycling through four baud rates at boot is cheaper for everyone than asking
 * the user to read a datasheet. */
constexpr uint32_t kBauds[] = {9600, 38400, 115200, 57600, 19200};

/* How long one baud rate gets to produce a single valid UBX frame. */
constexpr int64_t kScanAfterUs = 4000000;

/* An edge younger than this may still be the one the NAV-PVT describes: the
 * receiver emits it a few tens of milliseconds after the second boundary. Older
 * than this and the pairing would be ambiguous, so the message is dropped
 * instead of anchored a second out. */
constexpr int64_t kPairWindowUs = 900000;

/* Three missed edges. Two would demote the server on a single lost interrupt;
 * ten would hold a stopped module up for ten seconds. This is the window while
 * the servo has no rate for the crystal: with no ppm estimate there is nothing
 * to hold over *with*, and a clock whose second boundary is a guess is not a
 * stratum-1 clock. kHoldoverWindowUs is the window once there is one. */
constexpr int64_t kLockWindowUs = 3000000;

/* Ten minutes of holdover, once the rate is known. The limit is not the fit,
 * which is good to parts per billion and would carry hours; it is the crystal
 * wandering with temperature, which is why the dispersion below grows at 1 ppm
 * and a client can watch the claim decay. Past this, the honest answer is
 * "do not use me" again. */
constexpr int64_t kHoldoverWindowUs = 600000000;

/* What the anchor is worth while it is fresh: the PPS interrupt's latency, a
 * microsecond or two, plus esp_timer's own resolution. Stated rather than
 * discovered, and an order of magnitude above the measured agreement, because
 * root dispersion is a promise and not a boast. */
constexpr int64_t kBaseDispersionNs = 10000;

/* How fast that promise decays in a holdover: 1 ppm, i.e. 1000 ns of age per
 * second. A small XTAL across a room's temperature swing moves by about this
 * much, and no amount of fitting the last five minutes predicts it. */
constexpr int64_t kWanderNsPerS = 1000;

/* A step in (UTC - esp_timer) larger than this is not the crystal drifting: it
 * is the module moving the clock, or a pairing that landed a second out. Either
 * way the servo's history is about a different clock now, so it starts again. */
constexpr int64_t kRateResetNs = 100000000; /* 100 ms */

/* A larger estimate than this is a bad fit rather than a bad crystal (200 ppm
 * is already ten times a cheap XTAL's worst case), so it is clamped instead of
 * being applied to every timestamp. */
constexpr int32_t kRateLimitPpb = 200000;

/* Edge spacing below this is not a second passing: it is bounce on a header pin
 * or a module reconfiguring its pulse, and counting it would shift every anchor
 * after it by a whole second. */
constexpr int64_t kMinEdgeSpacingUs = 500000;

/* A correction larger than this is a step, not a slew. IDF implements adjtime()
 * at ~1.5% of elapsed time, so a second of error would take a minute to clear --
 * during which the served clock is knowingly wrong. */
constexpr int64_t kStepThresholdNs = 250000000; /* 250 ms */

constexpr int          kTaskStack    = 4096;
constexpr UBaseType_t  kTaskPriority = 4; /* below the responder: it answers packets */

/* NTP refid for this reference, spelled the way RFC 5905 spells it: four ASCII
 * bytes, the last a NUL that the responder's text formatter trims. */
constexpr uint32_t kRefIdGps = (static_cast<uint32_t>('G') << 24) | ('P' << 16) | ('S' << 8);

/* ---- UBX helpers ------------------------------------------------------- */

uint16_t le16(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

int32_t le32(const uint8_t *p)
{
    const uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(v);
}

/* Days since 1970-01-01 from a proleptic Gregorian date (Hinnant's
 * days_from_civil). Written out rather than reached for: timegm() is not in this
 * libc, and mktime() would apply the POSIX TZ from Settings -- turning UTC from
 * the receiver into local time, which is exactly the bug that would leave this
 * server an hour out. */
int64_t daysFromCivil(int year, unsigned month, unsigned day)
{
    year -= month <= 2;
    const int64_t  era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

/* NAV-PVT's date and time, sanity-checked before anything is believed. A
 * receiver that has just been powered up reports zeros or a 1980 default, and
 * stepping the clock to either is worse than waiting for the fix. */
bool civilToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
                  uint8_t second, time_t &out)
{
    if (year < 2000 || year > 2100 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 ||
        minute > 59 || second > 60) {
        return false;
    }
    out = static_cast<time_t>(daysFromCivil(year, month, day) * 86400 +
                              static_cast<int64_t>(hour) * 3600 +
                              static_cast<int64_t>(minute) * 60 + second);
    return true;
}

}  // namespace

GpsTime &GpsTime::instance()
{
    static GpsTime gps;
    return gps;
}

void IRAM_ATTR GpsTime::ppsIsr(void *arg)
{
    auto *self = static_cast<GpsTime *>(arg);
    /* esp_timer's counter is in IRAM (CONFIG_ESP_TIMER_IN_IRAM), which is what
     * lets this handler be an IRAM interrupt: an OTA write disables the flash
     * cache for milliseconds at a time, and an edge landing in that window must
     * not be a crash. */
    const int64_t nowUs = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&self->m_mux);
    /* The POST /gps/pps hook swallows the edge here rather than further up, so
     * what it produces is the real holdover code path and not an imitation of
     * it. One extra comparison, against a word that is zero unless somebody
     * asked for the test. */
    if (nowUs >= self->m_ppsSuppressUntilUs &&
        (self->m_ppsCount == 0 || nowUs - self->m_ppsMonoUs >= kMinEdgeSpacingUs)) {
        self->m_ppsMonoUs = nowUs;
        ++self->m_ppsCount;
    }
    portEXIT_CRITICAL_ISR(&self->m_mux);

}

void GpsTime::taskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->task();
}

esp_err_t GpsTime::begin()
{
    if (m_begun) {
        return ESP_OK;
    }

    uart_config_t config         = {};
    config.baud_rate             = static_cast<int>(kBauds[0]);
    config.data_bits             = UART_DATA_8_BITS;
    config.parity                = UART_PARITY_DISABLE;
    config.stop_bits             = UART_STOP_BITS_1;
    config.flow_ctrl             = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh   = 0;
    config.source_clk            = UART_SCLK_DEFAULT;

    /* A transmitter buffer is needed even though nothing is sent after boot: the
     * configuration messages go out through it. */
    esp_err_t err = uart_driver_install(kUart, 1024, 256, 0, nullptr, 0);
    if (err == ESP_OK) {
        err = uart_param_config(kUart, &config);
    }
    if (err == ESP_OK) {
        err = uart_set_pin(kUart, kTxPin, kRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART%d: %s -- no GPS", kUart, esp_err_to_name(err));
        return err;
    }

    gpio_config_t pps = {};
    pps.pin_bit_mask  = 1ull << kPpsPin;
    pps.mode          = GPIO_MODE_INPUT;
    pps.pull_up_en    = GPIO_PULLUP_DISABLE;
    /* Pulled down so an unconnected pin reads 0 rather than picking up noise;
     * the module drives the line push-pull, so the resistor never fights it. */
    pps.pull_down_en  = GPIO_PULLDOWN_ENABLE;
    pps.intr_type     = GPIO_INTR_POSEDGE;
    err               = gpio_config(&pps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: %s -- no PPS", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    /* IRAM flag: see ppsIsr(). The service is process-wide, so a second install
     * is not an error -- it means something else got here first, and the only
     * consequence is where this handler ends up. */
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR service already installed: the PPS handler may not be in IRAM");
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(kPpsPin, &GpsTime::ppsIsr, this);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS interrupt on GPIO%d: %s", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    if (xTaskCreate(&GpsTime::taskEntry, "gps", kTaskStack, this, kTaskPriority, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "cannot start the GPS task");
        return ESP_ERR_NO_MEM;
    }

    m_begun       = true;
    m_baud        = kBauds[0];
    m_nextScanUs  = esp_timer_get_time() + kScanAfterUs;
    m_nextCheckUs = esp_timer_get_time() + 1000000;

    ESP_LOGI(TAG, "UART%d %u 8N1 (esp TX %d -> module RX, esp RX %d <- module TX), PPS on GPIO%d",
             kUart, static_cast<unsigned>(m_baud), kTxPin, kRxPin, static_cast<int>(kPpsPin));
    return ESP_OK;
}

void GpsTime::task()
{
    uint8_t buf[64];

    /* Ask before listening: a receiver still in its factory NMEA mode is using
     * most of a 9600-baud line, and switching that off is the fastest way to
     * make room for the messages this reference actually reads. Harmless noise
     * if there is nothing on the other end. */
    configureModule();

    for (;;) {
        const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < got; ++i) {
            feed(buf[i]);
        }

        const int64_t nowUs = esp_timer_get_time();
        if (!m_frameSeen && nowUs >= m_nextScanUs) {
            advanceBaud();
            m_nextScanUs = nowUs + kScanAfterUs;
        }
        if (nowUs >= m_nextCheckUs) {
            logLockChange();
            m_nextCheckUs = nowUs + 1000000;
        }
    }
}

void GpsTime::advanceBaud()
{
    constexpr int kCount = static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0]));

    ++m_baudIndex;
    if (m_baudIndex >= kCount) {
        /* One pass only, then start again: a module that is powered up later
         * still gets found, without a warning every twenty seconds forever. */
        m_baudIndex = 0;
        if (!m_noDataLogged) {
            ESP_LOGW(TAG, "no UBX frames at any baud rate: check module TX -> GPIO%d "
                          "(esp RX) and PPS -> GPIO%d", kRxPin, static_cast<int>(kPpsPin));
            m_noDataLogged = true;
        }
    } else if (!m_noDataLogged) {
        ESP_LOGI(TAG, "no UBX frames yet, trying %u baud", static_cast<unsigned>(kBauds[m_baudIndex]));
    }

    m_baud = kBauds[m_baudIndex];
    const esp_err_t err = uart_set_baudrate(kUart, m_baud);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_baudrate(%u): %s", static_cast<unsigned>(m_baud),
                 esp_err_to_name(err));
    }
    configureModule();
}

void GpsTime::configureModule()
{
    /* UBX-CFG-MSG (class 0x06, id 0x01): the payload is the message class, its
     * id, then one output rate per port -- DDC, UART1, UART2, USB, SPI. NAV-PVT
     * once a second on UART1, and the NMEA sentences off so the line carries
     * what is read here and nothing else.
     *
     * NAV-TIMELS is the leap-second notice: the current GPS-UTC offset, whether
     * a leap is pending, and how many seconds until it. It is 24 bytes a second
     * on a line that carries 1 Hz at 9600 baud with room to spare, and it is the
     * only source for the leap indicator a reply puts in its first byte. */
    static constexpr uint8_t kNavPvt[8]    = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kNavTimels[8] = {0x01, 0x26, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSilence[][2] = {
        {0xF0, 0x00}, /* GGA */ {0xF0, 0x01}, /* GLL */ {0xF0, 0x02}, /* GSA */
        {0xF0, 0x03}, /* GSV */ {0xF0, 0x04}, /* RMC */ {0xF0, 0x05}, /* VTG */
        {0xF0, 0x08}, /* ZDA */
    };

    sendUbx(0x06, 0x01, kNavPvt, sizeof(kNavPvt));
    sendUbx(0x06, 0x01, kNavTimels, sizeof(kNavTimels));


    for (const auto &msg : kSilence) {
        const uint8_t off[8] = {msg[0], msg[1], 0, 0, 0, 0, 0, 0};
        sendUbx(0x06, 0x01, off, sizeof(off));
    }

    /* Deliberately not followed by UBX-CFG-CFG: saving this to the receiver's
     * flash would outlive the firmware and take away the module's own defaults,
     * which is a rude thing to do to a receiver someone may want to read with
     * u-center later. Sending it at every boot costs nine short frames. */
}

void GpsTime::sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8 + 120];
    if (len > sizeof(frame) - 8) {
        return;
    }

    size_t n     = 0;
    frame[n++]   = 0xB5;
    frame[n++]   = 0x62;
    frame[n++]   = cls;
    frame[n++]   = id;
    frame[n++]   = static_cast<uint8_t>(len & 0xFF);
    frame[n++]   = static_cast<uint8_t>(len >> 8);
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

void GpsTime::feed(uint8_t byte)
{
    switch (m_parse) {
    case Parse::Sync1:
        if (byte == 0xB5) {
            m_parse = Parse::Sync2;
        }
        break;

    case Parse::Sync2:
        /* Anything but the second sync byte is rubbish, including NMEA text; a
         * repeated 0xB5 might be the start of a real frame, so it is kept. */
        if (byte == 0x62) {
            m_parse = Parse::Class;
        } else if (byte != 0xB5) {
            m_parse = Parse::Sync1;
        }
        break;

    case Parse::Class:
        m_cls   = byte;
        m_ckA   = byte;
        m_ckB   = byte;
        m_parse = Parse::Id;
        break;

    case Parse::Id:
        m_id = byte;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        m_parse = Parse::LenLo;
        break;

    case Parse::LenLo:
        m_len = byte;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        m_parse = Parse::LenHi;
        break;

    case Parse::LenHi:
        m_len |= static_cast<uint16_t>(byte) << 8;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        m_got      = 0;
        m_overflow = m_len > sizeof(m_payload);
        m_parse    = m_len == 0 ? Parse::CkA : Parse::Payload;
        break;

    case Parse::Payload:
        if (!m_overflow) {
            m_payload[m_got] = byte;
        }
        ++m_got;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        if (m_got >= m_len) {
            m_parse = Parse::CkA;
        }
        break;

    case Parse::CkA:
        m_parse = byte == m_ckA ? Parse::CkB : Parse::Sync1;
        break;

    case Parse::CkB:
        /* A frame whose checksum does not match is dropped, not guessed at: this
         * is the only place a bad byte could turn into a wrong second. */
        if (byte == m_ckB && !m_overflow) {
            onFrame(m_cls, m_id, m_payload, m_len);
        }
        m_parse = Parse::Sync1;
        break;
    }
}

void GpsTime::onFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    if (!m_frameSeen) {
        m_frameSeen = true;
        ESP_LOGI(TAG, "UBX frames arriving at %u baud", static_cast<unsigned>(m_baud));
    }

    if (cls == 0x05 && id == 0x01) { /* UBX-ACK-ACK */
        if (!m_ackLogged) {
            m_ackLogged = true;
            ESP_LOGI(TAG, "module accepted the configuration (NAV-PVT + NAV-TIMELS at 1 Hz, NMEA off)");
        }
    } else if (cls == 0x05 && id == 0x00) { /* UBX-ACK-NAK */
        if (!m_nakLogged) {
            m_nakLogged = true;
            ESP_LOGW(TAG, "module rejected a configuration message (class 0x%02x id 0x%02x)",
                     len >= 2 ? payload[0] : 0, len >= 2 ? payload[1] : 0);
        }
    } else if (cls == 0x01 && id == 0x07) { /* UBX-NAV-PVT */
        onNavPvt(payload, len);
    } else if (cls == 0x01 && id == 0x26) { /* UBX-NAV-TIMELS */
        onNavTimels(payload, len);
    }
}

namespace {

const char *fixText(uint8_t fix)
{
    switch (fix) {
    case 0:  return "none";
    case 1:  return "dead-reckoning";
    case 2:  return "2D";
    case 3:  return "3D";
    case 4:  return "GNSS+DR";
    case 5:  return "time-only";
    default: return "unknown";
    }
}

}  // namespace

void GpsTime::onNavPvt(const uint8_t *p, uint16_t len)
{
    /* NAV-PVT is 92 bytes. The first 24 carry everything used here, but the
     * frame has to be complete or those offsets mean nothing. */
    if (len < 92) {
        return;
    }

    const uint8_t valid = p[11];
    time_t        stamp = 0;
    /* validDate | validTime. u-blox only sets both once the receiver has an
     * almanac and a position estimate, which is exactly the point at which its
     * UTC can be believed -- before that the fields hold a 1980 default. */
    const bool    haveTime = (valid & 0x03) == 0x03 &&
                          civilToEpoch(le16(p + 4), p[6], p[7], p[8], p[9], p[10], stamp);
    const int32_t nano   = le32(p + 16); /* offset into the second, ~0 at 1 Hz */
    const uint8_t fix    = p[20];
    const uint8_t sats   = p[23];

    portENTER_CRITICAL(&m_mux);
    m_timeValid  = haveTime;
    m_fixType    = fix;
    m_satellites = sats;
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

void GpsTime::onNavTimels(const uint8_t *p, uint16_t len)
{
    /* 24 bytes, version 0, from the u-blox 8 interface description:
     *   8  srcOfCurrLs (u1)   9  currLs (i1)
     *   10 srcOfLsChange (u1) 11 lsChange (i1)
     *   12 timeToLsEvent (i4) 16 dateOfLsGpsWn (u2) 18 dateOfLsGpsDn (u2)
     *   23 valid (u1): bit 0 says currLs is known, bit 1 says the event time is */
    if (len < 24) {
        return;
    }

    const int8_t  current = static_cast<int8_t>(p[9]);
    const int8_t  change  = static_cast<int8_t>(p[11]);
    const int32_t inS     = le32(p + 12);
    const uint8_t valid   = p[23];
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
            ESP_LOGW(TAG, "leap: GPS-UTC offset %+d s, and %+d s in %ld s (leap indicator 1 for that minute)",
                     m_leapSeconds, m_leapChange, static_cast<long>(m_leapInS));
        }
    }
}

void GpsTime::anchor(time_t second)
{
    int64_t  ppsMonoUs = 0;
    uint32_t ppsCount  = 0;

    portENTER_CRITICAL(&m_mux);
    ppsMonoUs = m_ppsMonoUs;
    ppsCount  = m_ppsCount;
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

    /* What the servo predicted for this instant before it knew. Across an
     * ordinary second this is a check on the rate; across a holdover it is the
     * error a client was served while the edge was missing, and the only
     * honest way to quote what a holdover was worth. */
    int64_t predictedNs = 0;
    int64_t gapUs       = 0;
    {
        portENTER_CRITICAL(&m_mux);
        if (m_anchorValid && ppsMonoUs > m_anchorMonoUs) {
            gapUs       = ppsMonoUs - m_anchorMonoUs;
            predictedNs = m_anchorUtcNs + gapUs * 1000 - gapUs * m_ratePpb / 1000000;
        }
        portEXIT_CRITICAL(&m_mux);
    }

    portENTER_CRITICAL(&m_mux);
    m_anchorUtcNs  = static_cast<int64_t>(second) * 1000000000;
    m_anchorMonoUs = ppsMonoUs;
    m_anchorValid  = true;
    m_lastFix      = second;
    portEXIT_CRITICAL(&m_mux);

    if (gapUs >= kLockWindowUs) {
        /* The edge had gone stale and the servo carried the clock: this is what
         * that cost, and it is logged rather than served in the hope that
         * nobody looks. */
        const int64_t errorNs = static_cast<int64_t>(second) * 1000000000 - predictedNs;
        portENTER_CRITICAL(&m_mux);
        m_lastHoldoverUs    = gapUs;
        m_lastHoldoverErrNs = errorNs;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG, "PPS back after %.1f s: the servo was %+lld us out across the gap "
                      "(%+.3f ppm residual)",
                 static_cast<double>(gapUs) / 1e6, static_cast<long long>(errorNs / 1000),
                 static_cast<double>(errorNs) / static_cast<double>(gapUs) * 1e3);
    }

    /* The same pair is one sample of the crystal against the GPS: the anchor
     * says where the second started, this says how fast our own counter got
     * here. */
    pushRateSample(ppsMonoUs, static_cast<int64_t>(second) * 1000000000);

    steer();
}

void GpsTime::pushRateSample(int64_t monoUs, int64_t utcNs)
{
    const int64_t offsetNs = utcNs - monoUs * 1000;

    if (m_lastRateOffsetSet &&
        (offsetNs - m_lastRateOffsetNs > kRateResetNs || m_lastRateOffsetNs - offsetNs > kRateResetNs)) {
        /* The module moved the clock under us, or a pairing landed a second out.
         * A slope fitted across that step is about nothing, so the window starts
         * again -- and a second out is worth saying out loud, because it is the
         * one failure that would otherwise look like a healthy clock. */
        const int64_t stepMs = (offsetNs - m_lastRateOffsetNs) / 1000000;
        m_rateCount          = 0;
        m_rateNext           = 0;
        ESP_LOGW(TAG, "servo: %+lld ms step in the reference, refitting the rate",
                 static_cast<long long>(stepMs));
    }
    m_lastRateOffsetNs  = offsetNs;
    m_lastRateOffsetSet = true;

    m_rate[m_rateNext] = RateSample{monoUs, utcNs};
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

    /* Least squares of UTC nanoseconds against esp_timer microseconds, both
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
        sumX += m_rate[i].monoUs - base.monoUs;
        sumY += m_rate[i].utcNs - base.utcNs;
    }
    const double meanX = static_cast<double>(sumX) / m_rateCount;
    const double meanY = static_cast<double>(sumY) / m_rateCount;

    double sxy = 0.0;
    double sxx = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoUs - base.monoUs) - meanX;
        double       dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY;
        sxy += dx * dy;
        sxx += dx * dx;
    }
    if (sxx <= 0.0) {
        return;
    }

    /* A perfect esp_timer gives exactly 1000 ns per microsecond. A counter that
     * runs fast by p gives 1000 * (1 - p), because it covers each GPS second in
     * slightly fewer of its own microseconds -- so p, in parts per billion, is
     * 1e9 - slope * 1e6. */
    const double  slope = sxy / sxx;
    int64_t       ppb   = static_cast<int64_t>(std::llround(1e9 - slope * 1e6));
    if (ppb > kRateLimitPpb) {
        ppb = kRateLimitPpb;
    } else if (ppb < -kRateLimitPpb) {
        ppb = -kRateLimitPpb;
    }

    portENTER_CRITICAL(&m_mux);
    m_ratePpb     = static_cast<int32_t>(ppb);
    m_rateSamples = static_cast<uint16_t>(m_rateCount);
    portEXIT_CRITICAL(&m_mux);

    /* Once when it is first known, and then only when it moves by enough to be
     * worth a line: a servo that logs its estimate every second has made the
     * console unusable for everything else. */
    if (!m_rateLogged || std::llabs(ppb - m_lastLoggedPpb) > 500) {
        /* Residual: the thing that says the samples are actually samples of a
         * clock and not of the receiver's mood. */
        double sse = 0.0;
        for (int i = 0; i < m_rateCount; ++i) {
            const double dx    = static_cast<double>(m_rate[i].monoUs - base.monoUs) - meanX;
            const double dy    = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY;
            const double resid = dy - slope * dx;
            sse += resid * resid;
        }
        const double sigmaNs = m_rateCount > 2 ? std::sqrt(sse / (m_rateCount - 2)) : 0.0;

        ESP_LOGI(TAG, "servo: %+lld ppb (%+.3f ppm) over %d anchors, residual %.1f us",
                 static_cast<long long>(ppb), static_cast<double>(ppb) / 1000.0, m_rateCount,
                 sigmaNs / 1000.0);
        m_rateLogged    = true;
        m_lastLoggedPpb = ppb;
    }
}

void GpsTime::steer()
{
    /* Only while this source is the configured one. With timeSource=dhcp an
     * SNTP poll owns the system clock, and two owners fighting over it is worse
     * than either of them alone. */
    if (Settings::instance().timeSource() != Settings::TimeSource::Gps) {
        return;
    }

    struct timespec gps = {};
    if (!now(gps)) {
        return;
    }
    struct timespec sys = {};
    clock_gettime(CLOCK_REALTIME, &sys);

    const int64_t offsetNs = (static_cast<int64_t>(gps.tv_sec) - static_cast<int64_t>(sys.tv_sec)) *
                                 1000000000 +
                             (gps.tv_nsec - sys.tv_nsec);
    {
        portENTER_CRITICAL(&m_mux);
        m_offsetUs = offsetNs / 1000;
        portEXIT_CRITICAL(&m_mux);
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
            ESP_LOGI(TAG, "clock set from the GPS (%s): CLOCK_REALTIME now agrees with what "
                          "the responder serves",
                     isoTimestamp(gps.tv_sec).c_str());
        } else {
            ESP_LOGW(TAG, "clock stepped %+lld ms: further than a slew can fix quickly",
                     static_cast<long long>(offsetNs / 1000000));
        }
        m_stepped = true;
        return;
    }

    /* Under a quarter of a second: slew, so a client polling across this moment
     * never sees the clock jump. IDF's adjtime() applies about 1.5% of elapsed
     * time per call, i.e. this clears in seconds rather than minutes. */
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

GpsTime::Status GpsTime::status() const
{
    Status   st;
    int64_t  ppsMonoUs     = 0;
    int64_t  suppressUntil = 0;

    portENTER_CRITICAL(&m_mux);
    st.ppsCount    = m_ppsCount;
    st.timeValid   = m_timeValid;
    st.fixType     = m_fixType;
    st.satellites  = m_satellites;
    st.anchored    = m_anchorValid;
    st.lastFix     = m_lastFix;
    st.offsetUs    = m_offsetUs;
    st.ratePpb     = m_ratePpb;
    st.rateSamples = m_rateSamples;
    st.leapSeconds = m_leapSeconds;
    st.leapChange  = m_leapChange;
    st.leapInS     = m_leapInS;
    st.leapValid   = m_leapValid;
    st.lastHoldoverUs    = m_lastHoldoverUs;
    st.lastHoldoverErrNs = m_lastHoldoverErrNs;
    ppsMonoUs      = m_ppsMonoUs;
    suppressUntil  = m_ppsSuppressUntilUs;
    portEXIT_CRITICAL(&m_mux);

    const int64_t nowUs = esp_timer_get_time();
    if (st.ppsCount != 0) {
        st.ppsAgeUs = nowUs - ppsMonoUs;
    }
    if (suppressUntil > nowUs) {
        st.ppsSuppressUs = suppressUntil - nowUs;
    }

    st.rateKnown = st.rateSamples >= kRateMinSamples;

    /* Two windows, and the difference between them is the servo. Without a rate
     * there is nothing to hold over with, so three missed edges demote; with one
     * the crystal's known error is taken out of every timestamp and ten minutes
     * of holdover is worth serving, with the claim decaying as it ages. */
    st.lockWindowUs = (st.rateKnown && st.timeValid && st.fixType >= 2) ? kHoldoverWindowUs
                                                                       : kLockWindowUs;

    /* Lock needs all three: a fix to trust the date, a valid NAV-PVT to know
     * which second, and an edge recent enough that the clock is still being
     * corrected. Fix type 2 is the floor -- "time-only" (5) dates the second but
     * does not put the receiver anywhere, and this reference does not want to
     * explain that difference in its stratum. */
    st.locked = st.anchored && st.timeValid && st.fixType >= 2 && st.ppsAgeUs >= 0 &&
                st.ppsAgeUs < st.lockWindowUs;

    if (st.locked) {
        /* What this reference states as its own dispersion: ten microseconds
         * while the anchor is fresh, then one ppm of age. Not the fit's
         * confidence -- that is parts per billion, and the crystal will wander
         * further than that before the fit notices. */
        const int64_t ageS = st.ppsAgeUs > 0 ? st.ppsAgeUs / 1000000 : 0;
        st.dispersionNs    = kBaseDispersionNs + ageS * kWanderNsPerS;
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

uint32_t GpsTime::suppressPps(uint32_t seconds)
{
    const int64_t until =
        seconds == 0 ? 0 : esp_timer_get_time() + static_cast<int64_t>(seconds) * 1000000;

    portENTER_CRITICAL(&m_mux);
    m_ppsSuppressUntilUs = until;
    portEXIT_CRITICAL(&m_mux);

    if (seconds == 0) {
        ESP_LOGW(TAG, "PPS edges counted again: the holdover test is over");
        return 0;
    }

    const Status st = status();
    ESP_LOGW(TAG, "PPS edges ignored for %u s at the request of POST /gps/pps: the holdover from "
                  "here is deliberate, and %s",
             static_cast<unsigned>(seconds),
             st.rateKnown ? "the servo is carrying it" : "with no rate yet it demotes in 3 s");
    return seconds;
}

bool GpsTime::now(struct timespec &ts) const
{
    int64_t utcNs   = 0;
    int64_t monoUs  = 0;
    int32_t ratePpb = 0;

    portENTER_CRITICAL(&m_mux);
    const bool valid = m_anchorValid;
    utcNs            = m_anchorUtcNs;
    monoUs           = m_anchorMonoUs;
    ratePpb          = m_ratePpb;
    portEXIT_CRITICAL(&m_mux);

    if (!valid) {
        return false;
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

std::string GpsTime::describe() const
{
    const Status st = status();
    char         text[96];

    if (st.locked) {
        if (st.ppsAgeUs > kLockWindowUs) {
            /* Still servable, and worth saying why in a line somebody reads
             * later: this is the holdover the servo bought. */
            std::snprintf(text, sizeof(text), "GPS/PPS, %s fix, %u satellites, holding over %.1f s",
                          fixText(st.fixType), st.satellites,
                          static_cast<double>(st.ppsAgeUs) / 1e6);
        } else {
            std::snprintf(text, sizeof(text), "GPS/PPS, %s fix, %u satellites", fixText(st.fixType),
                          st.satellites);
        }
    } else if (st.fixType >= 2 && st.timeValid) {
        /* A fix but no edge we can pair it with: the receiver is fine and the
         * pulse is not, which is a different fault from "no fix" and worth
         * saying differently in the boot log. */
        std::snprintf(text, sizeof(text), "GPS/PPS, %s fix but no fresh PPS edge",
                      fixText(st.fixType));
    } else {
        std::snprintf(text, sizeof(text), "GPS/PPS (no fix yet)");
    }
    return text;
}

void GpsTime::logLockChange()
{
    const Status st = status();

    if (st.locked != m_wasLocked) {
        m_wasLocked = st.locked;
        if (st.locked) {
            ESP_LOGI(TAG, "locked: stratum 1 from here on (PPS edges so far: %u, rate %+ld ppb)",
                     static_cast<unsigned>(st.ppsCount), static_cast<long>(st.ratePpb));
        } else if (st.ppsCount == 0) {
            ESP_LOGW(TAG, "no PPS edges: is the module's timepulse wired to GPIO%d?",
                     static_cast<int>(kPpsPin));
        } else {
            ESP_LOGW(TAG, "not locked (fix %s, last edge %lld ms ago): stratum 16 until it returns",
                     fixText(st.fixType), static_cast<long long>(st.ppsAgeUs / 1000));
        }
    }

    /* Inside a holdover nothing above changes for ten minutes, which would
     * otherwise look like a healthy clock that has quietly lost its PPS. Say it
     * when the edge goes stale, and say it again when it comes back. */
    const bool holding = st.locked && st.ppsAgeUs > kLockWindowUs;
    if (holding != m_holdoverLogged) {
        m_holdoverLogged = holding;
        if (holding) {
            ESP_LOGW(TAG, "holding over: no PPS for %.1f s, serving on the servo's rate (%+ld ppb) "
                          "with dispersion %lld us",
                     static_cast<double>(st.ppsAgeUs) / 1e6, static_cast<long>(st.ratePpb),
                     static_cast<long long>(st.dispersionNs / 1000));
        } else if (st.locked) {
            ESP_LOGI(TAG, "PPS back: the anchor is fresh again");
        }
    }
}

namespace {

constexpr uint8_t kStratumGps = 1;

/* The GPS as the responder sees it. Stateless -- everything it answers comes
 * from the GpsTime singleton, which is older than any responder task. */
class GpsTimeRef : public TimeRef {
public:
    TimeReference reference() const override
    {
        GpsTime             &gps = GpsTime::instance();
        const GpsTime::Status st = gps.status();

        TimeReference ref;
        ref.lastSync = st.lastFix;
        ref.locked   = st.locked;
        ref.stratum  = st.locked ? kStratumGps : 16;
        /* "GPS" while it is a stratum-1 reference, "INIT" once it is not -- for
         * the same reason the upstream ref does it: a client reading stratum 16
         * needs the refid to look like a refusal, not like a reference. */
        ref.refId = st.locked ? kRefIdGps : kRefIdInit;

        /* The receiver's own leap notice, straight into the packet: only it
         * knows whether the end of the UTC day has 59, 60 or 61 seconds. */
        ref.leap = gps.leapIndicator();

        /* And its own error budget. Root delay is zero because there is no path
         * to measure -- the reference is wired, not networked -- and dispersion
         * is the ten microseconds plus wander that status() works out. While it
         * is not locked there is no budget to state, so both stay zero and the
         * responder falls back to its own ageing model. */
        if (st.dispersionNs > 0) {
            ref.rootDelayNs  = 0;
            ref.dispersionNs = static_cast<uint32_t>(st.dispersionNs);
        }
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
