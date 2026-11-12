#include "gps_time.h"

#include <cerrno>
#include <cstdio>
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
 * ten would hold a stopped module up for ten seconds. */
constexpr int64_t kLockWindowUs = 3000000;

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
    if (self->m_ppsCount == 0 || nowUs - self->m_ppsMonoUs >= kMinEdgeSpacingUs) {
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
     * what is read here and nothing else. */
    static constexpr uint8_t kNavPvt[8] = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSilence[][2] = {
        {0xF0, 0x00}, /* GGA */ {0xF0, 0x01}, /* GLL */ {0xF0, 0x02}, /* GSA */
        {0xF0, 0x03}, /* GSV */ {0xF0, 0x04}, /* RMC */ {0xF0, 0x05}, /* VTG */
        {0xF0, 0x08}, /* ZDA */
    };

    sendUbx(0x06, 0x01, kNavPvt, sizeof(kNavPvt));

    for (const auto &msg : kSilence) {
        const uint8_t off[8] = {msg[0], msg[1], 0, 0, 0, 0, 0, 0};
        sendUbx(0x06, 0x01, off, sizeof(off));
    }

    /* Deliberately not followed by UBX-CFG-CFG: saving this to the receiver's
     * flash would outlive the firmware and take away the module's own defaults,
     * which is a rude thing to do to a receiver someone may want to read with
     * u-center later. Sending it at every boot costs eight short frames. */
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
            ESP_LOGI(TAG, "module accepted the configuration (NAV-PVT at 1 Hz, NMEA off)");
        }
    } else if (cls == 0x05 && id == 0x00) { /* UBX-ACK-NAK */
        if (!m_nakLogged) {
            m_nakLogged = true;
            ESP_LOGW(TAG, "module rejected a configuration message (class 0x%02x id 0x%02x)",
                     len >= 2 ? payload[0] : 0, len >= 2 ? payload[1] : 0);
        }
    } else if (cls == 0x01 && id == 0x07) { /* UBX-NAV-PVT */
        onNavPvt(payload, len);
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

    portENTER_CRITICAL(&m_mux);
    m_anchorUtcNs  = static_cast<int64_t>(second) * 1000000000;
    m_anchorMonoUs = ppsMonoUs;
    m_anchorValid  = true;
    m_lastFix      = second;
    portEXIT_CRITICAL(&m_mux);

    steer();
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
    Status  st;
    int64_t ppsMonoUs = 0;

    portENTER_CRITICAL(&m_mux);
    st.ppsCount   = m_ppsCount;
    st.timeValid  = m_timeValid;
    st.fixType    = m_fixType;
    st.satellites = m_satellites;
    st.anchored   = m_anchorValid;
    st.lastFix    = m_lastFix;
    st.offsetUs   = m_offsetUs;
    ppsMonoUs     = m_ppsMonoUs;
    portEXIT_CRITICAL(&m_mux);

    if (st.ppsCount != 0) {
        st.ppsAgeUs = esp_timer_get_time() - ppsMonoUs;
    }

    /* Lock needs all three: a fix to trust the date, a valid NAV-PVT to know
     * which second, and an edge recent enough that the clock is still being
     * corrected. Fix type 2 is the floor -- "time-only" (5) dates the second but
     * does not put the receiver anywhere, and this reference does not want to
     * explain that difference in its stratum. */
    st.locked = st.anchored && st.timeValid && st.fixType >= 2 && st.ppsAgeUs >= 0 &&
                st.ppsAgeUs < kLockWindowUs;
    return st;
}

bool GpsTime::now(struct timespec &ts) const
{
    int64_t utcNs  = 0;
    int64_t monoUs = 0;

    portENTER_CRITICAL(&m_mux);
    const bool valid = m_anchorValid;
    utcNs            = m_anchorUtcNs;
    monoUs           = m_anchorMonoUs;
    portEXIT_CRITICAL(&m_mux);

    if (!valid) {
        return false;
    }

    /* The anchor plus elapsed monotonic time. Reading the counter (1 us) rather
     * than CLOCK_REALTIME also means a slew in flight cannot make a timestamp
     * go backwards between two replies. */
    const int64_t totalNs = utcNs + (esp_timer_get_time() - monoUs) * 1000;
    ts.tv_sec             = static_cast<time_t>(totalNs / 1000000000);
    ts.tv_nsec            = static_cast<long>(totalNs % 1000000000);
    return true;
}

std::string GpsTime::describe() const
{
    const Status st = status();
    char         text[64];

    if (st.locked) {
        std::snprintf(text, sizeof(text), "GPS/PPS, %s fix, %u satellites", fixText(st.fixType),
                      st.satellites);
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
    if (st.locked == m_wasLocked) {
        return;
    }
    m_wasLocked = st.locked;

    if (st.locked) {
        ESP_LOGI(TAG, "locked: stratum 1 from here on (PPS edges so far: %u)",
                 static_cast<unsigned>(st.ppsCount));
    } else if (st.ppsCount == 0) {
        ESP_LOGW(TAG, "no PPS edges: is the module's timepulse wired to GPIO%d?", static_cast<int>(kPpsPin));
    } else {
        ESP_LOGW(TAG, "not locked (fix %s, last edge %lld ms ago): stratum 16 until it returns",
                 fixText(st.fixType), static_cast<long long>(st.ppsAgeUs / 1000));
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
        const GpsTime::Status st = GpsTime::instance().status();

        TimeReference ref;
        ref.lastSync = st.lastFix;
        ref.locked   = st.locked;
        ref.stratum  = st.locked ? kStratumGps : 16;
        /* "GPS" while it is a stratum-1 reference, "INIT" once it is not -- for
         * the same reason the upstream ref does it: a client reading stratum 16
         * needs the refid to look like a refusal, not like a reference. */
        ref.refId = st.locked ? kRefIdGps : kRefIdInit;
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
