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

/* UBX-CFG-MSG enabling NAV-TIM-TP on UART1 at 1 Hz. File scope because the
 * probe in task() resends exactly these bytes on their own. */
constexpr uint8_t kTimTpEnable[8] = {0x01, 0x27, 0, 1, 0, 0, 0, 0};

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

/* The part of the anchor's accuracy that is named rather than measured: entry
 * through the IRAM interrupt, plus esp_timer's own microsecond resolution. Two
 * microseconds is an order of magnitude above the agreement actually observed,
 * because root delay is a promise and not a boast.
 *
 * The *mean* of the interrupt latency is the one number a receiver's own pulse
 * cannot tell us, and it is worth being clear about why. The pulse defines the
 * epoch here: an anchor built from it moves with it, so a constant lateness
 * cancels out of every later comparison and out of every timestamp. What can be
 * measured is the spread around that mean -- which is the half that varies and
 * the half that actually hurts -- and the receiver's own statement about how
 * well it knows where the pulse is. Both are in the budget below; the mean
 * stays a named constant rather than a fiction of zero. */
constexpr int64_t kIsrLatencyNs = 2000;

/* Bounds on what TIM-TP may claim, so a nonsense frame cannot turn into a
 * 10 ms root delay in an answer to a client: a receiver that knows its pulse
 * worse than 5 us is not doing this job, and a spread wider than 20 us is a load
 * spike rather than jitter. */
constexpr int64_t kMaxQErrNs     = 5000;
constexpr int64_t kMaxJitterNs   = 20000;

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

void GpsTime::requestAntennaBias(bool on)
{
    m_biasReq = on ? 1 : 0;
}

void GpsTime::requestReset(bool cold, bool hardware)
{
    m_resetCold = cold;
    m_resetHard = hardware;
    m_resetReq  = true;
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

    /* A lone TIM-TP enable three seconds in, if none has produced a frame. Ask
     * it on its own, with nothing else outstanding, so the ACK or NAK that comes
     * back can only be about this one message -- in the boot burst the replies
     * to eight identical CFG-MSG frames are indistinguishable. Doubles as the
     * retry for a module that was still booting when the burst went out. */
    m_timTpProbeAtUs = esp_timer_get_time() + 3000000;
    m_nextHwUs       = esp_timer_get_time() + 2000000;
    m_warmRetryAtUs  = esp_timer_get_time() + 10000000;

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
        /* A bias change or a restart asked for over HTTP. Serviced here rather
         * than in the handler so exactly one thread owns the UART and the
         * parser. */
        if (m_biasReq >= 0) {
            const bool on = m_biasReq == 1;
            m_biasReq     = -1;
            /* Supply control bit on or off; short/open detection and
             * power-down-on-short stay set either way. */
            const uint8_t cfg[4] = {static_cast<uint8_t>(on ? 0x0F : 0x0E), 0, 0, 0};
            const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
            ESP_LOGW(TAG, "antenna bias %s on request: module %s", on ? "ON" : "OFF",
                     br == 1 ? "accepted" : br == 0 ? "refused, so the bias is unchanged"
                                                    : "did not answer");
            sendUbx(0x0A, 0x09, nullptr, 0); /* re-read the supervisor at once */
        }
        /* Three more tries at the two messages a module still booting may have
         * refused, ten seconds apart. The boot burst goes out about half a
         * second after power-up, which on this receiver is before it is ready
         * for everything -- MON-VER only answered at 1.2 s -- so a NAK that
         * early is not always a refusal. Only the refusals are retried; a
         * message that was acknowledged is left alone. */
        if (m_warmRetries < 3 && nowUs >= m_warmRetryAtUs &&
            (m_antCfgNaked || m_timTpNaked)) {
            ++m_warmRetries;
            m_warmRetryAtUs = nowUs + 10000000;
            if (m_antCfgNaked) {
                const uint8_t cfg[4] = {0x0F, 0x00, 0x00, 0x00};
                const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
                if (br >= 0) {
                    m_antCfgNaked = br == 0;
                    m_antCfgAcked = br == 1;
                    ESP_LOGW(TAG, "CFG-ANT retry %u: %s", m_warmRetries,
                             br == 1 ? "accepted now the receiver is up" : "still refused");
                }
            }
            if (m_timTpNaked && m_timTpCount == 0) {
                const int8_t br = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
                if (br >= 0) {
                    m_timTpNaked = br == 0;
                    m_timTpAcked = br == 1;
                    ESP_LOGW(TAG, "TIM-TP retry %u: %s", m_warmRetries,
                             br == 1 ? "accepted now the receiver is up" : "still refused");
                }
            }
        }

        if (m_reconfigAtUs != 0 && nowUs >= m_reconfigAtUs) {
            m_reconfigAtUs = 0;
            configureModule();
        }
        if (m_resetReq) {
            m_resetReq = false;
            /* CFG-RST. Hot by default: restart the navigation engine and keep
             * the almanac, which is the cheap way back from a state a module will
             * not leave on its own. Cold when asked, which throws the almanac
             * away -- the thing to try when a hot start changed nothing and the
             * receiver's own stored state is suspect. */
            const bool    cold = m_resetCold;
            const bool    hard = m_resetHard;
            m_resetHard        = false;
            /* resetMode: 0x00 is a hardware reset, 0x02 a controlled software
             * one (GNSS only). A hardware reset also drops the module's whole
             * runtime configuration back to its defaults, so the message set is
             * re-sent a few seconds later -- otherwise it comes back speaking
             * NMEA at 9600 and the parser has nothing to read. */
            const uint8_t rst[4] = {static_cast<uint8_t>(cold ? 0xFF : 0x00),
                                    static_cast<uint8_t>(cold ? 0xFF : 0x00),
                                    static_cast<uint8_t>(hard ? 0x00 : 0x02), 0x00};
            ESP_LOGW(TAG, "receiver %s%s start requested", hard ? "HARDWARE " : "",
                     cold ? "cold" : "hot");
            if (hard) {
                m_reconfigAtUs = esp_timer_get_time() + 3000000;
            }
            sendUbx(0x06, 0x04, rst, sizeof(rst));
        }

        /* Antenna supervisor, every two seconds. Polled rather than enabled at
         * a rate so a bench session that never looks at it costs nothing. */
        if (nowUs >= m_nextHwUs) {
            sendUbx(0x0A, 0x09, nullptr, 0);
            m_nextHwUs = nowUs + 2000000;
        }
        if (!m_timTpProbed && m_timTpCount == 0 && nowUs >= m_timTpProbeAtUs) {
            m_timTpProbed = true;
            const int8_t pr = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
            if (pr == 0) {
                m_timTpNaked = true;
                m_timTpAcked = false;
            } else if (pr == 1) {
                m_timTpAcked = true;
            }
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
     * only source for the leap indicator a reply puts in its first byte.
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
        {0xF0, 0x08}, /* ZDA */
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
     * asserts the shape rather than changing it. antCableDelay and
     * userConfigDelay stay zero: the antenna lead's delay is not known, and
     * pretending a figure would be worse than leaving it. */
    uint8_t tp5[32] = {};
    tp5[1]          = 1; /* version */
    /* freqPeriod and freqPeriodLock, in microseconds: isFreq (bit 3) is clear. */
    constexpr uint32_t kOneHzUs = 1000000;
    constexpr uint32_t kRatio10 = 429496730; /* 0.1 of the period in 2^-32 */
    std::memcpy(tp5 + 8, &kOneHzUs, 4);
    std::memcpy(tp5 + 12, &kOneHzUs, 4);
    std::memcpy(tp5 + 16, &kRatio10, 4);
    std::memcpy(tp5 + 20, &kRatio10, 4);
    /* active, lockGnssFreq, lockedOtherSet, alignToTow, rising, gridUtcGps. */
    constexpr uint32_t kTp5Flags = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 5) | (1u << 6) |
                                   (1u << 7);
    std::memcpy(tp5 + 28, &kTp5Flags, 4);

    /* Each result is acted on here rather than by whoever reads the log later:
     * an ACK means the pulse shape is asserted, a NAK means it is not, and a
     * silence means the module does not know the message -- which for CFG-TP5
     * would be a very old receiver indeed. */
    int8_t r = sendAndWait(0x06, 0x31, tp5, sizeof(tp5));
    m_tp5Acked = r == 1;
    m_tp5Naked = r == 0;
    if (r == 1) {
        ESP_LOGI(TAG, "CFG-TP5 accepted: pulse is 1 Hz, UTC, rising");
    } else {
        ESP_LOGW(TAG, "CFG-TP5 %s: the pulse is NOT asserted",
                 r == 0 ? "rejected" : "unanswered");
    }

    /* UBX-CFG-ANT (class 0x06, id 0x13). The bias tee on the RF input is not
     * something this firmware used to touch, so whether a preamp was powered
     * depended entirely on whatever the module's saved configuration said -- a
     * user swapping antennas had no way to know from here. It is now asserted:
     * supply voltage control on, short and open detection on, and power down on
     * a short so a bad lead cannot cook the supply.
     *
     * The detect bits are the point of the exercise: MON-HW reports what they
     * find as aStatus, so "is the antenna plugged in and powered" becomes a
     * number in /healthz instead of a guess. A receiver without a supervisor
     * NAKs this and the whole group stays at DONTKNOW. */
    constexpr uint8_t kCfgAnt[4] = {0x0F, 0x00, 0x00, 0x00};
    r = sendAndWait(0x06, 0x13, kCfgAnt, sizeof(kCfgAnt));
    m_antCfgAcked = r == 1;
    m_antCfgNaked = r == 0;
    if (r == 1) {
        ESP_LOGI(TAG, "CFG-ANT accepted: antenna bias on, with short/open detection");
    } else {
        ESP_LOGW(TAG, "CFG-ANT %s: antenna bias is whatever the module was left set to",
                 r == 0 ? "rejected" : "unanswered");
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
    r = sendAndWait(0x06, 0x01, kTimTpEnable, sizeof(kTimTpEnable));
    m_timTpAcked = r == 1;
    m_timTpNaked = r == 0;
    if (r == 0) {
        ESP_LOGW(TAG, "NAV-TIM-TP enable rejected: no TIM-TP on this firmware");
    } else if (r == 1) {
        ESP_LOGI(TAG, "NAV-TIM-TP enable accepted");
    }

    /* UBX-NAV-SAT at half rate. Twelve bytes per satellite with a 16-satellite
     * sky is 200 bytes, so once a second would spend a fifth of a 9600-baud line
     * on diagnostics; once every two seconds is plenty to compare two antennas
     * and leaves the line to the messages the clock actually needs. */
    {
        constexpr uint8_t kNavSat[8] = {0x01, 0x35, 0, 2, 0, 0, 0, 0};
        r = sendAndWait(0x06, 0x01, kNavSat, sizeof(kNavSat));
        if (r != 1) {
            ESP_LOGW(TAG, "NAV-SAT enable %s: no C/N0 in /healthz",
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
        const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < got; ++i) {
            feed(buf[i]);
        }
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

    /* The first dozen frames by class and id. Short lines, and only at
     * boot: when a message is enabled, acknowledged and then never arrives,
     * knowing what the receiver does send is the whole diagnosis. */
    if (m_framesLogged < 12) {
        ++m_framesLogged;
        ESP_LOGI(TAG, "frame %02x/%02x len %u", cls, id, static_cast<unsigned>(len));
    }

    if (cls == 0x05 && (id == 0x00 || id == 0x01)) { /* UBX-ACK-ACK / UBX-ACK-NAK */
        /* Only the message actually outstanding is recorded. Anything else is a
         * late reply to something already timed out, and guessing which would
         * put the answer on the wrong message. */
        if (len >= 2 && payload[0] == m_awaitCls && payload[1] == m_awaitId &&
            m_awaitResult == -1) {
            m_awaitResult = (id == 0x01) ? 1 : 0;
        }
        if (!m_ackLogged && id == 0x01) {
            m_ackLogged = true;
            ESP_LOGI(TAG, "module accepted the 1 Hz message set");
        }
    } else if (cls == 0x01 && id == 0x07) { /* UBX-NAV-PVT */
        onNavPvt(payload, len);
    } else if (cls == 0x01 && id == 0x26) { /* UBX-NAV-TIMELS */
        onNavTimels(payload, len);
    } else if (cls == 0x01 && id == 0x27) { /* UBX-NAV-TIM-TP */
        onNavTimTp(payload, len);
    } else if (cls == 0x0A && id == 0x04) { /* UBX-MON-VER */
        onMonVer(payload, len);
    } else if (cls == 0x0A && id == 0x09) { /* UBX-MON-HW */
        onMonHw(payload, len);
    } else if (cls == 0x01 && id == 0x35) { /* UBX-NAV-SAT */
        onNavSat(payload, len);
    }
}

void GpsTime::onMonVer(const uint8_t *p, uint16_t len)
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
    char text[160] = {};
    uint16_t n     = 0;
    for (uint16_t i = 0; i < len && n < sizeof(text) - 1; ++i) {
        const char c = static_cast<char>(p[i]);
        text[n++]    = c == '\0' ? ' ' : (c >= 0x20 && c < 0x7F ? c : '.');
    }

    portENTER_CRITICAL(&m_mux);
    std::memcpy(m_moduleId, text, sizeof(m_moduleId));
    portEXIT_CRITICAL(&m_mux);

    ESP_LOGI(TAG, "module: %.48s", text);
}

void GpsTime::onMonHw(const uint8_t *p, uint16_t len)
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
    if (len < 60) {
        return;
    }

    const uint16_t noise  = le16(p + 16);
    const uint16_t agc    = le16(p + 18);
    const uint8_t  status = p[20];
    const uint8_t  power  = p[21];
    const uint8_t  jstate = (p[22] >> 2) & 0x03;
    const uint8_t  jam    = p[45];

    uint8_t before = 0;
    uint8_t beforePower = 0;
    uint8_t beforeJam = 0;

    portENTER_CRITICAL(&m_mux);
    before      = m_antStatus;
    beforePower = m_antPower;
    beforeJam   = m_jamState;
    m_antStatus = status;
    m_antPower  = power;
    m_noisePerMs = noise;
    m_agcCnt     = agc;
    m_jamInd     = jam;
    m_jamState   = jstate;
    m_monHwValid = true;
    m_monHwAtUs  = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);

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
        case 2:
            ESP_LOGI(TAG, "antenna status OK");
            break;
        default:
            ESP_LOGI(TAG, "antenna status %u (no supervisor reading)", status);
            break;
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

void GpsTime::onNavSat(const uint8_t *p, uint16_t len)
{
    /* UBX-NAV-SAT: 8 bytes of header (iTOW, version, numSvs, reserved) then 12
     * bytes per satellite. Only cno and the svUsed flag are wanted, so the
     * groups are walked rather than the message being kept.
     *
     * C/N0 is the number to compare antennas with. Satellite count is not: a
     * better antenna adds the marginal ones last, and a worse one can hold the
     * count up on the strongest handful while everything near the horizon goes.
     * Mean and max over *used* satellites therefore, not tracked ones. */
    if (len < 8) {
        return;
    }
    const uint8_t num = p[5];
    if (len < static_cast<uint16_t>(8 + 12 * num)) {
        return;
    }

    uint16_t sum = 0;
    uint8_t  max = 0;
    uint8_t  used = 0;
    for (uint8_t i = 0; i < num; ++i) {
        const uint8_t *sv = p + 8 + 12 * i;
        const uint8_t  cno = sv[2];
        const uint32_t fl  = static_cast<uint32_t>(sv[8]) |
                            (static_cast<uint32_t>(sv[9]) << 8) |
                            (static_cast<uint32_t>(sv[10]) << 16) |
                            (static_cast<uint32_t>(sv[11]) << 24);
        if ((fl & (1u << 3)) == 0 || cno == 0) {
            continue;
        }
        sum = static_cast<uint16_t>(sum + cno);
        max = cno > max ? cno : max;
        ++used;
    }

    portENTER_CRITICAL(&m_mux);
    m_cn0Max   = max;
    m_cn0Mean  = used != 0 ? static_cast<uint8_t>(sum / used) : 0;
    m_cn0Used  = used;
    m_navSatAtUs = esp_timer_get_time();
    portEXIT_CRITICAL(&m_mux);
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

void GpsTime::onNavTimTp(const uint8_t *p, uint16_t len)
{
    /* NAV-TIM-TP is 16 bytes, from the u-blox 8 interface description:
     *    0 towMS (u4, ms of week)   4 towSubMS (u4, 2^-32 of a ms)
     *    8 qErr (i4, picoseconds)
     *   12 week (u2)   14 flags (X1)   15 refInfo (u1)
     *
     * The flags byte, bit 0: 0 = UTC time base, 1 = GPS. Bit 3 marks qErr
     * invalid. The rest (utc, raim, refInfo) say nothing used here.
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
    if (len < 16) {
        return;
    }

    const int32_t  towMs     = le32(p);
    const uint32_t towSubMs  = static_cast<uint32_t>(le32(p + 4));
    const int32_t  qErrPs    = le32(p + 8);
    const uint8_t  flags     = p[14];

    const bool utcBase     = (flags & 0x01) == 0;
    const bool qErrInvalid = (flags & 0x08) != 0;

    /* Where in the second the pulse sits, in nanoseconds: a UTC-aligned 1 Hz
     * pulse is a hair either side of zero. A reported position just under a
     * second is the same place seen from the other side, so it is made
     * negative rather than being read as 999 ms late. */
    int32_t msIntoSecond = towMs % 1000;
    if (msIntoSecond < 0) {
        msIntoSecond += 1000;
    }
    int64_t offsetNs = static_cast<int64_t>(msIntoSecond) * 1000000 +
                       (static_cast<int64_t>(towSubMs) * 1000000 >> 32);
    if (msIntoSecond == 999) {
        offsetNs -= 1000000000;
    }
    const bool onSecond = utcBase && offsetNs > -2000000 && offsetNs < 2000000;

    int32_t qErrNs = qErrPs / 1000;
    if (qErrNs < 0) {
        qErrNs = -qErrNs;
    }
    if (qErrNs > kMaxQErrNs) {
        qErrNs = kMaxQErrNs;
    }

    portENTER_CRITICAL(&m_mux);
    ++m_timTpCount;
    m_qErrNs        = qErrNs;
    m_qErrValid     = !qErrInvalid;
    m_pulseOnSecond = onSecond;
    portEXIT_CRITICAL(&m_mux);

    /* Once, so the log says which regime this board is in: a measured
     * quantisation, or none. */
    if (!m_timTpLogged) {
        m_timTpLogged = true;
        if (qErrInvalid) {
            ESP_LOGW(TAG, "NAV-TIM-TP flowing but qErr is flagged invalid: root delay stays "
                          "the named %lld us", static_cast<long long>(kIsrLatencyNs / 1000));
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
            ESP_LOGW(TAG, "timepulse is %.3f ms off the UTC second (%s base): every anchor is "
                          "shifted with it", static_cast<double>(offsetNs) / 1e6,
                     utcBase ? "UTC" : "GPS");
        }
    } else if (!m_pulseOnSecondSeen) {
        ESP_LOGI(TAG, "timepulse back on the UTC second");
    }
    m_pulseOnSecondSeen = onSecond;
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

    /* Residual: the thing that says the samples are actually samples of a
     * clock and not of the receiver's mood -- and, while the servo is running,
     * the bound this reference states. Each edge's distance from the line
     * through all of them is the stamping jitter, measured, and RMS is the
     * figure a root dispersion wants. TIM-TP would describe the same jitter
     * directly, and this receiver refuses TIM-TP, so the residuals carry it. */
    double sse = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoUs - base.monoUs) - meanX;
        const double dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY -
                          slope * dx;
        sse += dy * dy;
    }
    const double residualNs = std::sqrt(sse / m_rateCount);
    {
        const int64_t jitterNs = residualNs > static_cast<double>(kMaxJitterNs)
                                     ? kMaxJitterNs
                                     : static_cast<int64_t>(residualNs);
        portENTER_CRITICAL(&m_mux);
        m_jitterNs = jitterNs;
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
    int64_t  navSatAtUs    = 0;
    int64_t  monHwAtUs     = 0;

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
    st.timTpCount        = m_timTpCount;
    st.qErrNs            = m_qErrNs;
    st.qErrValid         = m_qErrValid;
    st.jitterNs          = m_jitterNs;
    st.pulseOnSecond     = m_pulseOnSecond;
    st.tp5Rejected       = m_tp5Naked;
    st.timTpRejected     = m_timTpNaked;
    st.timTpAcked        = m_timTpAcked;
    st.antCfgAcked       = m_antCfgAcked;
    st.antCfgRejected    = m_antCfgNaked;
    st.antStatus         = m_antStatus;
    st.antPower          = m_antPower;
    st.noisePerMs        = m_noisePerMs;
    st.agcCnt            = m_agcCnt;
    st.jamInd            = m_jamInd;
    st.jamState          = m_jamState;
    st.monHwValid        = m_monHwValid;
    st.cn0Max            = m_cn0Max;
    st.cn0Mean           = m_cn0Mean;
    st.cn0Used           = m_cn0Used;
    navSatAtUs           = m_navSatAtUs;
    monHwAtUs            = m_monHwAtUs;
    st.moduleId          = m_moduleId;
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
    if (navSatAtUs != 0) {
        st.navSatAgeUs = nowUs - navSatAtUs;
    }
    if (monHwAtUs != 0) {
        st.monHwAgeUs = nowUs - monHwAtUs;
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
        const int64_t ageS = st.ppsAgeUs > 0 ? st.ppsAgeUs / 1000000 : 0;
        st.rootDelayNs  = kIsrLatencyNs + (st.qErrValid ? st.qErrNs / 2 : 0);
        st.dispersionNs = st.rootDelayNs + st.jitterNs + ageS * kWanderNsPerS;
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

    /* A module that was narrating its pulse and has stopped. Not the same fault
     * as one that never did, and nothing on the wire shows it: the quantisation
     * term silently disappears from the budget. */
    const int64_t nowUs = esp_timer_get_time();
    if (st.timTpCount != m_timTpLastCount) {
        m_timTpLastCount = st.timTpCount;
        m_timTpLastUs    = nowUs;
        m_timTpGoneLogged = false;
    } else if (st.timTpCount != 0 && !m_timTpGoneLogged &&
               nowUs - m_timTpLastUs > 10 * 1000000) {
        m_timTpGoneLogged = true;
        /* The last qErr stays in the status for what it is worth, but stops
         * being counted: a figure from a message that has stopped arriving is a
         * stale claim. The delay drops back to the named latency; the jitter
         * does not, because it is measured from the anchors themselves and the
         * anchors are still coming. */
        portENTER_CRITICAL(&m_mux);
        m_qErrValid = false;
        portEXIT_CRITICAL(&m_mux);
        ESP_LOGW(TAG, "NAV-TIM-TP stopped 10 s ago: the pulse's quantisation is no longer stated, "
                      "root delay falls back to the named %lld us",
                 static_cast<long long>(kIsrLatencyNs / 1000));
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

        /* And its own error budget, both halves of it. Root delay used to be
         * zero here on the grounds that the reference is wired rather than
         * networked -- an assumption, and the one NAV-TIM-TP exists to replace.
         * status() now fills it from the interrupt's named latency and half the
         * receiver's stated pulse quantisation, and the dispersion from that
         * plus the jitter actually observed. While it is not locked there is no
         * budget to state, so both stay zero and the responder falls back to its
         * own ageing model. */
        if (st.dispersionNs > 0) {
            ref.rootDelayNs  = static_cast<uint32_t>(st.rootDelayNs);
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
