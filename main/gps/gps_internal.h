/* Constants and helpers shared by the GpsTime implementation files. */
#pragma once

#include "ptp/ptp.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_private/systimer.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "hal/mcpwm_ll.h"
#include "clock/clock_source.h"
#include "ntp/serve_path.h"
#include "clock/ntp_time.h"
#include "app/cores.h"
#include "app/settings.h"

#include "gps/gps_time.h"

namespace gps_detail {

/* ---- Wiring ------------------------------------------------------------ */

/* UART0 is the console (sdkconfig.defaults). */
constexpr uart_port_t kUart = UART_NUM_1;

constexpr int kTxPin = 15; /* esp TX -> module RX */

constexpr int kRxPin = 16; /* esp RX <- module TX */

constexpr gpio_num_t kPpsPin = GPIO_NUM_14;

/* Scan order. 38400 is the rate the module is saved at, so it goes first; 9600
 * is the factory default. */
constexpr uint32_t kBauds[] = {38400, 9600, 115200, 57600, 19200, 4800};

/* UBX-CFG-MSG: TIM-TP on UART1 at 1 Hz. The probe in task() resends these bytes.
 * Class 0x0D: the LEA-M8T timing firmware refuses the NAV-class form. */
constexpr uint8_t kTimTpEnable[8] = {0x0D, 0x01, 0, 1, 0, 0, 0, 0};

/* UBX-CFG-MSG: TIM-TOS. The interface description lists it for Time & Frequency
 * Sync products only, so it may be refused; when it is not, it reports the pulse's
 * offset from UTC and whether the pulse is locked. */
constexpr uint8_t kTimTosEnable[8] = {0x0D, 0x12, 0, 1, 0, 0, 0, 0};

/* How long one baud rate gets to produce a single valid UBX frame. */
constexpr int64_t kScanAfterUs = 4000000;

/* The first rate gets longer: 4 s could expire while the module was still
 * booting, and the scan then moved off the right rate. */
constexpr int64_t kFirstRateUs = 15000000;

/* Retry interval for a position hold the receiver did not take, doubling up to
 * the ceiling. */
constexpr int64_t kHoldRetryUs = 5000000;

constexpr int64_t kHoldRetryMaxUs = 300000000;

/* Wait after adopting a hold before checking it: TMODE2 restarts the pulse
 * generator, which takes longer than a few seconds to settle. */
constexpr int64_t kHoldSettleUs = 30000000;

/* The receiver sends at 1 Hz, so 5 s of silence is a dead line. */
constexpr int64_t kFrameSilenceUs = 5000000;

/* NAV-PVT arrives tens of ms after the edge it describes. An edge older than
 * this cannot be paired with it unambiguously, so the message is not used. */
constexpr int64_t kPairWindowUs = 900000;

/* Edge gap that ends the lock while there is no rate estimate to hold over
 * with. kHoldoverWindowUs applies once there is one. */
constexpr int64_t kLockWindowUs = 3000000;

/* Holdover once the rate is known. Limited by the crystal's temperature
 * wander (kWanderNsPerS), not by the fit. */
constexpr int64_t kHoldoverWindowUs = 600000000;

/* A real edge and the software catch that follows it are microseconds apart;
 * two edges are not. */
constexpr int64_t kSoftCatchWindowUs = 20;

/* MCPWM capture instead of the GPIO interrupt. Off: the capture latches the edge
 * exactly, but relating it to "now" needs a second counter read, and that jitter
 * measured worse than the interrupt stamp (fit worst residual 1.5-5.1 us against
 * 0.5-0.7 us). Measured before the 1588 counter was in use. */
constexpr bool kCaptureEdge = false;

/* Largest believable echo-to-edge gap; beyond it the interrupt's own stamp is
 * used. 50 us even at 320 MHz, and well inside the counter wrap. */
constexpr int32_t kMaxCatchTicks = 16000;

/* The capture timer has no readable count, so a PWM timer in the same group is
 * the "now" end. */
constexpr int kCapGroup = 0;

constexpr int kCapNowTimer = 0;

/* Dispersion allowance for interrupt entry and esp_timer's 1 us resolution. The
 * mean latency cannot be measured against the pulse, so it is a named
 * allowance; pps_latency removes the spread. */
constexpr int64_t kIsrLatencyNs = 2000;

/* Caps on the TIM-TP figures that feed root delay and dispersion, so a corrupt
 * frame cannot reach a client. */
constexpr int64_t kMaxQErrNs = 5000;

constexpr int64_t kMaxJitterNs = 20000;

/* Dispersion growth in holdover: 1 ppm, roughly a small crystal over a room's
 * temperature swing. */
constexpr int64_t kWanderNsPerS = 1000;

/* A jump in (UTC - esp_timer) this large is the module moving the clock or a
 * mispaired second, not drift; the servo restarts. */
constexpr int64_t kRateResetNs = 100000000; /* 100 ms */

/* A reference gap longer than this restarts the rate window: the sample after a
 * gap sits at the end of the baseline, where an error has the most leverage on
 * the fit. One missed edge is tolerated. */
constexpr int64_t kRateGapNs = 3000000000; /* 3 s */

/* Rate clamp: beyond any real crystal, so a larger estimate is a bad fit. */
constexpr int32_t kRateLimitPpb = 200000;

/* Closer edges are bounce or the module reconfiguring its pulse, and counting
 * one would shift every later anchor by a second. */
constexpr int64_t kMinEdgeSpacingUs = 500000;

/* Spacing accepted as the next second. Outside it is a missed or extra pulse. */
constexpr int64_t kEdgeRegularMinUs = 900000;

constexpr int64_t kEdgeRegularMaxUs = 1100000;

/* Consecutive refused edges after which the fit is discarded and rebuilt. */
constexpr uint32_t kRejectStreakReacquire = 45;

/* Phase offset and worst fit residual beyond which the reference is not locked,
 * so a servo that is still ringing is not advertised as stratum 1. */
constexpr int64_t kLockOffsetNs = 100000;

constexpr double kLockTieMaxNs = 20000.0;

/* Consecutive failing edges before a lock gate demotes. */
constexpr uint32_t kGateStreak = 3;

/* Frame-rate floor for keeping a baud rate. A wrong rate can still parse an
 * occasional frame; a right one gives about two a second even while the
 * receiver re-acquires. */
constexpr uint32_t kMinFramesPerSec = 1;

/* Offsets above this are stepped, below it slewed. A step lands tens of us late
 * (read-to-set latency); a slew of 10 ms at IDF's adjtime rate takes under a
 * second. */
constexpr int64_t kStepThresholdNs = 10000000; /* 10 ms */

/* The residual a step leaves is slewed out at once. A larger residual is not
 * the step's latency, so it is left to the servo. */
constexpr int64_t kPostStepSlewNs = 2000000; /* 2 ms */

/* PTP stamps carry the 1588 counter raw, so its phase against the edge is
 * stepped out when it exceeds this for several edges in a row. NTP uses counter
 * differences and does not see the phase. */
constexpr int64_t kPhcPhaseStepNs = 10000; /* 10 us */
constexpr int     kPhcPhaseEdges  = 3;
/* A learned step lag beyond this is a bad edge, not the set path. */
constexpr int64_t kPhcStepLagMaxNs = 200000;

/* An edge this far from the last one's prediction is a corrupt pulse; crystal
 * drift is tens of us per second at most. */
constexpr int64_t kEdgeSpikeNs = 1000000; /* 1 ms */

/* An edge this far from the fit is refused, so one late interrupt cannot hold
 * the fit's worst residual over kLockTieMaxNs for a whole window. A real step in
 * the pulse is accepted after the refusals open a gap the fit restarts across. */
constexpr int64_t kEdgeOutlierNs = 20000; /* 20 us */

/* The decoder, servo fit and NAV-SAT walk run on this stack, and the log helpers
 * build a Status by value. */
constexpr int kTaskStack = 8192;

/* Below the NTP responder and lwIP, above HTTP, the console and PTP, which share
 * its core: at 4, HTTP polling starved the UART into a baud rescan. */
constexpr UBaseType_t kTaskPriority = 7;

/* RFC 5905 refid "GPS\0". */
constexpr uint32_t kRefIdGps = (static_cast<uint32_t>('G') << 24) | ('P' << 16) | ('S' << 8);

inline int64_t clamp(int64_t v, int64_t lo, int64_t hi)
{
    return v > hi ? hi : v < lo ? lo : v;
}

/* Days since 1970-01-01 (Hinnant's days_from_civil), so a UTC date never goes
 * through mktime(), which applies the configured TZ. */
inline int64_t daysFromCivil(int year, unsigned month, unsigned day)
{
    year -= month <= 2;
    const int64_t  era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

/* NAV-PVT date and time to epoch. Rejects the zeros or 1980 default a freshly
 * powered receiver reports. */
inline bool civilToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
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

/* Integer square root, for mm^2 variances. */
inline uint32_t isqrt(uint32_t v)
{
    uint32_t r   = 0;
    uint32_t bit = 1u << 30;
    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

inline const char *fixText(uint8_t fix)
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

}  // namespace gps_detail
