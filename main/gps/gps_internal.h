/*
 * Shared by the parts of GpsTime and by nothing else: the wiring, the windows
 * and limits the parts agree on, and a handful of helpers. The class itself is
 * in gps_time.h; this is what its implementation files have in common.
 */
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
#include "hal/systimer_ll.h"
#include "soc/systimer_struct.h"
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

/* UART1, not UART0: the console owns UART0 on this board (sdkconfig.defaults),
 * and handing the module the same port as the log would be a bad trade. */
constexpr uart_port_t kUart = UART_NUM_1;

constexpr int kTxPin = 15; /* esp TX -> module RX */

constexpr int kRxPin = 16; /* esp RX <- module TX */

constexpr gpio_num_t kPpsPin = GPIO_NUM_14;

/* Order matters, and it is the order the scan walks. 38400 leads because it is
 * what this module is now saved at: the line ran at 43% of 9600 with the
 * diagnostics loaded, and at 10% once it was moved, so the saved rate is the
 * expected one and leading with it saves the twenty-five seconds of scanning
 * that a 9600-first list costs every boot. Every other rate stays reachable, so
 * a receiver reset to its factory default (9600) is still found -- just later.
 *
 * A module that has been through u-center once usually is not on 9600 any more,
 * which is why the list exists at all rather than a single rate. */
constexpr uint32_t kBauds[] = {38400, 9600, 115200, 57600, 19200, 4800};

/* UBX-CFG-MSG enabling TIM-TP on UART1 at 1 Hz. File scope because the probe in
 * task() resends exactly these bytes on their own.
 *
 * Class 0x0D, not 0x01: this is a TIM firmware, and the TIM dialect is the one
 * it speaks -- it answers TIM-SVIN (0x0D 0x04) and refuses NAV-TIM-TP outright,
 * which is the same pair the survey already uses. The two messages are the same
 * 16 bytes; only the class differs, and only this one is answered here. */
constexpr uint8_t kTimTpEnable[8] = {0x0D, 0x01, 0, 1, 0, 0, 0, 0};

/* UBX-TIM-TOS, same TIM family. "Only with Time & Frequency Sync products" per
 * the interface description, so this may be refused -- but it is the only
 * message that states the pulse's offset from the UTC top of second with an
 * uncertainty, and whether the pulse is locked or coherently generated at all.
 * A pulse that is not locked is expected to wander by milliseconds, which is
 * the thing being chased. */
constexpr uint8_t kTimTosEnable[8] = {0x0D, 0x12, 0, 1, 0, 0, 0, 0};

/* How long one baud rate gets to produce a single valid UBX frame. */
constexpr int64_t kScanAfterUs = 4000000;

/* The first rate is the one the module is expected to be on, and it gets longer.
 * Four seconds was not enough and cost a boot: the scan walked off a correct
 * 38400 before the module had finished starting, landed on 9600, and there read
 * mangled frames off a 38400 stream as though they were a link. Fifteen seconds
 * covers a module boot plus a config burst; a wrong rate only costs 15 s of one
 * pass, and the pass starts again afterwards. */
constexpr int64_t kFirstRateUs = 15000000;

/* How soon a position hold that would not take is tried again, and the ceiling
 * the interval backs off to. The first attempt lands right after configureModule
 * at boot, when the receiver may still be booting; five minutes is often enough
 * for a module that was mid-restart and short enough that a boot which failed it
 * does not spend long moving. */
constexpr int64_t kHoldRetryUs = 5000000;

constexpr int64_t kHoldRetryMaxUs = 300000000;

/* How long after adopting a hold the receiver is given before being asked for
 * it. TMODE2 restarts the pulse generator; at four seconds in it is still
 * acquiring, and the cost of doing it early was five seconds of edges, two
 * refusals and the fallback upstream taking the clock. */
constexpr int64_t kHoldSettleUs = 30000000;

/* Silence that counts as a dead line rather than a quiet one. The receiver sends
a message set at 1 Hz, so 5 s is five missed turns of it. */
constexpr int64_t kFrameSilenceUs = 5000000;

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
/* A real edge and the software catch that follows it are microseconds apart;
 * two edges are not. */
constexpr int64_t kSoftCatchWindowUs = 20;

/* The most ticks the echo may be behind the edge before the difference is
 * disbelieved, and the stamp falls back to the interrupt's own time. 16000
 * ticks is 50 us even at the 320 MHz the fastest parts run, far beyond any echo
 * and far inside the wrap. */
/* Whether the PPS edge is taken by the capture peripheral or by a plain
 * interrupt.
 *
 * Measured on this board, four ways, by the rate fit's own worst residual: the
 * interrupt stamp 0.5-0.7 us, the capture stamping at ISR entry 1.5 us, the
 * capture with a software-catch sub-microsecond part 4.3 us, and with a group
 * timer as the "now" end 5.1 us. The capture is not currently worth it here.
 *
 * The reason is the "now" end. A capture has no readable counter of its own, so
 * the interval from the edge has to be measured out of band -- by catching the
 * same counter again, or by a second counter that may not even run at the same
 * rate -- and both of those jitter by more than the interrupt latency the
 * capture removes. The edge *is* latched perfectly; it is the arithmetic that
 * cannot reach it.
 *
 * What would change the answer: a MAC that stamps the frame itself (the 1588
 * unit on this part), where "now" is hardware and the capture's own resolution
 * finally counts. The code is kept, and switching this on is one line.
 */
constexpr bool kCaptureEdge = false;

constexpr int32_t kMaxCatchTicks = 16000;

/* The MCPWM group this uses, and the PWM timer inside it that only ever counts:
 * the capture timer offers nothing readable. */
constexpr int kCapGroup = 0;

constexpr int kCapNowTimer = 0;

constexpr int64_t kIsrLatencyNs = 2000;

/* The receive side used to be the transmit probe's own figure, on the argument
 * that both stamps are software and both err by the same path. The figure is
 * the same but the *variation* is not: the probe reports one syscall taken from
 * an idle task, while the error a client actually sees moved by 72 us in nine
 * minutes with the probe standing still. So the receive side is no longer
 * bounded by a probe at all -- serve_path.cpp measures the drift on the replies
 * that went out, and that number is added to the dispersion directly. */

/* Bounds on what TIM-TP may claim, so a nonsense frame cannot turn into a
 * 10 ms root delay in an answer to a client: a receiver that knows its pulse
 * worse than 5 us is not doing this job, and a spread wider than 20 us is a load
 * spike rather than jitter. */
constexpr int64_t kMaxQErrNs = 5000;

constexpr int64_t kMaxJitterNs = 20000;

/* How fast that promise decays in a holdover: 1 ppm, i.e. 1000 ns of age per
 * second. A small XTAL across a room's temperature swing moves by about this
 * much, and no amount of fitting the last five minutes predicts it. */
constexpr int64_t kWanderNsPerS = 1000;

/* A step in (UTC - esp_timer) larger than this is not the crystal drifting: it
 * is the module moving the clock, or a pairing that landed a second out. Either
 * way the servo's history is about a different clock now, so it starts again. */
constexpr int64_t kRateResetNs = 100000000; /* 100 ms */

/* A gap in the reference longer than this restarts the rate window. The sample
 * that spans a gap averages the rate on both sides of it, and the sample sits at
 * the far end of the baseline where leverage is highest: one 5 ms error there
 * moves a full 300 s fit by about 330 ppb, which is more than a whole tuning
 * step. Two seconds is one missed edge and not worth the churn; three is an
 * outage. (An 80 s outage at 1.6 ppm is 131 ms, and only trips the 100 ms step
 * detector by luck -- a shorter one would have been fitted straight in.) */
constexpr int64_t kRateGapNs = 3000000000; /* 3 s */

/* A larger estimate than this is a bad fit rather than a bad crystal (200 ppm
 * is already ten times a cheap XTAL's worst case), so it is clamped instead of
 * being applied to every timestamp. */
constexpr int32_t kRateLimitPpb = 200000;

/* Edge spacing below this is not a second passing: it is bounce on a header pin
 * or a module reconfiguring its pulse, and counting it would shift every anchor
 * after it by a whole second. */
constexpr int64_t kMinEdgeSpacingUs = 500000;

/* What counts as the next second rather than an extra or a missed one. The ISR
 * above already refuses anything under 500 ms, so what these catch is the rest:
 * an edge at 1.4 s is a pulse that was lost, one at 0.7 s is a receiver emitting
 * twice, and both land in the fit at a place where their error has the most
 * leverage. */
constexpr int64_t kEdgeRegularMinUs = 900000;

constexpr int64_t kEdgeRegularMaxUs = 1100000;

/* Consecutive refused edges that mean the pulse is failing rather than merely
 * glitching once. At 1 Hz this is forty-five seconds of unusable reference: the
 * fit is thrown away and rebuilt rather than carried across the hole. */
constexpr uint32_t kRejectStreakReacquire = 45;

/* How far out the phase may sit, and how wide the fit's worst residual may be,
 * before this stops calling itself locked. Age alone calls a servo locked while
 * it is still ringing; these two magnitudes make "locked" a claim about the
 * clock rather than about the interrupt still firing. The band is five times
 * the worst jitter this reference will ever publish. */
constexpr int64_t kLockOffsetNs = 100000;

constexpr double kLockTieMaxNs = 20000.0;

/* Consecutive edges a lock gate has to hold before it demotes. One bad second
 * is noise on a pulse that is otherwise clean, and demoting for it costs a
 * client a sample; three in a row is a servo or a fit that has actually gone
 * wrong. */
constexpr uint32_t kGateStreak = 3;

/* The silence test never fires while a wrong rate still parses the odd frame.
 * But the floor has to sit below what a *correct* rate can look like while the
 * receiver re-acquires -- roughly two frames a second here, mostly ACKs -- or
 * the scan walks off a rate that was working. A wrong rate gives far less. */
constexpr uint32_t kMinFramesPerSec = 1;

/* Where a step stops being cheaper than a slew -- and it is far lower than the
 * slew rate alone suggests, because a step does not land on target. A step is a
 * read of the clock followed by a set(): the set reaches the timebase tens of
 * microseconds after the instant the timestamp was taken, so what it leaves
 * behind is that latency, and the servo spends the following seconds removing
 * it. A slew leaves nothing behind and is invisible to a client.
 *
 * So the threshold belongs at the point where the slew starts costing more than
 * the step's own latency: 10 ms at IDF's ~1.5% per call converges in under a
 * second, where the old 250 ms figure meant sixteen seconds of a knowingly
 * wrong clock to save a write. Steps remain for the first set and for a module
 * that has moved the clock under us, where the error is past what a slew can
 * follow anyway. */
constexpr int64_t kStepThresholdNs = 10000000; /* 10 ms */

/* What a step leaves behind is bounded by the syscall path rather than by the
 * servo, so it is removed with a slew straight away instead of waiting for the
 * next edge to notice. Wider than this and the re-measurement has stopped
 * describing the step, so it is left alone. */
constexpr int64_t kPostStepSlewNs = 2000000; /* 2 ms */

/* The 1588 counter's phase against the second, as PTP stamps carry it raw. The
 * served time only ever uses counter differences, so nothing else notices a
 * phase error: set once from CLOCK_REALTIME, it sat 0.65 ms out for good. A few
 * edges in a row, because one edge read just before a step elsewhere would
 * show the step as its phase. */
constexpr int64_t kPhcPhaseStepNs = 10000; /* 10 us */
constexpr int     kPhcPhaseEdges  = 3;
/* A learned step lag beyond this is a bad edge, not the set path. */
constexpr int64_t kPhcStepLagMaxNs = 200000;

/* An edge landing further than this from where the last one put it is a
 * corrupted pulse rather than a slow crystal: the crystal's own worst case is
 * tens of microseconds a second, and the loop only ever sees the remainder. */
constexpr int64_t kEdgeSpikeNs = 1000000; /* 1 ms */

/* An edge this far from where the fit put it is not used. The pulse is stamped
 * by an interrupt, and an interrupt can be held off: one edge stamped 45 us
 * late was anchored, entered the fit, and held the worst residual over
 * kLockTieMaxNs for the 300 s it took to leave the window -- five minutes at
 * stratum 16 for one late interrupt, on a fit whose RMS was 0.36 us. The limit
 * is the lock test's own, so a sample that is let in cannot close it alone.
 *
 * A real step in the pulse is refused too, and that is bounded: three refusals
 * make a gap the fit restarts across, and the fourth edge is taken as it is. */
constexpr int64_t kEdgeOutlierNs = 20000; /* 20 us */

/* 6 kB, raised from 4096 when the sky view landed. The decoder, the servo fit
 * and the NAV-SAT walk all run here, and a frame walk that keeps a row per
 * satellite is what pushed the old figure over: it overflowed into a panic
 * loop that only showed as the box re-DHCPing every ten seconds. */
/* 8192: status() is a large by-value Status and the same fast_float call the
 * responder hit, and this task builds Status in its own log helpers. */
constexpr int kTaskStack = 8192;

/* Below the responder and the network stack, which answer packets, and above
 * the HTTP server, the console and PTP. All of them share one core now
 * (app/cores.h), and at 4 a client polling the HTTP API kept this task off the
 * UART for as long as it polled: no frames, no anchors, then a baud rescan. */
constexpr UBaseType_t kTaskPriority = 7;

/* NTP refid for this reference, spelled the way RFC 5905 spells it: four ASCII
 * bytes, the last a NUL that the responder's text formatter trims. */
constexpr uint32_t kRefIdGps = (static_cast<uint32_t>('G') << 24) | ('P' << 16) | ('S' << 8);

/* Squeeze a value into a range. The record narrowing below clamps at four
 * different widths and the expression was written out at each. */
inline int64_t clamp(int64_t v, int64_t lo, int64_t hi)
{
    return v > hi ? hi : v < lo ? lo : v;
}

/* Days since 1970-01-01 from a proleptic Gregorian date (Hinnant's
 * days_from_civil). Written out rather than reached for: timegm() is not in this
 * libc, and mktime() would apply the POSIX TZ from Settings -- turning UTC from
 * the receiver into local time, which is exactly the bug that would leave this
 * server an hour out. */
inline int64_t daysFromCivil(int year, unsigned month, unsigned day)
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

/* Integer square root, for turning a variance in mm squared into an accuracy in
 * mm without pulling in floats. */
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
