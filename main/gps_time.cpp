#include "gps_time.h"

#include "ptp.h"

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
#include "esp_timer.h"
#include "hal/mcpwm_ll.h"

#include "clock_source.h"
#include "serve_path.h"
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
constexpr int64_t kHoldRetryUs     = 5000000;
constexpr int64_t kHoldRetryMaxUs  = 300000000;
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
constexpr int kCapGroup    = 0;
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
constexpr double  kLockTieMaxNs = 20000.0;

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

/* An edge landing further than this from where the last one put it is a
 * corrupted pulse rather than a slow crystal: the crystal's own worst case is
 * tens of microseconds a second, and the loop only ever sees the remainder. */
constexpr int64_t kEdgeSpikeNs = 1000000; /* 1 ms */

/* 6 kB, raised from 4096 when the sky view landed. The decoder, the servo fit
 * and the NAV-SAT walk all run here, and a frame walk that keeps a row per
 * satellite is what pushed the old figure over: it overflowed into a panic
 * loop that only showed as the box re-DHCPing every ten seconds. */
/* 8192: status() is a large by-value Status and the same fast_float call the
 * responder hit, and this task builds Status in its own log helpers. */
constexpr int          kTaskStack    = 8192;
constexpr UBaseType_t  kTaskPriority = 4; /* below the responder: it answers packets */

/* NTP refid for this reference, spelled the way RFC 5905 spells it: four ASCII
 * bytes, the last a NUL that the responder's text formatter trims. */
constexpr uint32_t kRefIdGps = (static_cast<uint32_t>('G') << 24) | ('P' << 16) | ('S' << 8);

/* Squeeze a value into a range. The record narrowing below clamps at four
 * different widths and the expression was written out at each. */
static int64_t clamp(int64_t v, int64_t lo, int64_t hi)
{
    return v > hi ? hi : v < lo ? lo : v;
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

void GpsTime::applyBias(bool on)
{
    m_antBiasOn = on;
    gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
}

namespace {

/* Integer square root, for turning a variance in mm squared into an accuracy in
 * mm without pulling in floats. */
uint32_t isqrt(uint32_t v)
{
    uint32_t r = 0;
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

}  // namespace

void GpsTime::requestAntennaBias(bool on)
{
    m_biasReq = on ? 1 : 0;
}

void GpsTime::requestGnss(uint8_t mask)
{
    m_gnssReqMask = mask;
    m_gnssReq     = true;
}

void GpsTime::requestSurvey(bool start, uint32_t durS, uint32_t accMm)
{
    m_svinDurS  = durS;
    m_svinAccMm = accMm;
    m_svinCmd   = start ? 1 : 2;
}

void GpsTime::requestFixPosition()
{
    m_svinCmd = 3;
}

void GpsTime::requestAntDelay()
{
    m_tp5Cmd = true;
}

/* Pulse width, as a fraction of the second. Kept in the object only: it is a
 * bench knob for the timepulse, not a site setting. */
void GpsTime::requestPulseMs(uint32_t ms)
{
    m_pulseMs = ms;
    m_tp5Cmd  = true;
}

/* Period in microseconds, so 1000000 is 1 Hz. Changing it is the structural
 * test of whether this frame reaches the hardware at all: the pulse then stops
 * matching the second and every anchor shifts by a known fraction. */
void GpsTime::requestPeriodUs(uint32_t us)
{
    m_periodUs = us;
    m_tp5Cmd   = true;
}

void GpsTime::requestBaud(uint32_t baud, bool moduleToo)
{
    m_baudReq       = baud;
    m_baudReqModule = moduleToo;
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
    const int body = std::snprintf(bodyText, sizeof(bodyText), "PUBX,41,1,0003,0001,%u,0",
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
    text::putLe(cfg + 0, 0u);        /* clearMask: nothing */
    text::putLe(cfg + 4, 0x00000001u); /* saveMask: IO */
    text::putLe(cfg + 8, 0u);        /* loadMask: nothing */
    return sendAndWait(0x06, 0x09, cfg, sizeof(cfg));
}

int8_t GpsTime::sendCfgPrt(uint32_t baud)
{
    uint8_t cfg[20] = {};
    cfg[0] = 1;                                          /* portID: UART1 */
    text::putLe(cfg + 4, 0x000008D0u);                   /* 8 data bits, no parity, 1 stop */
    text::putLe(cfg + 8, baud);
    /* The masks are BITFIELDS: bit 0 UBX, bit 1 NMEA, bit 2 RTCM, bit 5 RTCM3.
     * Outgoing has to keep bit 0 or the receiver stops sending UBX altogether
     * and the only frames left are the ACKs to these very messages -- which read
     * as a live-but-idle link and strand the scan, because a trickle keeps it
     * from ever advancing. Incoming keeps NMEA too, harmlessly, because a
     * module someone has been at with u-center may still be emitting it. */
    text::putLe(cfg + 12, 0x0003u);                      /* in: UBX | NMEA */
    text::putLe(cfg + 14, 0x0001u);                      /* out: UBX */
    return sendAndWait(0x06, 0x00, cfg, sizeof(cfg));
}

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
    Settings   &settings = Settings::instance();
    const std::string &mode = settings.gpsTimeMode();

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
    long long     x   = 0;
    long long     y   = 0;
    long long     z   = 0;
    unsigned long acc = 0;
    unsigned long val = 0;
    const std::string &pos = Settings::instance().gpsFixedPos();
    const bool stored = (text::getList(pos, ',', x, y, z, acc, val) ||
                         text::getList(pos, ',', x, y, z, acc)) &&
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

void GpsTime::requestDefaults()
{
    m_defaultsReq = true;
}

void GpsTime::requestReset(bool cold, bool hardware, bool powerCycle)
{
    m_resetCold  = cold;
    m_resetHard  = hardware;
    m_resetPower = powerCycle;
    m_resetReq   = true;
}

void GpsTime::bridgeTaskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->bridgeTask();
}

void GpsTime::bridgeTask()
{
    /* Four clients, and the listening socket stays open. u-center taking the
     * port must not lock out a shell, and the single-client version did. Four
     * rather than two because a socket that was closed without the peer saying
     * so keeps its slot until the next select sees it readable, and u-center
     * dead against a bridge with both slots full looks exactly like u-center
     * dead against a bridge that is ignoring it. */
    constexpr int kMaxClients = 4;

    /* What a session sets, in the CFG-MSG shape configureModule() uses: the
     * message class, its id, then one rate per port -- DDC, UART1, UART2, USB,
     * SPI. UART1 only, because it is the only port wired to anything.
     *
     * NAV-SAT is here because the sky view and the C/N0 bars are drawn from it;
     * it is what a bench session exists for. NAV-PVT and NAV-TIMELS are because
     * a session can set every rate to zero -- u-center's revert-to-defaults does
     * exactly that -- and without them a session stops the clock as well as
     * drawing nothing. Three frames and three ACKs, once, at attach. */
    static constexpr uint8_t kSessionPvt[8]    = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSessionTimels[8] = {0x01, 0x26, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSessionSat[8]    = {0x01, 0x35, 0, 1, 0, 0, 0, 0};

    for (;;) {
        const int srv = socket(AF_INET, SOCK_STREAM, 0);
        if (srv < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        int yes = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr = {};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(kBridgePort);
        if (bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
            listen(srv, kMaxClients) != 0) {
            close(srv);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "u-center bridge on tcp/%d", kBridgePort);

        int clients[kMaxClients] = {-1, -1, -1, -1};
        bool quieted = false;
        /* u-center opens a TCP port by polling MON-COMMS, and this receiver
         * ignores it. A synthetic reply goes out if the module stays silent --
         * 400 ms, because the module's silence is a property of the message and
         * not of the line, and a client waiting on it is waiting on us. */
        int     commsClient   = -1;
        int64_t commsDeadline = 0;
        /* An armed rate change: what was asked for, whether the receiver has
         * acknowledged it, and when to stop waiting. */
        uint32_t baudWanted = 0;
        bool     baudAcked  = false;
        int64_t  baudAtUs   = 0;

        /* The two directions are decoded separately because they interleave and
         * must not share parse state: the client's session and the receiver's
         * message set are different streams that happen to cross one socket. */
        ubx::Decoder clientDecoder;
        ubx::Decoder moduleDecoder;

        /* What the bridge wants to notice in the client's traffic: a request to
         * move the port's rate, and a MON-COMMS poll that will need answering. */
        struct ClientSniffer : ubx::Listener {
            uint32_t *wanted;
            bool     *acked;
            int64_t  *atUs;
            int      *commsClient;
            int64_t  *commsDeadline;
            int       fd = -1;

            ClientSniffer(uint32_t *w, bool *a, int64_t *at, int *cp, int64_t *cd,
                          volatile uint32_t *dr)
                : wanted(w), acked(a), atUs(at), commsClient(cp), commsDeadline(cd), dropped(dr)
            {
            }

            volatile uint32_t *dropped;

            void onDrop(ubx::Drop why, const ubx::Frame &) override
            {
                if (why == ubx::Drop::Checksum) {
                    *dropped = *dropped + 1;
                }
            }

            void onFrame(const ubx::Frame &f) override
            {
                if (f.cls == 0x06 && f.id == 0x00) {
                    /* CFG-PRT is 20 bytes and the decoder has already enforced
                     * it. baudRate is a four-byte field at offset 8; reading two
                     * of those bytes, as this used to, turns 115200 into 49664
                     * and 460800 into 2048. */
                    const uint8_t  port = f.u8(0);
                    const uint32_t baud = f.u4(8);
                    if (port != 1) {
                        ESP_LOGI(TAG, "port %u set to %u: not this line", port,
                                 static_cast<unsigned>(baud));
                        return;
                    }
                    if (baud < 4800 || baud > 921600) {
                        ESP_LOGW(TAG, "%u baud is outside 4800..921600: not following",
                                 static_cast<unsigned>(baud));
                        return;
                    }
                    *wanted = baud;
                    *acked  = false;
                    *atUs   = esp_timer_get_time() + 250000;
                } else if (f.cls == 0x0A && f.id == 0x36) {
                    *commsClient   = fd;
                    *commsDeadline = esp_timer_get_time() + 400000;
                }
            }
        } clientSniffer(&baudWanted, &baudAcked, &baudAtUs, &commsClient, &commsDeadline,
                       &m_bridgeDroppedCrc);

        /* ...and in the receiver's: the answer to that request, and a MON-COMMS
         * reply it bothered to send itself. */
        struct ModuleSniffer : ubx::Listener {
            uint32_t       *wanted;
            bool           *acked;
            int64_t        *atUs;
            int            *commsClient;
            int             fd = -1;

            ModuleSniffer(uint32_t *w, bool *a, int64_t *at, int *cp)
                : wanted(w), acked(a), atUs(at), commsClient(cp)
            {
            }

            void onDrop(ubx::Drop, const ubx::Frame &) override {}

            void onFrame(const ubx::Frame &f) override
            {
                if (f.cls == 0x05 && (f.id == 0x00 || f.id == 0x01) && *wanted != 0 &&
                    f.len >= 2 && f.u8(0) == 0x06 && f.u8(1) == 0x00) {
                    if (f.id == 0x00) {
                        *acked = true;
                        *atUs  = esp_timer_get_time();
                    } else {
                        ESP_LOGW(TAG, "receiver refused the %u baud change",
                                 static_cast<unsigned>(*wanted));
                        *wanted = 0;
                    }
                } else if (f.cls == 0x0A && f.id == 0x36 && *commsClient >= 0) {
                    *commsClient = -1;
                }
            }
        } moduleSniffer(&baudWanted, &baudAcked, &baudAtUs, &commsClient);

        clientDecoder.listen(&clientSniffer);
        moduleDecoder.listen(&moduleSniffer);

        /* Every way a client ends goes through here, so the detached state --
         * port handed back, own configuration restored -- cannot be forgotten
         * on one of the exits. It was forgotten on all of them before: the flag
         * that stops the GPS task reading was only ever cleared when the
         * listening socket itself failed, so after the first session the task
         * never read the port again and the receiver kept a session's rates. */
        auto dropClient = [&](int i, const char *why) {
            text::put(m_bridgeEnd, "{}", why);
            ESP_LOGW(TAG, "client gone: %s", why);
            close(clients[i]);
            clients[i] = -1;
            m_bridgeClients = m_bridgeClients - 1;
            bool any = false;
            for (int j = 0; j < kMaxClients; ++j) {
                any = any || clients[j] >= 0;
            }
            if (!any) {
                quieted        = false;
                m_bridgeActive = false;
                /* Two seconds, not one, and it is a debounce rather than a
                 * delay: a client that is unhappy reconnects about once a
                 * second, and configureModule() takes roughly three of them
                 * because it waits out an ACK on every message. Restoring on
                 * the first detach therefore never finishes before the next
                 * attach stands it aside again, and a bouncing client would
                 * thrash the receiver without either side settling. A pending
                 * reconfigure is stood aside on attach anyway. */
                m_reconfigAtUs = esp_timer_get_time() + 2000000;
            }
        };

        for (;;) {
            fd_set set;
            FD_ZERO(&set);
            FD_SET(srv, &set);
            int maxFd = srv;
            for (int i = 0; i < kMaxClients; ++i) {
                if (clients[i] >= 0) {
                    FD_SET(clients[i], &set);
                    if (clients[i] > maxFd) {
                        maxFd = clients[i];
                    }
                }
            }
            timeval tv = {.tv_sec = 0, .tv_usec = 50000};
            if (select(maxFd + 1, &set, nullptr, nullptr, &tv) < 0) {
                break;
            }

            if (FD_ISSET(srv, &set)) {
                sockaddr_in peer    = {};
                socklen_t   peerLen = sizeof(peer);
                const int   c       = accept(srv, reinterpret_cast<sockaddr *>(&peer), &peerLen);
                if (c >= 0) {
                    int slot = -1;
                    for (int i = 0; i < kMaxClients; ++i) {
                        if (clients[i] < 0) {
                            slot = i;
                            break;
                        }
                    }
                    if (slot < 0) {
                        /* Both slots full. Worth reporting: a client that is
                         * accepted and immediately closed looks identical from
                         * the outside to one that attaches and is ignored, and
                         * telling those apart has cost a round already. */
                        m_bridgeRefused = m_bridgeRefused + 1;
                        text::put(m_bridgeEnd, "refused: both slots taken");
                        ESP_LOGW(TAG, "client refused from %s: both slots taken",
                                 inet_ntoa(peer.sin_addr));
                        close(c);
                    } else {
                        fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
                        clients[slot]    = c;
                        clientSniffer.fd = c;
                        moduleSniffer.fd = c;
                        text::put(m_bridgeLast, "attach");
                        ESP_LOGW(TAG, "client attached from %s", inet_ntoa(peer.sin_addr));
                        m_bridgeSessions = m_bridgeSessions + 1;
                        m_bridgeClients  = m_bridgeClients + 1;
                        if (!quieted) {
                            /* Claim the port and put the session's message set
                             * on it. No delay first: u-center's polls are
                             * already in the socket by the time we get here, so
                             * waiting only moves our frames into the middle of
                             * the handshake it is trying to run. */
                            quieted        = true;
                            m_bridgeActive = true;
                            sendUbx(0x06, 0x01, kSessionPvt, sizeof(kSessionPvt));
                            sendUbx(0x06, 0x01, kSessionTimels, sizeof(kSessionTimels));
                            sendUbx(0x06, 0x01, kSessionSat, sizeof(kSessionSat));
                        }
                    }
                }
            }

            for (int i = 0; i < kMaxClients; ++i) {
                if (clients[i] < 0 || !FD_ISSET(clients[i], &set)) {
                    continue;
                }
                uint8_t       in[256];
                const int     n = recv(clients[i], in, sizeof(in), 0);
                if (n <= 0 && !(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                    text::Buf<48> why;
                    text::put(why, "recv {} errno {}", n, errno);
                    dropClient(i, why.data());
                    continue;
                }
                if (n <= 0) {
                    continue;
                }
                m_bridgeRx = m_bridgeRx + static_cast<uint32_t>(n);

                /* Built up as text and then cut to the buffer: snprintf returns
                 * the length it *wanted*, so accumulating its return value into
                 * an offset lets the offset pass the end of the array on a long
                 * frame and `sizeof - offset` underflow to something huge. */
                std::string hex;
                for (int k = 0; k < n && k < 40; ++k) {
                    hex += std::format("{:02x} ", static_cast<unsigned>(in[k]));
                }
                text::put(m_bridgeLast, "{}", hex);
                ESP_LOGI(TAG, "client -> %d bytes: %s", n, hex.c_str());

                clientDecoder.feed(std::span(in, static_cast<std::size_t>(n)));
                uart_write_bytes(kUart, in, n);
            }

            uint8_t   out[256];
            /* Only while a client holds the port. With nobody attached the GPS
             * task is the reader, and a second one here took whatever sat in
             * the ring between its reads -- 7280 bytes with no session ever
             * opened, each theft a frame with a hole in it for the clock's
             * decoder. */
            const int got = m_bridgeActive ? uart_read_bytes(kUart, out, sizeof(out), 0) : 0;
            moduleDecoder.feed(std::span(out, static_cast<std::size_t>(got > 0 ? got : 0)));

            /* The rate change, if one is armed. The old code switched the baud
             * the instant it found the request in the client's bytes -- which is
             * when uart_write_bytes() returns, not when the bytes have gone --
             * so the tail of the very frame asking for the change could leave at
             * the new rate while its head left at the old one. It now waits for
             * the module's ACK, drains the transmitter, and switches. A NAK
             * cancels; an ACK that never comes does not, because u-blox sends it
             * on whichever side of the change it likes and a client left at the
             * wrong rate is worse than a rate change the receiver grumbled at. */
            if (baudWanted != 0 && esp_timer_get_time() >= baudAtUs) {
                const uint32_t want = baudWanted;
                baudWanted          = 0;
                if (baudAcked) {
                    ESP_LOGW(TAG, "client asked for %u baud: following", static_cast<unsigned>(want));
                } else {
                    ESP_LOGW(TAG, "client asked for %u baud: no ACK, following anyway",
                             static_cast<unsigned>(want));
                }
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                uart_set_baudrate(kUart, want);
                m_baud = want;
                /* The receiver has almost certainly been asked to change too, so
                 * the next PPS pairing is not comparable with the last. */
                m_frameSeen = false;
            }
            if (commsClient >= 0 && esp_timer_get_time() > commsDeadline) {
                /* The receiver ignores MON-COMMS outright -- MON-VER, CFG-PRT and
                 * NAV-SAT polls all get answers on the same line, this one gets
                 * nothing -- and u-center opens a TCP port by polling it. So the
                 * bridge answers for the module.
                 *
                 * Built as a real reply for one port: version 0, nPorts 1, port 1
                 * (UART1, the only port here), every counter zero. The first
                 * version of this sent nPorts 0 in a 12-byte payload when a
                 * zero-port payload is 8, so a client that enumerates ports to
                 * decide what it is talking to had a length it could not walk
                 * and nothing in the list. The counters stay honest zeros: the
                 * module does not report them here and inventing figures would
                 * be worse than a blank. */
                constexpr int kLen = 8 + 34;
                uint8_t       reply[kLen] = {0xB5, 0x62, 0x0A, 0x36, 34, 0x00};
                reply[6 + 4]              = 1; /* nPorts */
                reply[6 + 8]              = 1; /* portId, little endian */
                uint8_t a = 0;
                uint8_t b = 0;
                for (int i = 2; i < kLen - 2; ++i) {
                    a = static_cast<uint8_t>(a + reply[i]);
                    b = static_cast<uint8_t>(b + a);
                }
                reply[kLen - 2] = a;
                reply[kLen - 1] = b;
                if (send(commsClient, reply, sizeof(reply), 0) > 0) {
                    m_bridgeTx = m_bridgeTx + sizeof(reply);
                }
                ESP_LOGI(TAG, "answered MON-COMMS locally (the module does not)");
                commsClient = -1;
            }

            if (got > 0) {
                m_bridgeTx = m_bridgeTx + static_cast<uint32_t>(got);
                for (int i = 0; i < kMaxClients; ++i) {
                    if (clients[i] < 0) {
                        continue;
                    }
                    int sent   = 0;
                    int64_t by = esp_timer_get_time() + 500000;
                    while (sent < got && esp_timer_get_time() < by) {
                        const int w = send(clients[i], out + sent, got - sent, 0);
                        if (w > 0) {
                            sent += w;
                        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                            vTaskDelay(pdMS_TO_TICKS(2));
                        } else {
                            text::Buf<48> why;
                            text::put(why, "send errno {}", errno);
                            dropClient(i, why.data());
                            break;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < kMaxClients; ++i) {
            if (clients[i] >= 0) {
                close(clients[i]);
            }
        }
        close(srv);
        m_bridgeActive  = false;
        m_bridgeClients = 0;
        ESP_LOGW(TAG, "bridge reset; re-applying configuration");
        m_reconfigAtUs = esp_timer_get_time() + 500000;
    }
}

void IRAM_ATTR GpsTime::recordEdge(const int64_t edgeNs)
{
    const int64_t nowUs = edgeNs / 1000;
    /* Stamped here rather than in the capture handler because this is the path
     * both handlers share: the capture peripheral is not always the one in use
     * (the GPIO fallback is what runs when the timer cannot be set up), and
     * anything waiting on "an edge arrived" was waiting on the wrong one. */
    m_lastEdgeUs = nowUs;

    /* The same edge in the 1588 counter's base, read before the lock rather
     * than under it: the other core can hold m_mux for the length of a status
     * copy, and a read taken after that wait put the counter's anchor late by
     * the wait while the esp_timer stamp, taken on entry, was not. */
    const int64_t edgePhcNs = ptp::hardware() ? ptp::nowNs() : 0;

    portENTER_CRITICAL_ISR(&m_mux);
    /* The POST /gps/pps hook swallows the edge here rather than further up, so
     * what it produces is the real holdover code path and not an imitation of
     * it. One extra comparison, against a word that is zero unless somebody
     * asked for the test. */
    if (nowUs >= m_ppsSuppressUntilUs &&
        (m_ppsCount == 0 || nowUs - m_ppsMonoUs >= kMinEdgeSpacingUs)) {
        m_ppsMonoUs = nowUs;
        m_ppsMonoNs = edgeNs;
        if (edgePhcNs != 0) {
            m_ppsPtpNs = edgePhcNs;
        }
        ++m_ppsCount;
    }
    portEXIT_CRITICAL_ISR(&m_mux);
}

/* The fallback edge handler, used only if the capture timer cannot be set up.
 * Stamps with esp_timer at whole-microsecond resolution, so the residual
 * carries the interrupt latency. */
void IRAM_ATTR GpsTime::ppsIsr(void *arg)
{
    auto *self = static_cast<GpsTime *>(arg);
    /* esp_timer's counter is in IRAM (CONFIG_ESP_TIMER_IN_IRAM), which is what
     * lets this handler be an IRAM interrupt: an OTA write disables the flash
     * cache for milliseconds at a time, and an edge landing in that window must
     * not be a crash. */
    self->recordEdge(esp_timer_get_time() * 1000);
}

/* The edge as the capture peripheral latched it.
 *
 * The tick is latched by hardware at the edge, so no interrupt latency is in
 * the stamp. What is *not* available is a way to read the counter, so the
 * "now" end of the difference is taken by catching the same counter from
 * software and reading that latch. The interval between the two is a few ticks,
 * which is why the sub-microsecond part is finally real: the arithmetic happens
 * in esp_timer's own domain (now minus the elapsed ticks), not against a
 * foreign timebase. That is the difference from the two earlier attempts the
 * old comment describes -- neither systimer's raw count nor the CPU cycle
 * counter shares esp_timer's phase, but this does, because "now" is esp_timer.
 */
bool IRAM_ATTR GpsTime::ppsCapIsr(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *edge,
                                  void *arg)
{
    auto *self = static_cast<GpsTime *>(arg);
    const int64_t atUs = esp_timer_get_time();

    /* The software catch below runs this handler again a tick or two later.
     * Real edges are a second apart, so anything this close is that echo. The
     * store itself is in recordEdge, which both handlers reach. */
    if (self->m_lastEdgeUs >= 0 && atUs - self->m_lastEdgeUs < kSoftCatchWindowUs) {
        return false;
    }

    int64_t edgeNs = atUs * 1000;
    if (self->m_capNowValid && self->m_capHz != 0) {
        /* Wrap-safe: one counter, one signed difference. */
        const uint32_t now =
            mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
        const int32_t ticks = static_cast<int32_t>(now - edge->cap_value);
        if (ticks >= 0 && ticks <= kMaxCatchTicks) {
            edgeNs -= static_cast<int64_t>(ticks) * 1000000000LL / self->m_capHz;
        }
    }

    self->recordEdge(edgeNs);
    return false;
}

void GpsTime::taskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->task();
}

/* The fallback: a GPIO interrupt that stamps on entry. Level 3, above the UART\'s
 * level 1 and the Ethernet\'s, so the edge nests over a serial or MAC handler
 * already running instead of waiting for it, and IRAM so it survives the flash
 * cache going away under an OTA write. Neither comes free: at level 3 the
 * handler may only use what is safe there. */
esp_err_t GpsTime::beginInterruptEdge()
{
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR service already installed: the PPS handler may not be in IRAM");
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(kPpsPin, &GpsTime::ppsIsr, this);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PPS edge: GPIO interrupt (stamp on entry, interrupt latency included)");
    }
    return err;
}

/* The edge is latched by the capture peripheral at the edge itself, so the
 * stamp does not carry the interrupt latency, and the resolution is a tick of
 * the 80 MHz APB clock. The ISR only publishes the already-latched value.
 *
 * MCPWM needs its interrupt handler and its control functions in IRAM for this
 * to keep the old interrupt\'s property of running with the flash cache off
 * (CONFIG_MCPWM_ISR_CACHE_SAFE and CONFIG_MCPWM_CTRL_FUNC_IN_IRAM, both pinned in
 * sdkconfig.defaults), because the software catch used to read the counter is a
 * control call.
 */
esp_err_t GpsTime::beginCapture()
{
    mcpwm_capture_timer_config_t timerConfig = {};
    timerConfig.group_id                     = 0;
    timerConfig.clk_src                      = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    esp_err_t err = mcpwm_new_capture_timer(&timerConfig, &m_capTimer);
    if (err != ESP_OK) {
        return err;
    }

    mcpwm_capture_channel_config_t channelConfig = {};
    channelConfig.gpio_num                       = kPpsPin;
    channelConfig.prescale                       = 1;
    channelConfig.flags.pos_edge                 = 1;
    channelConfig.flags.neg_edge                 = 0;

    err = mcpwm_new_capture_channel(m_capTimer, &channelConfig, &m_capChannel);
    if (err != ESP_OK) {
        return err;
    }

    mcpwm_capture_event_callbacks_t callbacks = {};
    callbacks.on_cap                          = &GpsTime::ppsCapIsr;
    err = mcpwm_capture_channel_register_event_callbacks(m_capChannel, &callbacks, this);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_channel_enable(m_capChannel);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_timer_enable(m_capTimer);
    if (err != ESP_OK) {
        return err;
    }
    err = mcpwm_capture_timer_start(m_capTimer);
    if (err != ESP_OK) {
        return err;
    }

    /* "Now" in the same clock. The capture timer has no readable counter, and
     * the software catch that would supply one carries jitter: as measured it
     * took the fit residual from 1.5 us to 4.3 us, worse than the latency the
     * capture removes. A PWM timer in the same group counts from the same clock
     * and costs one register read, so it is the "now" end -- after a check that
     * the two really do count at one rate. */
    mcpwm_ll_timer_set_count_mode(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                 MCPWM_TIMER_COUNT_MODE_UP);
    mcpwm_ll_timer_set_start_stop_command(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer,
                                         MCPWM_TIMER_START_NO_STOP);

    /* In Hz, not nanoseconds, and not the same on every part. */
    err = mcpwm_capture_timer_get_resolution(m_capTimer, &m_capHz);
    if (err != ESP_OK || m_capHz == 0) {
        ESP_LOGW(TAG, "capture resolution unknown: the stamp falls back to the interrupt time");
        m_capHz = 0;
    } else {
        ESP_LOGI(TAG, "PPS edge: MCPWM capture at %lu Hz (%.1f ns a tick, latched at the edge, "
                      "no ISR latency)",
                 static_cast<unsigned long>(m_capHz), 1e9 / static_cast<double>(m_capHz));
    }

    /* Rate check: both counters are sampled across 10 ms through the same
     * jittery path, so jitter washes out and only a real clock difference shows.
     * The sub-microsecond part is dropped if they disagree. */
    uint32_t capFirst = 0, capLast = 0, nowFirst = 0, nowLast = 0;
    bool     sampled  = false;
    if (mcpwm_capture_channel_trigger_soft_catch(m_capChannel) == ESP_OK &&
        mcpwm_capture_get_latched_value(m_capChannel, &capFirst) == ESP_OK) {
        nowFirst = mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
        vTaskDelay(pdMS_TO_TICKS(10));
        if (mcpwm_capture_channel_trigger_soft_catch(m_capChannel) == ESP_OK &&
            mcpwm_capture_get_latched_value(m_capChannel, &capLast) == ESP_OK) {
            nowLast = mcpwm_ll_timer_get_count_value(MCPWM_LL_GET_HW(kCapGroup), kCapNowTimer);
            sampled = true;
        }
    }
    if (sampled) {
        const int32_t capRate = static_cast<int32_t>(capLast - capFirst);
        const int32_t nowRate = static_cast<int32_t>(nowLast - nowFirst);
        if (capRate > 0 && nowRate > 0) {
            const double ratio = static_cast<double>(nowRate) / static_cast<double>(capRate);
            m_capNowValid      = ratio > 0.99 && ratio < 1.01;
            ESP_LOGI(TAG, "capture counter vs group timer: %.4f%s", ratio,
                     m_capNowValid ? "" : " -- sub-microsecond stamp disabled");
        }
    }
    return ESP_OK;
}

esp_err_t GpsTime::begin()
{
    if (m_begun) {
        return ESP_OK;
    }

    /* Start at the rate this receiver was last heard on rather than at the
     * head of the list. The module keeps its own rate across a power cut, so
     * assuming the head means every boot walks the scan while the receiver is
     * still starting -- and a boot that arrives here with the module slow to
     * talk loses the correct rate and takes minutes to come back to it. */
    const std::string stored = Settings::instance().gpsBaud();
    m_preferredIndex         = 0;
    for (int i = 0; i < static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0])); ++i) {
        if (stored == std::to_string(kBauds[i])) {
            m_preferredIndex = i;
            break;
        }
    }
    m_baudIndex = m_preferredIndex;

    uart_config_t config         = {};
    config.baud_rate             = static_cast<int>(kBauds[m_preferredIndex]);
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

    err = kCaptureEdge ? beginCapture() : ESP_ERR_NOT_SUPPORTED;
    if (err != ESP_OK) {
        /* The capture is how the edge is stamped without interrupt latency in
         * it, but a plain GPIO interrupt is a working second best and it is not
         * worth refusing to run over. */
        ESP_LOGW(TAG, "MCPWM capture unavailable (%s): falling back to an interrupt stamp",
                 esp_err_to_name(err));
        err = beginInterruptEdge();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS edge on GPIO%d: %s", static_cast<int>(kPpsPin), esp_err_to_name(err));
        return err;
    }

    if (xTaskCreate(&GpsTime::taskEntry, "gps", kTaskStack, this, kTaskPriority, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "cannot start the GPS task");
        return ESP_ERR_NO_MEM;
    }

    /* The bias switch. Output, idle high (off, which is also the board's own
     * default), then turned on below; the sense pin is an input with the board's
     * pull-up already on the net. */
    gpio_config_t bias = {};
    bias.pin_bit_mask  = (1ULL << kAntDrivePin) | (1ULL << kAntSensePin);
    bias.mode          = GPIO_MODE_INPUT_OUTPUT;
    bias.pull_up_en    = GPIO_PULLUP_ENABLE;
    bias.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    bias.intr_type     = GPIO_INTR_DISABLE;
    if (gpio_config(&bias) != ESP_OK) {
        ESP_LOGW(TAG, "cannot claim GPIO%d/%d for the bias switch", kAntDrivePin, kAntSensePin);
    } else {
        gpio_set_direction(static_cast<gpio_num_t>(kAntDrivePin), GPIO_MODE_OUTPUT);
        gpio_set_direction(static_cast<gpio_num_t>(kAntSensePin), GPIO_MODE_INPUT);
        gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), 1); /* off until the module is up */
        ESP_LOGI(TAG, "antenna bias on GPIO%d (active low), short sense on GPIO%d",
                 kAntDrivePin, kAntSensePin);
    }

    if (xTaskCreate(&GpsTime::bridgeTaskEntry, "gpsbridge", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "no bridge task: u-center passthrough unavailable");
    }

    /* The decoder emits into this object; without the listener it counts frames
     * and dispatches none of them, which reads as a receiver sending nothing
     * useful. */
    m_decoder.listen(this);
    m_begun       = true;
    m_baud        = kBauds[m_preferredIndex];
    m_nextScanUs  = esp_timer_get_time() + kFirstRateUs;
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
     * most of the line, and switching that off is the fastest way to
     * make room for the messages this reference actually reads. Harmless noise
     * if there is nothing on the other end. */
    configureModule();
    restoreTimeMode();

    /* Power the antenna once the receiver has been configured. Off during the
     * baud scan so a wrong lead is not sitting powered through it. */
    vTaskDelay(pdMS_TO_TICKS(1500));
    applyBias(true);
    ESP_LOGI(TAG, "antenna bias on");

    for (;;) {
        /* While u-center is attached, the bridge task does the reading and taps
         * the bytes into the parser, so only the read is skipped here. The
         * periodic work below carries on: the clock must keep being disciplined
         * and /healthz must keep telling the truth while somebody is poking at
         * the receiver.
         *
         * Only the read is contended, not the parser -- two readers on one
         * stream is what the bridge exists to avoid. */
        if (m_bridgeActive) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            const int got = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(100));
            m_decoder.feed(std::span(buf, static_cast<std::size_t>(got > 0 ? got : 0)));
        }

        /* TIM-TP is only produced while the receiver is surveying.
         *
         * This cost a morning. In fixed (time) mode the M8T acknowledges the
         * rate, answers a poll with nothing, and emits no frame at all -- for
         * hours, across reboots, a power reset and a configuration default
         * clear. Back in survey mode it returns at 1 Hz immediately (42 replies
         * against 0). NAV-TIM-TP, the same bytes under class 0x01, is never
         * answered on this part at all, so it is polled too rather than assumed.
         *
         * A poll is asked for once a second because the rate is not honoured
         * even when the message does flow. Sixteen bytes back; the reply arrives
         * as a normal frame and lands in onNavTimTp. The consequence worth
         * knowing: the pulse statement and qErr are available in the mode that
         * is *worse* for timing, and absent in the mode that is better. */
        const int64_t nowUs = esp_timer_get_time();
        if (m_timTpCfg == 1 && nowUs - m_timTpPolledUs >= 1000000) {
            m_timTpPolledUs = nowUs;
            sendUbx(0x0D, 0x01, nullptr, 0);
            sendUbx(0x01, 0x27, nullptr, 0);
        }

        /* The scan used to run only until the first frame of the boot, so a rate
         * set from the bridge -- or by anything else that moved the module --
         * wedged the line until the next power cycle. Silence now re-arms it. A
         * session on tcp/3333 holds the port and must not have the rate moved
         * under it, so the bridge suppresses the scan for its duration. */
        if (!m_bridgeActive && m_frameSeen && m_lastFrameUs != 0 &&
            nowUs - m_lastFrameUs > kFrameSilenceUs) {
            m_frameSeen   = false;
            m_lastFrameUs = 0;
            ++m_rescans;
            /* The rate that was working a moment ago gets its full turn before
             * the scan moves. The deadline used to be whatever the last scan
             * left behind -- on a healthy box, fifteen seconds after boot -- so
             * the first silent five seconds walked straight off the right rate
             * and began configuring the module blind at the wrong ones. A
             * decoder caught mid-frame is the likelier cause than a rate that
             * moved, and starting it again is the whole cure for that. */
            m_decoder.reset();
            m_nextScanUs = nowUs + kFirstRateUs;
            ESP_LOGW(TAG, "no frames for %lld ms: rescanning the baud rate",
                     static_cast<long long>(kFrameSilenceUs / 1000));
        }
        /* Frame rate, evaluated once a second. Three poor seconds in a row is
         * treated as the same condition the silence test uses, so the scan
         * advances. Without it a mismatched rate that still parses the odd
         * frame sits there indefinitely. */
        if (m_secStartUs == 0) {
            m_secStartUs = nowUs;
        } else if (nowUs - m_secStartUs >= 1000000) {
            /* No exemption while the config burst is talking. A rate that works
             * answers the burst with ACKs, so an empty second during a config is
             * itself evidence; a rate that does not work sits there silently. The
             * exemption used to be unconditional, and it was what wedged the boot
             * at 9600: the burst runs every six seconds, so it kept zeroing the
             * counter and the poor-rate test could never reach its limit. */
            if (m_secFrames < kMinFramesPerSec) {
                if (m_poorSecs < 255) {
                    ++m_poorSecs;
                }
                if (m_poorSecs >= 6) {
                    m_frameSeen = false;
                    m_poorSecs  = 0;
                    m_decoder.reset();
                    ESP_LOGW(TAG, "only %u frames in 6 s at %u baud: rescanning",
                             static_cast<unsigned>(m_secFrames * 6), static_cast<unsigned>(m_baud));
                }
            } else {
                m_poorSecs = 0;
            }
            m_secFrames  = 0;
            m_secStartUs = nowUs;
        }

        if (!m_frameSeen && nowUs >= m_nextScanUs) {
            advanceBaud();
            m_nextScanUs = nowUs + (m_baudIndex == m_preferredIndex ? kFirstRateUs : kScanAfterUs);
        }
        if (nowUs >= m_nextCheckUs) {
            logLockChange();
            m_nextCheckUs = nowUs + 1000000;

            /* Nothing heard for a while: is the receiver transmitting at all?
             * The RX pin is sampled for transitions rather than the UART being
             * asked, because the question is below the driver -- a pin that
             * never moves is a wire or a dead port, a pin that moves while no
             * frame parses is a rate or framing fault, and the two need
             * different work. */
            if (!m_frameSeen) {
                int edges = 0;
                int last  = gpio_get_level(static_cast<gpio_num_t>(kRxPin));
                for (int i = 0; i < 400; ++i) {
                    const int now = gpio_get_level(static_cast<gpio_num_t>(kRxPin));
                    if (now != last) {
                        ++edges;
                        last = now;
                    }
                    esp_rom_delay_us(500);
                }
                ESP_LOGW(TAG, "GPIO%d idle probe at %u baud: %d transitions in 200 ms "
                              "(level %d)",
                         kRxPin, static_cast<unsigned>(m_baud), edges, last);
            }
        }
        if (m_baudReq != 0) {
            const uint32_t want   = m_baudReq;
            const bool     module = m_baudReqModule;
            m_baudReq             = 0;
            m_baudReqModule       = false;
            if (module) {
                /* Tell the receiver first, while the host is still listening at
                 * a rate the receiver knows. Then hold the old rate: if frames
                 * stop, this is written back as soon as anything is heard
                 * again, on whatever rate that turns out to be. */
                m_baudPrev            = m_baud;
                m_baudRevertTo        = m_baud;
                /* Four silence windows, not two. This module can take most of
                 * ten seconds to answer at a new rate -- measured: it accepted
                 * 115200, sent frames at 8.7 s, and had already been reverted at
                 * 8.0 s, so the change looked like a failure and undid itself. */
                m_baudRevertAtUs      = esp_timer_get_time() + 4 * kFrameSilenceUs;
                m_baudRevertPending   = true;
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                /* Acknowledged, not fire-and-forget: saving while the port
                 * change is still in flight captures the OLD rate, and the save
                 * itself reloads the port, so the change silently undoes. */
                /* This module does not ACK CFG-PRT -- it goes unanswered even
                 * when the change takes -- so silence has to mean "carry on".
                 * An explicit NAK is the only thing that stops the move, and
                 * the revert covers a change that simply does not work. */
                const int8_t prt = sendCfgPrt(want);
                if (prt == 0) {
                    ESP_LOGW(TAG, "CFG-PRT rejected: rate unchanged");
                } else {
                    /* Saved so the next power cycle comes back at the new rate
                     * rather than at 9600, which would undo the whole move. The
                     * pause lets the port change land before it is captured. */
                    uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                    esp_rom_delay_us(200000);
                    const int8_t saved = saveCfgIo();
                    ESP_LOGW(TAG, "port set to %u and %s", static_cast<unsigned>(want),
                             saved == 1 ? "saved"
                                        : saved == 0 ? "save REJECTED" : "save unanswered");
                }
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
            }
            uart_set_baudrate(kUart, want);
            m_baud      = want;
            m_frameSeen = false;
            ++m_rescans;
            if (module) {
                ESP_LOGW(TAG, "receiver and host set to %u baud; reverting to %u if frames do not "
                              "resume", static_cast<unsigned>(want), static_cast<unsigned>(m_baudPrev));
            } else {
                ESP_LOGW(TAG, "host side set to %u baud alone: the scan will find the receiver",
                         static_cast<unsigned>(want));
            }
        }

        /* The revert. Either frames came back and the old rate is written to the
         * receiver again, or the wait expires and the next rate the scan lands
         * on is treated as the one to write it at. */
        if (m_baudRevertPending && nowUs >= m_baudRevertAtUs) {
            m_baudRevertPending = false;
            m_baudRevertTo      = m_baudPrev;
            ESP_LOGW(TAG, "no frames at %u baud: reverting to %u",
                     static_cast<unsigned>(m_baud), static_cast<unsigned>(m_baudPrev));
        }
        if (m_baudRevertTo != 0 && m_frameSeen) {
            const uint32_t back = m_baudRevertTo;
            m_baudRevertTo      = 0;
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
            /* Saved as well, and for the same reason as the forward path: the
             * forward change already saved ITS rate, so reverting only in RAM
             * would leave the rate that failed in flash -- and the next power
             * cycle would come straight back to it. */
            if (sendCfgPrt(back) != 0) {
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                esp_rom_delay_us(200000);
                saveCfgIo();
            }
            uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
            uart_set_baudrate(kUart, back);
            m_baud                = back;
            m_baudRevertPending   = false;
            ESP_LOGW(TAG, "reverted to %u baud", static_cast<unsigned>(back));
        }
        /* A bias change or a restart asked for over HTTP. Serviced here rather
         * than in the handler so exactly one thread owns the UART and the
         * parser. */
        if (m_biasReq >= 0) {
            const bool on      = m_biasReq == 1;
            const bool wasOn   = m_antBiasOn;
            m_biasReq          = -1;
            /* Supply control bit on or off; short/open detection and
             * power-down-on-short stay set either way. */
            const uint8_t cfg[4] = {static_cast<uint8_t>(on ? 0x0F : 0x0E), 0, 0, 0};
            const int8_t  br     = sendAndWait(0x06, 0x13, cfg, sizeof(cfg));
            /* The pin is what actually does it; the CFG-ANT frame is kept
             * because it costs nothing and would start working on a module that
             * has a supervisor. */
            m_antLatched = false;
            m_shortSamples = 0;
            applyBias(on);
            gpio_set_level(static_cast<gpio_num_t>(kAntDrivePin), on ? 0 : 1);
            ESP_LOGW(TAG, "antenna bias %s on request (module said %s)", on ? "ON" : "OFF",
                     br == 1 ? "accepted" : br == 0 ? "refused" : "nothing");
            sendUbx(0x0A, 0x09, nullptr, 0); /* re-read the supervisor at once */

            /* Powering the preamp back up is not enough to get the sky back. The
             * supply was restored and the receiver sat at zero satellites, a
             * pegged AGC and a DONTKNOW antenna status for four minutes; the
             * same supply at the next boot works, so what is missing is the
             * module re-initialising its front end. A hardware reset gives it
             * that, and it keeps the almanac. Only on the off -> on edge: an
             * "on" while already on has nothing to re-initialise. */
            if (on && !wasOn) {
                m_resetReq   = true;
                m_resetHard  = true;
                m_resetCold  = false;
                m_resetPower = false;
                ESP_LOGW(TAG, "hardware reset queued to re-initialise the front end");
            }
        }
        if (m_gnssReq) {
            m_gnssReq  = false;
            m_gnssMask = m_gnssReqMask;
            /* The whole message set, not one frame. A mask change throws the
             * acquisition away, and configureModule() is the only thing that
             * knows every message the clock needs, so letting it run is
             * cheaper than a second path that could drift from it. */
            ESP_LOGW(TAG, "constellation mask 0x%02x requested", static_cast<unsigned>(m_gnssMask));
            configureModule();
        }
        if (m_svinCmd != 0) {
            const uint8_t cmd = m_svinCmd;
            m_svinCmd         = 0;
            /* The command names what only the task may do -- touch the UART --
             * so the payload is built here rather than by the handler.
             *
             * **CFG-TMODE2 (0x06 0x3D), not TMODE3 (0x06 0x71).** At protocol
             * 22 this is an M8, and TMODE3 is not its message. What gave the
             * mistake away is that TMODE3 was acknowledged and then read back
             * as all zeros: the same 28 bytes under TMODE2's layout start with
             * a byte that is `version` in one and `timeMode` in the other, so
             * TMODE3 was being told "disabled" each time. NAV-SVIN being NAKed
             * while TIM-SVIN (0x0D 0x04) answers says the same about the status
             * message -- this is a TIM firmware and the TIM dialect is the one
             * it speaks. */
            uint8_t  tm[28]   = {};
            uint32_t fixedAcc = 0;
            uint32_t minDur   = 0;
            uint32_t accLimit = 0;
            /* A survey reply is the only thing that fills the mean, so "fix" on a
             * box whose position came back from NVS has no mean at all. Locking
             * the (0,0,0) that is left points the receiver at the centre of the
             * Earth, and kilometres of position are microseconds of clock: the
             * box served 10 ms out until it was stopped. The held position is
             * the right thing to re-lock from in that case, and only when there
             * is neither is there nothing to lock. */
            const bool haveMean =
                !(m_svinMean[0] == 0 && m_svinMean[1] == 0 && m_svinMean[2] == 0);
            if (cmd == 3 && !haveMean && !m_fixedValid) {
                ESP_LOGW(TAG, "nothing to lock: no survey mean and no held position, "
                              "position left moving");
                m_svinCmd = 0;
            } else if (cmd == 1) {
                tm[0]    = 1; /* survey-in */
                minDur   = m_svinDurS;
                /* Millimetres, not millimetres squared. Both accuracy fields
                 * in this message are plain mm with no scaling (spec R27 p.267);
                 * everything the receiver says about its survey is a variance in
                 * mm squared, so the square has to come off here. It used to go
                 * on the wire squared: a 30 m limit was sent as 9e8 mm, i.e.
                 * 900 km, which no site can fail. */
                accLimit = m_svinAccMm;
            } else if (cmd == 3) {
                /* ECEF from the receiver's own mean: it reports ECEF and takes
                 * ECEF, so staying in it avoids a projection between two things
                 * that already agree. The accuracy is the mean's own standard
                 * deviation, again in mm out of a variance in mm squared. This
                 * is not decoration: the spec (R27 p.100) says the 3D accuracy
                 * here "directly affects the accuracy of the timepulse", so a
                 * variance of 1.4e7 sent raw told the receiver its position was
                 * good to 14 000 km and put the stated pulse accuracy near 47 ms
                 * -- which is the size of the errors that then went unnoticed. */
                tm[0]    = 2; /* fixed */
                /* Re-locking what is already held keeps its accuracy and its
                 * provenance; only a mean from this survey can supersede it. */
                fixedAcc = haveMean ? isqrt(m_svinAcc) : m_fixedAcc;
                for (int i = 0; i < 3; ++i) {
                    text::putLe(tm + 4 + 4 * i, haveMean ? m_svinMean[i] : m_fixedMean[i]);
                }
            }
            /* flags stay zero: ECEF, not LLA. */
            text::putLe(tm + 16, fixedAcc);
            text::putLe(tm + 20, minDur);
            text::putLe(tm + 24, accLimit);
            const int8_t r = sendAndWait(0x06, 0x3D, tm, sizeof(tm));
            if (r == 1) {
                m_tmode      = cmd == 1 ? 1 : cmd == 3 ? 2 : 0;
                m_holdWanted = cmd == 3;
                /* The two persisted forms, read once into plain values: the
                 * members are volatile, and a volatile reference is not one of
                 * the types std::formatter is specialised for. */
                if (cmd == 1) {
                    const auto hours = static_cast<uint32_t>(m_svinDurS / 3600);
                    const auto accMm = static_cast<uint32_t>(m_svinAccMm);
                    Settings::instance().setGpsSurvey(std::format("{},{}", hours, accMm));
                }
                if (cmd == 3) {
                    /* Set before the write, because the write is what has to
                     * carry it. */
                    if (haveMean) {
                        portENTER_CRITICAL(&m_mux);
                        m_holdValidated = m_svinValid;
                        portEXIT_CRITICAL(&m_mux);
                    }
                    Settings::instance().setGpsFixedPos(
                        std::format("{},{},{},{},{}", haveMean ? m_svinMean[0] : m_fixedMean[0],
                                    haveMean ? m_svinMean[1] : m_fixedMean[1],
                                    haveMean ? m_svinMean[2] : m_fixedMean[2],
                                    haveMean ? m_svinAcc : m_fixedAcc,
                                    m_holdValidated ? 1 : 0));
                }
                saveTimeMode();
                m_svinAtUs = 0;
                if (cmd == 1) {
                    ESP_LOGW(TAG, "survey-in started: %u s minimum, %u mm limit",
                             static_cast<unsigned>(minDur), static_cast<unsigned>(m_svinAccMm));
                    /* The limit is compared against the site's wander, so one
                     * below what the receiver is already scattering can never
                     * be met and the survey will run the duration out. Say so
                     * at the start rather than 24 hours later. */
                    if (m_svinAcc != 0 && isqrt(m_svinAcc) > accLimit) {
                        ESP_LOGW(TAG, "  the limit is tighter than this site scatters (%u mm "
                                      "now): expect it to run the full duration",
                                 static_cast<unsigned>(isqrt(m_svinAcc)));
                    }
                } else if (cmd == 3) {
                    ESP_LOGW(TAG, "position locked fixed at %d %d %d mm", m_svinMean[0],
                             m_svinMean[1], m_svinMean[2]);
                    /* A lock taken before the survey signed itself off is a
                     * position the receiver never agreed was good enough. It is
                     * usually fine -- metres are nanoseconds -- but it is also
                     * the state that produced the 18:43 free-run, and nothing
                     * said so at the time. Say it, and keep saying it in
                     * /gps/info through hold_validated. */
                    if (!m_holdValidated) {
                        ESP_LOGW(TAG, "  the survey had NOT met its own limit (%u mm needed, "
                                      "%u mm measured): locked unvalidated",
                                 static_cast<unsigned>(m_svinAccMm),
                                 static_cast<unsigned>(isqrt(m_svinAcc)));
                    }
                } else {
                    ESP_LOGW(TAG, "time mode cleared: back to a moving position");
                }
            } else {
                ESP_LOGW(TAG, "CFG-TMODE2 %s: time mode unchanged",
                         r == 0 ? "rejected" : "unanswered");
            }
        }
        /* A new cable or user delay re-sends the pulse frame and nothing else.
         * The survey and the mask are untouched: this is a property of the lead,
         * and restarting a day's survey to change it would be a poor trade.
         *
         * The rate window is thrown away, though, and that is the point. A delay
         * is a step in the phase: inside a 300 s fit a 10 ms step reads as
         * 33 ppm of rate error, so the servo trims it back out over the window
         * and the correction quietly undoes itself. Measured: a 10 ms delay
         * moved the pulse for a second or two and left the served time where it
         * was. With the window cleared, the step is geometry again. */
        if (m_tp5Cmd) {
            m_tp5Cmd = false;
            const int8_t r = sendTp5();
            m_tp5Acked     = r == 1;
            m_tp5Naked     = r == 0;
            if (r == 1) {
                m_rateCount    = 0;
                m_rateNext     = 0;
                m_rateLogged   = false;
                m_lastRateOffsetSet = false;
            } else {
                ESP_LOGW(TAG, "CFG-TP5 delay change %s", r == 0 ? "rejected" : "unanswered");
            }
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
                const uint8_t cfg[4] = {0x0B, 0x00, 20, 27};
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
            if (m_bridgeActive && !m_reconfigForce) {
                /* A client is configuring the receiver. Stamping eleven CFG-MSG
                 * frames of our own over the top of whatever it set is how a
                 * session ends up fighting the firmware for the module. Stand
                 * aside -- detaching re-arms this anyway. */
                m_reconfigAtUs = 0;
            } else {
                const bool forced = m_reconfigForce;
                m_reconfigAtUs    = 0;
                m_reconfigForce   = false;
                m_skipAdopt       = forced;
                configureModule();
                m_skipAdopt = false;
                if (forced) {
                    /* A reset or a clear-to-defaults took the time mode with it.
                     * Put a known position hold back, and do not ask the module
                     * what its mode is first: it is still rebooting and would
                     * answer with the mode the reset left behind. */
                    reassertFix();
                }
            }
        }
        /* The hold is asserted again if the receiver is not holding and the
         * settings still want it. The stored mode is read rather than a member,
         * because "stop" puts the module back to moving and only the settings
         * know that has happened -- without this test the retry would put a hold
         * back under a box that had been asked for a moving solution. A survey
         * in progress wins for the same reason: TMODE2 fixed would end it.
         *
         * It also waits for a fix, satellites and a live edge, because the frame
         * restarts the pulse generator and there is nothing to restart before
         * the receiver has begun: an unfixed module given TMODE2 spends the
         * boot's first edges and can hand the clock to the fallback. Waiting
         * does not cost a retry, so the deadline is simply moved on. */
        if (m_holdAssertAtUs != 0 && m_tmode == 2 && !m_svinActive) {
            /* The receiver is already holding -- the poll said so at boot, or a
             * previous attempt took -- so nothing is outstanding. Left armed the
             * deadline reads as overdue forever in /gps/info. */
            m_holdAssertAtUs = 0;
        } else if (m_holdWanted && m_tmode != 2 && m_holdAssertAtUs != 0 &&
                   !m_svinActive && Settings::instance().gpsTimeMode() == "fixed") {
            const bool ready = m_fixType >= 2 && m_satCount >= 4 && m_lastEdgeUs > 0 &&
                               nowUs - m_lastEdgeUs < 3000000;
            if (nowUs < m_holdAssertAtUs) {
                /* Not due yet. */
            } else if (!ready) {
                m_holdAssertAtUs = nowUs + kHoldRetryUs;
            } else if (sendFixedPosition()) {
                m_holdAssertAtUs = 0;
                m_holdRetryUs    = 0;
                ESP_LOGW(TAG, "position hold asserted at the receiver");
            } else {
                m_holdRetryUs    = std::min(m_holdRetryUs * 2, kHoldRetryMaxUs);
                m_holdAssertAtUs = nowUs + m_holdRetryUs;
                ESP_LOGW(TAG, "receiver refused the position hold: next try in %lld s",
                         static_cast<long long>(m_holdRetryUs / 1000000));
            }
        }
        if (m_defaultsReq) {
            m_defaultsReq = false;
            /* CFG-CFG with clearMask 0x1F: put every configuration block back to
             * its default in RAM. Unlike the resets, this touches the settings
             * themselves -- constellation selection, message rates, the lot --
             * and leaves the module's saved flash alone, so u-center still gets
             * its defaults back at the next power-up. The last software lever
             * when the receiver is deaf and its configuration is the suspect. */
            const uint8_t cfg[12] = {0x1F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            ESP_LOGW(TAG, "receiver configuration cleared to defaults");
            sendUbx(0x06, 0x09, cfg, sizeof(cfg));
            /* The clear takes CFG-PRT back to the module's 9600 default with it.
             * Left alone the scan adopts 9600, then walks down looking for a
             * line that will carry the message set, and at 4800 the frames are
             * late enough to spike an edge a minute -- 109 of them and 1.5 ms of
             * offset before this was spotted. Put the working rate straight back
             * rather than let the scan find its way there. */
            requestBaud(m_baud, true);
            /* Forced: the clear took every rate to zero, so a client attached
             * right now has no stream either. */
            m_reconfigForce = true;
            m_reconfigAtUs  = esp_timer_get_time() + 3000000;
        }
        if (m_resetReq) {
            m_resetReq = false;
            /* CFG-RST. Hot by default: restart the navigation engine and keep
             * the almanac, which is the cheap way back from a state a module will
             * not leave on its own. Cold when asked, which throws the almanac
             * away -- the thing to try when a hot start changed nothing and the
             * receiver's own stored state is suspect. */
            const bool    cold  = m_resetCold;
            const bool    hard  = m_resetHard;
            const bool    power = m_resetPower;
            m_resetHard         = false;
            m_resetPower        = false;
            /* resetMode: 0x00 is a hardware reset, 0x02 a controlled software
             * one (GNSS only). A hardware reset also drops the module's whole
             * runtime configuration back to its defaults, so the message set is
             * re-sent a few seconds later -- otherwise it comes back speaking
             * NMEA at 9600 and the parser has nothing to read. */
            /* 0x04 is "hardware reset after shutdown", the nearest thing to
             * pulling the module's supply: it shuts the receiver down first, so
             * the RF and baseband are re-initialised rather than restarted, and
             * every RAM configuration written since power-up goes with it. The
             * saved configuration in the module's own flash comes back, which is
             * the point when the RAM state is what is suspect. */
            const uint8_t mode = power ? 0x04 : hard ? 0x00 : 0x02;
            const uint8_t rst[4] = {static_cast<uint8_t>(cold ? 0xFF : 0x00),
                                    static_cast<uint8_t>(cold ? 0xFF : 0x00), mode, 0x00};
            ESP_LOGW(TAG, "receiver %s%s start requested", power ? "POWER-CYCLE " : hard ? "HARDWARE " : "",
                     cold ? "cold" : "hot");
            if (hard || power) {
                /* Forced for the same reason as the clear above: the module
                 * comes back at its own defaults, in NMEA, at 9600, and a
                 * client attached through the reset needs a UBX stream just as
                 * much as the clock does. */
                m_reconfigForce = true;
                m_reconfigAtUs  = esp_timer_get_time() + 3000000;
            }
            sendUbx(0x06, 0x04, rst, sizeof(rst));
        }

        /* The short sense. Sample four times a second; three consecutive reads
         * in the shorted state cut the bias and latch, because the point of the
         * protection is to act faster than a person. */
        {
            static int64_t nextSenseUs = 0;
            if (nowUs >= nextSenseUs) {
                nextSenseUs = nowUs + 250000;
                const int level = gpio_get_level(static_cast<gpio_num_t>(kAntSensePin));
                if (level != m_senseLogged) {
                    m_senseLogged = level;
                    ESP_LOGI(TAG, "antenna sense GPIO%d reads %d", kAntSensePin, level);
                }
                /* SCS1_N is active low: a short pulls it down. */
                if (m_antBiasOn && !m_antLatched && level == 0) {
                    if (++m_shortSamples >= 3) {
                        m_antShort   = true;
                        m_antLatched = true;
                        applyBias(false);
                        ESP_LOGW(TAG, "antenna SHORT: bias cut and latched off");
                    }
                } else if (level != 0) {
                    m_shortSamples = 0;
                    m_antShort     = false;
                }
            }
        }

        /* Antenna supervisor, every two seconds. Polled rather than enabled at
         * a rate so a bench session that never looks at it costs nothing. */
        if (!m_bridgeActive && nowUs >= m_nextHwUs) {
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
        if (nowUs - m_lastSweepUs > 60000000LL) {
            m_lastSweepUs = nowUs;
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
            ESP_LOGW(TAG, "no UBX frames at any baud rate: check module TX -> GPIO%d "
                          "(esp RX) and PPS -> GPIO%d", kRxPin, static_cast<int>(kPpsPin));
            m_noDataLogged = true;
        }
    } else {
        m_baudIndex = (m_baudIndex + 1) % kCount;
        if (!m_noDataLogged) {
            ESP_LOGI(TAG, "no UBX frames yet, trying %u baud",
                     static_cast<unsigned>(kBauds[m_baudIndex]));
        }
    }

    m_baud = kBauds[m_baudIndex];
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
    const int32_t userNs = static_cast<int32_t>(
        userAsked > 10000000L ? 10000000L : userAsked < -10000000L ? -10000000L : userAsked);
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
        const int8_t rr = sendAndWait(0x06, 0x08, kRate, sizeof(kRate));
        m_rateCfg = rr == 1 ? 1 : rr == 0 ? 2 : 0;
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
        nav5[2]  = 2;                   /* dynModel: stationary */
        nav5[30] = 3;                   /* utcStandard: UTC(USNO) from GPS */
        const int8_t nr = sendAndWait(0x06, 0x24, nav5, sizeof(nav5));
        m_nav5Cfg = nr == 1 ? 1 : nr == 0 ? 2 : 0;
        if (nr == 1) {
            ESP_LOGI(TAG, "CFG-NAV5 accepted: stationary platform, UTC from GPS");
        } else {
            ESP_LOGW(TAG, "CFG-NAV5 %s: dynamic model and UTC standard are defaults",
                     nr == 0 ? "rejected" : "unanswered");
        }
    }
    int8_t r = sendTp5();
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
    r = sendAndWait(0x06, 0x13, kCfgAnt, sizeof(kCfgAnt));
    m_antCfgAcked = r == 1;
    m_antCfgNaked = r == 0;
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
        struct GnssBlock
        {
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
        constexpr int    kCount = static_cast<int>(sizeof(kBlocks) / sizeof(kBlocks[0]));
        uint8_t          cfg[4 + 8 * kCount] = {0x00, 32, 32, kCount};
        const uint32_t   mask = m_gnssMask;
        for (int i = 0; i < kCount; ++i) {
            const uint32_t flags = kBlocks[i].flags | (((mask >> kBlocks[i].id) & 1u) != 0 ? 1u : 0u);
            uint8_t       *b     = cfg + 4 + 8 * i;
            b[0]                 = kBlocks[i].id;
            b[1]                 = kBlocks[i].res;
            b[2]                 = kBlocks[i].max;
            b[3]                 = 0;
            text::putLe(b + 4, flags);
        }
        r = sendAndWait(0x06, 0x3E, cfg, sizeof(cfg));
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
    m_tosCfg = tos == 1 ? 1 : tos == 0 ? 2 : 0;
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
        r = sendAndWait(0x06, 0x01, kNavSat, sizeof(kNavSat));
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
            const char   *name;
            uint8_t       cls;
            uint8_t       id;
            uint8_t       div;
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
        r = sendAndWait(0x06, 0x01, kTimSvin, sizeof(kTimSvin));
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
            vTaskDelay(pdMS_TO_TICKS(2));
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
    m_lastFrameUs = esp_timer_get_time();
    ++m_secFrames;
    m_recentFrames[m_recentAt++ % 16] = static_cast<uint16_t>((cls << 8) | id);
    if (!m_frameSeen) {
        m_frameSeen = true;
        m_scanSteps = 0;
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

void GpsTime::adoptFixedPosition(long long x, long long y, long long z, unsigned long acc,
                                 bool validated)
{
    for (int i = 0; i < 3; ++i) {
        m_fixedMean[i] = static_cast<int32_t>(i == 0 ? x : i == 1 ? y : z);
    }
    m_fixedAcc      = acc;
    m_fixedValid    = true;
    m_holdWanted    = true;
    m_holdValidated = validated;
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
    text::Buf<241> hex = {};
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
    text::Buf<241> hex = {};
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
    m_gDop        = dop(4);
    m_pDop        = dop(6);
    m_vDop        = dop(10);
    m_hDop        = dop(12);
    m_dopAtUs     = nowUs;
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
        worstJam = jam > worstJam ? jam : worstJam;
        worstAnt = ant > worstAnt ? ant : worstAnt;
        minAgc   = agc < minAgc ? agc : minAgc;
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

    uint16_t sum = 0;
    uint8_t  max = 0;
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
            Sat &s = m_sats[kept++];
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
            s.prRes         = (pr == INT16_MIN) ? 0 : pr;
            s.prResValid    = pr != INT16_MIN;
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
        ESP_LOGW(TAG, "survey-in finished after %u s at %u mm: "
                      "POST /gps/survey {\"action\":\"fix\"} to lock it",
                 static_cast<unsigned>(dur), static_cast<unsigned>(isqrt(var)));
    }
    wasActive = m_svinActive;
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

void GpsTime::onNavPvt(const ubx::Frame &f)
{
    /* The first 24 bytes carry everything used here; the decoder has already
     * held the frame to its full 92. */
    const uint8_t valid = f.u8(11);
    time_t        stamp = 0;
    /* validDate | validTime. u-blox only sets both once the receiver has an
     * almanac and a position estimate, which is exactly the point at which its
     * UTC can be believed -- before that the fields hold a 1980 default. */
    const bool    haveTime = (valid & 0x03) == 0x03 &&
                          civilToEpoch(f.u2(4), f.u8(6), f.u8(7), f.u8(8), f.u8(9), f.u8(10),
                                       stamp);
    const int32_t nano   = f.i4(16); /* offset into the second, ~0 at 1 Hz */
    const uint8_t fix    = f.u8(20);
    const uint8_t sats   = f.u8(23);

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
    const int16_t cable = static_cast<int16_t>(f.u2(4));
    const int32_t user  = f.i4(24);
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
    const uint32_t flags    = f.u4(4);
    const int32_t  utcOff   = f.i4(16);
    const uint32_t utcUnc   = f.u4(20);

    portENTER_CRITICAL(&m_mux);
    m_tosFlags     = flags;
    m_tosUtcOffset = utcOff;
    m_tosUtcUnc    = utcUnc;
    portEXIT_CRITICAL(&m_mux);

    if (!m_tosLogged) {
        m_tosLogged = true;
        ESP_LOGW(TAG, "TIM-TOS flowing: pulse %s, generation %s, %.3f us off the UTC second "
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
    const int8_t  current = static_cast<int8_t>(f.u8(9));
    const int8_t  change  = static_cast<int8_t>(f.u8(11));
    const int32_t inS     = f.i4(12);
    const uint8_t valid   = f.u8(23);
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
    const uint32_t towSubMs  = f.u4(4);
    const int32_t  qErrPs    = f.i4(8);
    const uint8_t  flags     = f.u8(14);

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
    const int32_t qErrNs = static_cast<int32_t>(
        std::clamp<int64_t>(qErrPs / 1000, -kMaxQErrNs, kMaxQErrNs));

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
                     gnssBase ? "GNSS" : "UTC");
        }
    } else if (!m_pulseOnSecondSeen) {
        ESP_LOGI(TAG, "timepulse back on the UTC second");
    }
    m_pulseOnSecondSeen = onSecond;
}

void GpsTime::anchor(time_t second)
{
    int64_t  ppsMonoUs = 0;
    int64_t  ppsMonoNs = 0;
    int64_t  ppsPtpNs  = 0;
    uint32_t ppsCount  = 0;

    portENTER_CRITICAL(&m_mux);
    ppsMonoUs = m_ppsMonoUs;
    ppsMonoNs = m_ppsMonoNs;
    ppsPtpNs  = m_ppsPtpNs;
    ppsCount  = m_ppsCount;
    /* The edge's own quantisation, from the TIM-TP that described this pulse.
     * Convention is corrected = captured - qErr (gnssdo, qerr.rs); the sawtooth
     * is the receiver only being able to place an edge on its own clock's grid.
     * TIM-TP describes the *next* pulse, so a correction received in the last
     * 1.2 s belongs to this edge; older than that and it describes the one
     * before, younger than -50 ms and it arrived after this one and belongs to
     * the next.
     *
     * Requires the UTC base with UTC available. gpsd also withholds the figure
     * while raim is active; this module reports raim active continuously
     * (flags 0x0b), so following that would disable the correction outright --
     * and raim says nothing about the pulse generator's grid. */
    if (m_qErrValid && (m_timTpFlags & 0x03) == 0x03 && m_qErrAtUs != 0) {
        const int64_t ageUs = ppsMonoUs - m_qErrAtUs;
        if (ageUs >= -50000 && ageUs <= 1200000) {
            /* Nanoseconds off a microsecond stamp: without the division a
             * 10 ns sawtooth moved the edge 10 us, and the clamp allowed 5 ms. */
            ppsMonoUs -= m_qErrNs / 1000;
            if (ppsMonoNs != 0) {
                ppsMonoNs -= m_qErrNs;
            }
            /* The same correction in the counter the reply is stamped in, or
             * the two anchors would differ by the sawtooth. */
            if (ppsPtpNs != 0) {
                ppsPtpNs -= m_qErrNs;
            }
        }
    }
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

    /* Accepted, so this is a real measurement of the sentence latency rather
     * than of a sentence that arrived too late to trust. Recorded here and not
     * in the ISR: it is a property of the pairing, and the pairing is what the
     * window above bounds. */
    {
        const int64_t atUs = esp_timer_get_time();
        portENTER_CRITICAL(&m_mux);
        m_pairAgeUs    = ageUs;
        m_pairAgeMaxUs = ageUs > m_pairAgeMaxUs ? ageUs : m_pairAgeMaxUs;
        m_pairAgeAtUs  = atUs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* What the servo predicted for this instant before it knew. Across an
     * ordinary second this is a check on the rate; across a holdover it is the
     * error a client was served while the edge was missing, and the only
     * honest way to quote what a holdover was worth. */
    int64_t predictedNs = 0;
    int64_t gapUs       = 0;
    bool    hadAnchor   = false;
    {
        portENTER_CRITICAL(&m_mux);
        if (m_anchorValid && ppsMonoUs > m_anchorMonoUs) {
            hadAnchor   = true;
            gapUs       = ppsMonoUs - m_anchorMonoUs;
            predictedNs = m_anchorUtcNs + gapUs * 1000 - gapUs * m_ratePpb / 1000000;
        }
        portEXIT_CRITICAL(&m_mux);
    }

    /* Classify the edge before it is allowed to move anything, on two
     * independent questions that both have to be answered yes:
     *
     *  - regularity: did about a second pass since the last edge? The ISR has
     *    already refused anything under half a second, so what is left is a
     *    pulse that was lost (1.4 s) or one emitted twice (0.7 s);
     *  - continuity: does the module's own date agree with the time that
     *    actually passed? A receiver that reconfigured its pulse, or that has
     *    just been stepped, emits an edge a second early or late while the
     *    spacing looks perfect -- and a sample whose second is wrong is a whole
     *    second wrong, not a little wrong.
     *
     * An edge failing either is counted and dropped. It does not become the
     * anchor, does not enter the fit and does not steer: holding the anchor
     * still lets the servo's own rate carry this second, which is what the fit
     * is for, and costs one second of phase correction instead of poisoning
     * five minutes of rate. */
    /* Spacing is measured edge to edge, not from the anchor. The two differ
     * exactly when it matters: a refused edge leaves the anchor behind, so the
     * next gap measured from the anchor is two seconds while only one second of
     * edges went by -- and testing against that refuses every edge after the
     * first and never recovers. */
    const int64_t edgeGapUs = m_lastEdgeMonoSet ? ppsMonoUs - m_lastEdgeMonoUs : 0;
    const bool regular = !m_lastEdgeMonoSet || (edgeGapUs >= kEdgeRegularMinUs &&
                                                edgeGapUs <= kEdgeRegularMaxUs);
    /* Continuity is tested against the seconds that actually elapsed rather than
     * against a flat one, or a returning edge after an outage would always fail
     * it and the anchor would never come back. Half a second of rounding, and
     * nothing beyond what the pulse allowed: a date that jumps further than the
     * edges did is the pairing being wrong. */
    const int64_t expectedS = m_lastEdgeMonoSet ? (edgeGapUs + 500000) / 1000000 : 1;
    const bool advance = m_lastEdgeSecondSet &&
                         static_cast<int64_t>(second) ==
                             static_cast<int64_t>(m_lastEdgeSecond) + expectedS;
    /* An edge that ends a holdover is outside a second's spacing by definition,
     * and has to be usable: it is the one the holdover error is measured
     * against. From kRateGapNs up, the fit is dropped by the gap path in
     * pushRateSample() anyway, so admitting it costs the fit nothing. Between
     * one missed edge and that, the spanning sample would still be fitted and
     * the edge is refused instead. */
    const bool resumed = m_lastEdgeMonoSet && edgeGapUs >= kRateGapNs / 1000;
    const bool trusted = advance && (regular || resumed);

    /* A pulse arriving a millisecond or more from where the fit and the anchor
     * put it is being corrupted rather than merely late. Measured against the
     * prediction in the monotonic domain, because that is the domain the servo
     * works in -- against the realtime clock the figure would carry whatever
     * slew is in flight. Only for ordinary edges: across a holdover the
     * prediction has been running on the crystal for seconds and its error is
     * expected, so testing there would count every return from a gap. */
    if (trusted && hadAnchor && gapUs < kLockWindowUs &&
        (static_cast<int64_t>(second) * 1000000000 - predictedNs > kEdgeSpikeNs ||
         predictedNs - static_cast<int64_t>(second) * 1000000000 > kEdgeSpikeNs)) {
        portENTER_CRITICAL(&m_mux);
        ++m_edgeSpikes;
        portEXIT_CRITICAL(&m_mux);
    }

    m_lastEdgeSecond    = second;
    m_lastEdgeSecondSet = true;
    m_lastEdgeMonoUs    = ppsMonoUs;
    m_lastEdgeMonoSet   = true;

    portENTER_CRITICAL(&m_mux);
    m_lastEdgeTrusted = trusted;
    uint32_t streak   = trusted ? 0 : ++m_edgeRejects;
    if (trusted) {
        m_edgeRejects = 0;
    }
    portEXIT_CRITICAL(&m_mux);

    if (!trusted) {
        /* One refused edge is one second of unconsumed correction. Forty-five
         * of them in a row is not a coincidence: either the pulse or the
         * module's idea of the second is wrong, and a fit built across that is
         * worth less than the minute it takes to build a new one. */
        if (streak == kRejectStreakReacquire) {
            resetRateWindow();
            ESP_LOGW(TAG, "servo: %u edges in a row refused (spacing or second continuity) "
                          "- refitting the rate",
                     static_cast<unsigned>(streak));
        }
        return;
    }

    portENTER_CRITICAL(&m_mux);
    m_anchorUtcNs  = static_cast<int64_t>(second) * 1000000000;
    m_anchorMonoUs = ppsMonoUs;
    /* The same anchor in the counter a reply's arrival is stamped by, so the
     * served time and the stamp need no conversion. Zero when the PHC is not
     * running, and now() then falls back to the monotonic clock. */
    m_anchorPhcNs  = ppsPtpNs;
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
    pushRateSample(ppsMonoNs != 0 ? ppsMonoNs : ppsMonoUs * 1000,
                   static_cast<int64_t>(second) * 1000000000);

    steer();
}

int64_t GpsTime::rootDelayNs() const
{
    /* Zero, because there is no round trip to measure. The reference is a
     * receiver on the board, so the field that means "path to the reference" has
     * nothing to say, and ntpd reports 0 for a local refclock for exactly that
     * reason.
     *
     * What the 283 us once published here actually is: our own stamping error,
     * which belongs in dispersion. Keeping it in the delay is not neutral --
     * a client halves the delay into its bound, so a local error advertised
     * there is charged at half rate while the client *also* measures a real
     * delay to us that already includes the processing. It has been charged
     * twice. */
    return 0;
}

int64_t GpsTime::sampleErrorNs() const
{
    /* The fixed part of the budget: the interrupt latency of the pulse capture,
     * and half the receiver's stated pulse quantisation -- half because the
     * sawtooth is uniform over the step and a bound is about the worst case
     * from the middle of it. The moving part, the drift of the served time, is
     * measured on the wire and added in dispersionNs(). */
    return kIsrLatencyNs + (m_qErrValid ? std::abs(m_qErrNs) / 2 : 0);
}

int64_t GpsTime::unobservedNs() const
{
    /* A pulse nobody is watching. While TIM-TP is configured and has stopped
     * arriving, the position of the edge against its own second is unobserved
     * -- the servo cannot see it either, because the anchor is taken from the
     * same edge and moves with it. Unobserved is not known, so the stated
     * uncertainty grows with how long it has been silent.
     *
     * 10 us a second covers the measured growth (4.2 us/s to 133 ms) with
     * margin, and the 30 s grace keeps a mode change from widening it. Capped
     * at 200 ms, six times the worst error seen and still finite. */
    if (m_timTpCfg != 1) {
        return 0;
    }
    /* In fixed mode the receiver stops sending TIM-TP: it is a survey-time
     * message, and a held position has no survey to describe. The edge itself is
     * still captured and still trusted, so what is missing is only the
     * receiver's statement of where its own pulse sits -- bounded by the
     * generator's step, not unknowable. Escalating here demoted a box whose
     * clock was being disciplined by the same receiver, and since the term only
     * grows and never comes back the box never promoted again: the upstream kept
     * the clock while 40 µs of honest dispersion had been the truth. The
     * quantisation clamp is the bound to state instead, and only while the edge
     * is trusted -- an untrusted edge below returns to the escalation. */
    /* The edge has to be current as well as trusted. m_lastEdgeTrusted is only
     * written when an edge is processed, so it keeps its last value across an
     * outage -- and a free-running box claiming the quantisation clamp while
     * nothing is arriving is the underclaim the escalation was added for. */
    if (m_tmode == 2 && m_lastEdgeTrusted && m_lastEdgeUs > 0 &&
        esp_timer_get_time() - m_lastEdgeUs < 2000000) {
        return kMaxQErrNs;
    }
    const int64_t silentS = (esp_timer_get_time() - m_qErrAtUs) / 1000000;
    if (silentS <= 30) {
        return 0;
    }
    return std::min<int64_t>((silentS - 30) * 10000, 200000000);
}

int64_t GpsTime::dispersionNs(int64_t ageUs, int64_t unobservedNs) const
{
    const int64_t ageS = ageUs > 0 ? ageUs / 1000000 : 0;
    return sampleErrorNs() + ServePath::instance().boundNs() + m_jitterNs +
           ageS * kWanderNsPerS + unobservedNs;
}

void GpsTime::recordSample(uint32_t epoch, int64_t offsetNs)
{
    /* Missing epochs first. A holdover produces no call at all until the next
     * edge, so the holes are only visible here, and one record per skipped
     * second is what lets a client tell "nothing was measured" from "nothing
     * arrived". */
    if (m_lastSampleSet && epoch > m_lastSampleEpoch + 1) {
        uint32_t missing = epoch - m_lastSampleEpoch - 1;
        if (missing > kSampleRing) {
            missing = kSampleRing;
        }
        for (uint32_t i = 0; i < missing; ++i) {
            portENTER_CRITICAL(&m_mux);
            Sample &slot = m_samples[m_sampleSeq % kSampleRing];
            slot.seq     = m_sampleSeq;
            slot.epoch   = epoch - missing + i;
            slot.kind    = 1;
            slot.offsetNs = 0;
            slot.ratePpb  = m_ratePpb;
            /* The row has no measurement, but the seconds it covers were still
             * served: their bound is the one in force at the time, which is the
             * current budget plus the wander over the age of that particular
             * second. Leaving it zero said "no uncertainty here" about seconds
             * that had nothing but the crystal behind them. */
            const int64_t ageUs = static_cast<int64_t>(epoch) - slot.epoch;
            slot.dispersionNs =
                static_cast<uint32_t>(clamp(dispersionNs(ageUs * 1000000, unobservedNs()), 0,
                                            UINT32_MAX));
            slot.jitterNs     = 0;
            slot.flags        = 0;
            slot.sats         = 0;
            slot.fix          = 0;
            ++m_sampleSeq;
            if (m_sampleSeq - m_sampleOldest > kSampleRing) {
                m_sampleOldest = m_sampleSeq - kSampleRing;
            }
            portEXIT_CRITICAL(&m_mux);
        }
    }

    portENTER_CRITICAL(&m_mux);
    Sample &slot      = m_samples[m_sampleSeq % kSampleRing];
    slot.seq          = m_sampleSeq;
    slot.epoch        = epoch;
    slot.kind         = 0;
    slot.offsetNs     = static_cast<int32_t>(clamp(offsetNs, INT32_MIN, INT32_MAX));
    slot.ratePpb      = m_ratePpb;
    /* The bound this reference states about the packet it would have built for
     * this second, from the same terms status() uses. Recorded at the instant
     * of the sample, so the age term is zero and the wander arrives later, in
     * the rows a holdover generates. */
    slot.dispersionNs = static_cast<uint32_t>(clamp(dispersionNs(0, unobservedNs()), 0,
                                                    UINT32_MAX));
    slot.jitterNs     = static_cast<uint16_t>(clamp(m_jitterNs, 0, UINT16_MAX));
    slot.flags        = m_timTpFlags;
    slot.sats         = m_satCount;
    slot.fix          = m_fixType;
    ++m_sampleSeq;
    if (m_sampleSeq - m_sampleOldest > kSampleRing) {
        m_sampleOldest = m_sampleSeq - kSampleRing;
    }
    m_lastSampleEpoch = epoch;
    m_lastSampleSet   = true;
    portEXIT_CRITICAL(&m_mux);
}

size_t GpsTime::drainSamples(uint32_t since, Sample *out, size_t max, uint32_t &first,
                             uint32_t &last, uint32_t &lost) const
{
    portENTER_CRITICAL(&m_mux);
    const uint32_t newest = m_sampleSeq; /* one past the last written */
    first                 = m_sampleOldest;
    last                  = newest == 0 ? 0 : newest - 1;
    if (since == 0 || since < m_sampleOldest) {
        lost  = since == 0 ? 0 : m_sampleOldest - since;
        since = m_sampleOldest;
    } else {
        lost = 0;
    }
    size_t copied = 0;
    for (uint32_t seq = since; seq < newest && copied < max; ++seq) {
        out[copied++] = m_samples[seq % kSampleRing];
    }
    portEXIT_CRITICAL(&m_mux);
    return copied;
}

void GpsTime::resetRateWindow()
{
    m_rateCount = 0;
    m_rateNext  = 0;
    /* The first sample after this seeds the window instead of being differenced
     * against one from the window that was just thrown away. */
    m_lastRateOffsetSet = false;
    /* The published deviation and sample count have to go with the window they
     * were taken over, or the panel keeps quoting both for a sequence that no
     * longer exists. rateSamples is normally only refreshed inside fitRate(),
     * which early-returns below its minimum, so left alone it would sit at the
     * old number for a minute. */
    portENTER_CRITICAL(&m_mux);
    m_rateSamples = 0;
    portEXIT_CRITICAL(&m_mux);
}

void GpsTime::pushRateSample(int64_t monoNs, int64_t utcNs)
{
    const int64_t offsetNs = utcNs - monoNs;

    /* A gap in the reference: neither the fit below nor a second difference
     * wants a sample with a hole in front of it. Drop the window, drop the
     * spanning sample, and let the first post-gap second seed it again. */
    if (m_lastRateMonoSet && monoNs - m_lastRateMonoNs > kRateGapNs) {
        const int64_t gapMs = (monoNs - m_lastRateMonoNs) / 1000000;
        resetRateWindow();
        m_lastRateMonoNs  = monoNs;
        m_lastRateMonoSet = true;
        ESP_LOGW(TAG, "servo: %lld s without a reference, dropping the rate window",
                 static_cast<long long>(gapMs / 1000));
        return;
    }

    if (m_lastRateOffsetSet &&
        (offsetNs - m_lastRateOffsetNs > kRateResetNs || m_lastRateOffsetNs - offsetNs > kRateResetNs)) {
        /* The module moved the clock under us, or a pairing landed a second out.
         * A slope fitted across that step is about nothing, so the window starts
         * again -- and a second out is worth saying out loud, because it is the
         * one failure that would otherwise look like a healthy clock. */
        const int64_t stepMs = (offsetNs - m_lastRateOffsetNs) / 1000000;
        resetRateWindow();
        ESP_LOGW(TAG, "servo: %+lld ms step in the reference, refitting the rate",
                 static_cast<long long>(stepMs));
    }
    m_lastRateOffsetNs  = offsetNs;
    m_lastRateOffsetSet = true;
    m_lastRateMonoNs    = monoNs;
    m_lastRateMonoSet   = true;

    m_rate[m_rateNext] = RateSample{monoNs, utcNs};
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
    const int64_t t0 = esp_timer_get_time();

    /* Least squares of UTC nanoseconds against the edge stamp in nanoseconds,
     * both
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
        sumX += m_rate[i].monoNs - base.monoNs;
        sumY += m_rate[i].utcNs - base.utcNs;
    }
    const double meanX = static_cast<double>(sumX) / m_rateCount;
    const double meanY = static_cast<double>(sumY) / m_rateCount;

    double sxy = 0.0;
    double sxx = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoNs - base.monoNs) - meanX;
        double       dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY;
        sxy += dx * dy;
        sxx += dx * dx;
    }
    if (sxx <= 0.0) {
        return;
    }

    /* Both axes are nanoseconds now, so a perfect counter gives a slope of
     * exactly 1 and the rate error is how far from 1 it sits, in parts per
     * billion. */
    const double  slope = sxy / sxx;
    int64_t       ppb   = static_cast<int64_t>(std::llround((1.0 - slope) * 1e9));
    bool          clamped = false;
    if (ppb > kRateLimitPpb) {
        ppb     = kRateLimitPpb;
        clamped = true;
    } else if (ppb < -kRateLimitPpb) {
        ppb     = -kRateLimitPpb;
        clamped = true;
    }

    portENTER_CRITICAL(&m_mux);
    if (clamped) {
        ++m_servoClamps;
    }
    m_ratePpb     = static_cast<int32_t>(ppb);
    m_rateSamples = static_cast<uint16_t>(m_rateCount);
    portEXIT_CRITICAL(&m_mux);

    /* Residual: the thing that says the samples are actually samples of a
     * clock and not of the receiver's mood -- and, while the servo is running,
     * the bound this reference states. Each edge's distance from the line
     * through all of them is the stamping jitter, measured, and RMS is the
     * figure a root dispersion wants. TIM-TP would describe the same jitter
     * directly, and this receiver refuses TIM-TP, so the residuals carry it. */
    double sse    = 0.0;
    double tieMax = 0.0;
    for (int i = 0; i < m_rateCount; ++i) {
        const double dx = static_cast<double>(m_rate[i].monoNs - base.monoNs) - meanX;
        const double dy = static_cast<double>(m_rate[i].utcNs - base.utcNs) - meanY -
                          slope * dx;
        sse += dy * dy;
        tieMax = std::fabs(dy) > tieMax ? std::fabs(dy) : tieMax;
    }
    m_fitUs = esp_timer_get_time() - t0;
    const double residualNs = std::sqrt(sse / m_rateCount);
    {
        const int64_t jitterNs = residualNs > static_cast<double>(kMaxJitterNs)
                                     ? kMaxJitterNs
                                     : static_cast<int64_t>(residualNs);
        portENTER_CRITICAL(&m_mux);
        m_jitterNs = jitterNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* The worst excursion, from the same residuals: one bad edge shows here
     * while an RMS barely moves. The RMS is the residual already computed. */
    {
        portENTER_CRITICAL(&m_mux);
        m_tieMaxNs = tieMax;
        m_tieRmsNs = residualNs;
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
        m_offsetNs = offsetNs;
        portEXIT_CRITICAL(&m_mux);
    }

    /* The record carries what a client reads, not the raw counter phase. The
     * monotonic counter is never steered -- every correction is arithmetic in
     * now() -- so its phase against GPS sits at milliseconds and wanders with
     * the crystal, while the offset here is against the steered realtime clock
     * and is the number the servo is actually holding. */
    recordSample(static_cast<uint32_t>(gps.tv_sec), offsetNs);

    /* Who may write CLOCK_REALTIME: this source, or the fallback's upstream
     * client. The measurement above happens either way -- the offset is
     * published and charted through a fallback too, and it is what shows the two
     * references pulling apart -- but only the owner corrects anything. Two
     * owners is worse than either alone: the client would step, this would
     * measure the step as an error, and the two would undo each other. */
    if (!ClockSource::instance().gpsSteers()) {
        return;
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
        portENTER_CRITICAL(&m_mux);
        ++m_servoSteps;
        /* The counter has to move with the clock, and the anchor's stamp in it
         * is now stale by the step: dropping it sends now() back to the
         * monotonic path until the next edge pairs. */
        m_anchorPhcNs = 0;
        portEXIT_CRITICAL(&m_mux);
        if (ptp::hardware()) {
            (void)ptp::stepToRealtime();
        }

        /* A step is a phase discontinuity the fit cannot see coming and the
         * step detector below cannot catch: that one compares samples and
         * trips at kRateResetNs (100 ms), while a step here is anything over
         * 10 ms. Left in the window, the samples on the far side of it are a
         * constant offset away and *tieMaxNs* -- the lock's own worst-residual
         * test -- sits above its bound for the five minutes the window takes
         * to flush. That is five minutes of stratum 16 immediately after every
         * hand-back from a fallback, which is the opposite of the point. */
        resetRateWindow();

        /* The set() reached the timebase after the instant gps was read, so the
         * clock is now late by the syscall path -- tens of microseconds that
         * would otherwise sit there until the next edge noticed. Measure it
         * again and take it out with a slew, which is the one kind of
         * correction that does not add latency of its own. */
        struct timespec after = {};
        if (now(after)) {
            struct timespec sysAfter = {};
            clock_gettime(CLOCK_REALTIME, &sysAfter);
            const int64_t residueNs =
                (static_cast<int64_t>(after.tv_sec) - static_cast<int64_t>(sysAfter.tv_sec)) *
                    1000000000 +
                (after.tv_nsec - sysAfter.tv_nsec);
            if (residueNs <= kPostStepSlewNs && residueNs >= -kPostStepSlewNs) {
                slew(residueNs);
            }
        }
        return;
    }

    /* Everything smaller than the step threshold is slewed, so a client polling
     * across this moment never sees the clock jump -- and unlike a step the
     * correction leaves no residue of its own behind. */
    slew(offsetNs);
}

void GpsTime::slew(int64_t offsetNs)
{
    /* IDF's adjtime() applies about 1.5% of elapsed time per call, so this
     * clears in seconds rather than minutes. A slew already in flight is not
     * lost: adjtime() replaces the remaining adjustment rather than queueing
     * behind it. */
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

uint8_t GpsTime::copySats(Sat *out, uint8_t max) const
{
    /* One short critical section around the whole copy: the rows are written by
     * the GPS task on a 2 s message, so a torn read here would mix two samples
     * into one plot. Callers pass their own buffer; /gps/sats is the only one. */
    portENTER_CRITICAL(&m_mux);
    const uint8_t n = m_satCount < max ? m_satCount : max;
    for (uint8_t i = 0; i < n; ++i) {
        out[i] = m_sats[i];
    }
    portEXIT_CRITICAL(&m_mux);
    return n;
}

bool GpsTime::evaluateLock(const LockInputs &in) const
{
    /* Lock needs all three: a fix to trust the date, a valid NAV-PVT to know
     * which second, and an edge recent enough that the clock is still being
     * corrected. Fix type 2 is the floor -- "time-only" (5) dates the second but
     * does not put the receiver anywhere, and this reference does not want to
     * explain that difference in its stratum. */
    if (!(in.anchored && in.timeValid && in.fixType >= 2 && in.ppsAgeUs >= 0 &&
          in.ppsAgeUs < in.lockWindowUs)) {
        return false;
    }

    /* Age is not enough on its own: it says an edge arrived, not that the clock
     * is being held where it belongs. Further tests, each of which demotes to
     * stratum 16 rather than serving on a claim that has stopped being true:
     *
     *  - the newest edge passed the trust test. A pulse that was corrupt, or a
     *    second that did not advance, means this second's anchor is a guess;
     *  - the receiver is not reporting critical jamming. Not gated on freshness:
     *    a jammed front end can hold a fix and keep emitting a pulse while the
     *    solution underneath it drifts, so the pulse still looks regular. MON-HW
     *    carries the state and this module does not send it, so on this board
     *    the test is dormant -- taken from Baumart's server, which demotes on it
     *    unconditionally;
     *  - while the edge is fresh, the phase sits inside the band. A servo still
     *    ringing out from a step has edges arriving perfectly and an offset in
     *    the hundreds of milliseconds;
     *  - while the edge is fresh and a rate is known, the fit's worst residual
     *    is inside the jitter bound. That is the slope test: a window assembled
     *    across a fault fits a line through noise, and the residual is what
     *    gives it away. Skipped in a holdover, where the phase is not measured
     *    at all and the residual is by definition the gap.
     *
     * The offset and residual tests wait until a rate exists, so the first
     * minute still locks on a fresh, trusted edge as it always did. */
    const bool fresh = in.ppsAgeUs <= kLockWindowUs;
    const bool bad   = !in.trusted || in.jamState == 3 ||
                     (fresh && (in.offsetNs > kLockOffsetNs || in.offsetNs < -kLockOffsetNs)) ||
                     (fresh && in.rateKnown && in.tieMaxNs > kLockTieMaxNs);

    /* Debounced per edge, not per call: this runs on every reply and every
     * health poll, so a counter advanced per call would saturate inside a
     * second. m_gateLastPps makes it one vote per edge. A single glitched pulse,
     * or one sample outside the band on its way back from a step, is then not
     * worth a demotion and a relock line -- three consecutive seconds is. */
    uint32_t badFor = 0;
    portENTER_CRITICAL(&m_mux);
    if (m_gateLastPps != in.ppsCount) {
        m_gateLastPps = in.ppsCount;
        m_gateBad     = bad ? m_gateBad + 1 : 0;
    }
    badFor = m_gateBad;
    portEXIT_CRITICAL(&m_mux);
    return badFor < kGateStreak;
}

GpsTime::Status GpsTime::status() const
{
    Status   st;
    int64_t  ppsMonoUs     = 0;
    int64_t  anchorMonoUs  = 0;
    int64_t  suppressUntil = 0;
    int64_t  navSatAtUs    = 0;
    int64_t  pairAgeAtUs   = 0;
    int64_t  monHwAtUs     = 0;
    int64_t  dopAtUs       = 0;
    int64_t  clockAtUs     = 0;
    int64_t  navStatusAtUs = 0;
    int64_t  rfAtUs        = 0;
    int64_t  ioAtUs        = 0;
    uint8_t  recentAt      = 0;

    /* String-ish state captured as raw bytes and mask bits. Nothing below
     * allocates: a copy into a local array is a memcpy, and building the
     * std::string happens after the lock is dropped. */
    char        localMsgpp[241]  = {};
    char        localIo[241]     = {};
    char        localModule[160]    = {};
    char        localBridgeEnd[48]  = {};
    char        localBridgeLast[128] = {};
    size_t      msgppLen = 0, ioLen = 0, moduleLen = 0, endLen = 0, lastLen = 0;
    uint8_t     gnssMask = 0;
    bool        gnssNaked = false, gnssAcked = false;
    uint8_t     tmode = 0;

    portENTER_CRITICAL(&m_mux);
    st.ppsCount    = m_ppsCount;
    st.ppsPtpNs    = m_ppsPtpNs;
    if (m_anchorValid && m_anchorPhcNs != 0 && ptp::hardware()) {
        const int64_t byPhc  = ptp::nowNs() - m_anchorPhcNs;
        const int64_t byMono = (esp_timer_get_time() - m_anchorMonoUs) * 1000;
        st.phcMonoDeltaNs    = byPhc - byMono;
    }
    st.timeValid   = m_timeValid;
    st.fixType     = m_fixType;
    st.satellites  = m_satellites;
    st.anchored    = m_anchorValid;
    st.lastFix     = m_lastFix;
    st.offsetUs    = m_offsetUs;
    st.offsetNs    = m_offsetNs;
    st.frames          = m_decoder.frames();
    st.droppedChecksum = m_decoder.droppedChecksum();
    st.droppedLength   = m_decoder.droppedLength();
    st.rescans         = m_rescans;
    st.bridgeDroppedCrc = m_bridgeDroppedCrc;
    st.ratePpb     = m_ratePpb;
    st.rateSamples = m_rateSamples;
    st.leapSeconds = m_leapSeconds;
    st.leapChange  = m_leapChange;
    st.leapInS     = m_leapInS;
    st.leapValid   = m_leapValid;
    st.lastHoldoverUs    = m_lastHoldoverUs;
    st.lastHoldoverErrNs = m_lastHoldoverErrNs;
    st.timTpCount        = m_timTpCount;
    st.qErrNs            = std::abs(m_qErrNs);
    st.qErrValid         = m_qErrValid;
    st.jitterNs          = m_jitterNs;
    st.pairAgeUs         = m_pairAgeUs;
    st.pairAgeMaxUs      = m_pairAgeMaxUs;
    pairAgeAtUs          = m_pairAgeAtUs;
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
    text::getList(Settings::instance().antDelayNs(), ',', st.antDelayNs);
    text::getList(Settings::instance().gpsUserDelayNs(), ',', st.userDelayNs);
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
    {
        const uint32_t lo = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        const uint32_t hi = systimer_ll_get_counter_value_high(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        st.ticksPerUs    = static_cast<uint32_t>(systimer_us_to_ticks(1));
        st.tickUsDelta   = static_cast<int64_t>(systimer_ticks_to_us((static_cast<uint64_t>(hi) << 32) | lo)) -
                           esp_timer_get_time();
    }
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
    msgppLen = ::strnlen(m_msgppHex.data(), sizeof(m_msgppHex) - 1);
    std::memcpy(localMsgpp, m_msgppHex.data(), msgppLen);
    recentAt             = m_recentAt;
    ioLen = ::strnlen(m_ioHex.data(), sizeof(m_ioHex) - 1);
    std::memcpy(localIo, m_ioHex.data(), ioLen);
    st.msgSkipped        = m_msgSkipped;
    st.ioSeen            = m_ioSeen;
    st.baud              = m_baud;
    ioAtUs               = m_ioAtUs;
    st.servoSteps        = m_servoSteps;
    st.servoClamps       = m_servoClamps;
    st.edgeSpikes        = m_edgeSpikes;
    st.edgeRejects       = m_edgeRejects;
    st.lastEdgeTrusted   = m_lastEdgeTrusted;
    dopAtUs              = m_dopAtUs;
    clockAtUs            = m_clockAtUs;
    navStatusAtUs        = m_navStatusAtUs;
    rfAtUs               = m_rfAtUs;
    st.bridgeSessions    = m_bridgeSessions;
    st.bridgeRefused     = m_bridgeRefused;
    st.bridgeActive      = m_bridgeActive;
    st.bridgeClients     = m_bridgeClients;
    st.bridgeRx          = m_bridgeRx;
    st.bridgeTx          = m_bridgeTx;
    endLen = ::strnlen(m_bridgeEnd.data(), sizeof(m_bridgeEnd) - 1);
    std::memcpy(localBridgeEnd, m_bridgeEnd.data(), endLen);
    lastLen = ::strnlen(m_bridgeLast.data(), sizeof(m_bridgeLast) - 1);
    std::memcpy(localBridgeLast, m_bridgeLast.data(), lastLen);
    /* Named rather than numeric: the interesting thing is which constellations
     * are being solved, and a reader should not have to hold the gnssId table
     * in their head to find out. */
    gnssMask   = m_gnssMask;
    gnssNaked  = m_gnssNaked;
    gnssAcked  = m_gnssAcked;
    tmode      = m_tmode;
    st.svinDurS = static_cast<double>(m_svinDur);
    st.svinAccMm = static_cast<double>(isqrt(m_svinAcc));
    st.svinVarMm2 = static_cast<double>(m_svinAcc);
    st.svinObs  = static_cast<double>(m_svinObs);
    for (int k = 0; k < 3; ++k) {
        st.svinMeanCm[k] = m_svinMean[k];
    }
    st.svinTargetS = static_cast<double>(m_svinDurS);
    st.svinLimitMm = static_cast<double>(m_svinAccMm);
    st.svinValid = m_svinValid;
    st.holdValidated = m_holdValidated;
    st.holdWanted    = m_holdWanted;
    st.holdAssertInS = m_holdAssertAtUs == 0
                           ? -1
                           : (static_cast<double>(m_holdAssertAtUs - esp_timer_get_time()) / 1e6);
    st.lastEdgeAgeMs = m_lastEdgeUs < 0
                           ? -1
                           : (static_cast<double>(esp_timer_get_time() - m_lastEdgeUs) / 1e3);
    st.holdReady     = m_fixType >= 2 && m_satCount >= 4 && m_lastEdgeUs > 0 &&
                   esp_timer_get_time() - m_lastEdgeUs < 3000000;
    st.svinAgeS = m_svinAtUs != 0 ? static_cast<double>(esp_timer_get_time() - m_svinAtUs) / 1e6
                                  : -1;
    st.pinSel            = m_pinSel;
    st.pinDir            = m_pinDir;
    st.pinVal            = m_pinVal;
    st.cn0Max            = m_cn0Max;
    st.cn0Mean           = m_cn0Mean;
    st.cn0Used           = m_cn0Used;
    navSatAtUs           = m_navSatAtUs;
    monHwAtUs            = m_monHwAtUs;
    moduleLen = ::strnlen(m_moduleId.data(), sizeof(m_moduleId) - 1);
    std::memcpy(localModule, m_moduleId.data(), moduleLen);
    ppsMonoUs      = m_ppsMonoUs;
    anchorMonoUs   = m_anchorMonoUs;
    suppressUntil  = m_ppsSuppressUntilUs;
    portEXIT_CRITICAL(&m_mux);

    /* Everything string-shaped happens now, with the lock gone: the copies
     * above were raw bytes into stack arrays, so no allocation and no
     * formatting ever ran with interrupts masked. */
    st.msgppHex.assign(localMsgpp, msgppLen);
    st.ioHex.assign(localIo, ioLen);
    st.moduleId.assign(localModule, moduleLen);
    st.bridgeEnd.assign(localBridgeEnd, endLen);
    st.bridgeLast.assign(localBridgeLast, lastLen);
    {
        static constexpr const char *kNames[] = {"GPS", "SBAS", "Galileo", "BeiDou",
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

    /* Built out here too: sixteen std::format calls are sixteen heap operations.
     * The ring is read without the lock, so a list taken while a frame lands can
     * mix two samples -- acceptable for a diagnostic. */
    st.recentFrames.clear();
    for (uint8_t i = 0; i < 16; ++i) {
        const uint8_t at = static_cast<uint8_t>((recentAt + 15 - i) % 16);
        if (m_recentFrames[at] == 0) {
            continue;
        }
        st.recentFrames += std::format("{:02x}:{:02x} ", m_recentFrames[at] >> 8,
                                       m_recentFrames[at] & 0xFF);
    }

    const int64_t nowUs = esp_timer_get_time();
    if (st.ppsCount != 0) {
        st.ppsAgeUs = nowUs - ppsMonoUs;
    }
    if (st.anchored) {
        st.anchorAgeUs = nowUs - anchorMonoUs;
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

    st.pairAgeS = pairAgeAtUs != 0 ? static_cast<double>(nowUs - pairAgeAtUs) / 1e6 : -1;
    const bool rateKnown = st.rateSamples >= kRateMinSamples;

    /* Two windows, and the difference between them is the servo. Without a rate
     * there is nothing to hold over with, so three missed edges demote; with one
     * the crystal's known error is taken out of every timestamp and ten minutes
     * of holdover is worth serving, with the claim decaying as it ages. */
    st.lockWindowUs = (rateKnown && st.timeValid && st.fixType >= 2) ? kHoldoverWindowUs
                                                                    : kLockWindowUs;

    /* The gate itself, and why each test is there, lives in evaluateLock(); it
     * carries the per-edge debounce too, which is why replyFacts() shares it
     * rather than repeating the predicates. */
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
        st.rootDelayNs = rootDelayNs();

        /* A pulse nobody is watching. While TIM-TP is agreeing to be sent and
         * has stopped arriving, the position of the edge against its own second
         * is unobserved -- the servo cannot see it either, because the anchor is
         * taken from the same edge and moves with it. Unobserved is not known:
         * the 133 ms failure of 18:43 was exactly this state, on a clock quoting
         * 2.4 us of dispersion, and chrony had to work it out for itself.
         *
         * So the stated uncertainty grows with how long the pulse has been
         * unobserved. 10 us a second covers the measured growth (4.2 us/s to
         * 133 ms) with margin, and the 30 s grace keeps a mode change from
         * widening it. Capped at 200 ms, which is six times the worst error seen
         * and still finite. */
        int64_t lastTpUs = 0;
        portENTER_CRITICAL(&m_mux);
        lastTpUs = m_qErrAtUs;
        portEXIT_CRITICAL(&m_mux);
        /* Published inside the budget below as well as on its own, so a wide
         * bound can be told from a bad clock. */
        st.pulseUnobservedNs = unobservedNs();
        st.dispersionNs      = dispersionNs(st.anchorAgeUs, st.pulseUnobservedNs);
        if (m_timTpCfg == 1) {
            const int64_t silentS = (esp_timer_get_time() - lastTpUs) / 1000000;
            if (silentS > 30) {
                if (!m_pulseUnobservedLogged) {
                    m_pulseUnobservedLogged = true;
                    ESP_LOGW(TAG, "TIM-TP silent for %lld s: the pulse position is unobserved, "
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
             st.rateSamples >= kRateMinSamples ? "the servo is carrying it"
                                               : "with no rate yet it demotes in 3 s");
    return seconds;
}

bool GpsTime::now(struct timespec &ts) const
{
    int64_t utcNs     = 0;
    int64_t monoUs    = 0;
    int64_t anchorPhc = 0;
    int32_t ratePpb   = 0;

    portENTER_CRITICAL(&m_mux);
    const bool valid = m_anchorValid;
    utcNs            = m_anchorUtcNs;
    monoUs           = m_anchorMonoUs;
    anchorPhc        = m_anchorPhcNs;
    ratePpb          = m_ratePpb;
    portEXIT_CRITICAL(&m_mux);

    if (!valid) {
        return false;
    }

    /* The 1588 counter is the served base while it runs: a reply's arrival
     * stamp is taken by hardware in that base, and a served time computed in
     * another one would have to be converted back on every packet. A stale
     * anchor stamp -- the pair and the anchor come from the same edge, so this
     * is only the first second of a boot -- falls through to the monotonic
     * path. */
    if (ptp::hardware() && anchorPhc != 0) {
        const int64_t now       = ptp::nowNs();
        const int64_t elapsedNs = now - anchorPhc;
        if (elapsedNs >= 0 && elapsedNs < 2000000000LL) {
            static bool told = false;
            if (!told) {
                told = true;
                ESP_LOGI(TAG, "phc anchor: phc elapsed %lld ns, mono elapsed %lld ns",
                         static_cast<long long>(elapsedNs),
                         static_cast<long long>(esp_timer_get_time() * 1000 - monoUs * 1000));
            }
            /* Nanoseconds here, so the ppb divisor is 1e9 and not the 1e6 the
             * microsecond path above uses: 30 ppm over half a second is 15 us,
             * which is 15000 ns, which is 5e8 * 3e4 / 1e9. */
            const int64_t totalNs = utcNs + elapsedNs - elapsedNs * ratePpb / 1000000000;
            ts.tv_sec             = static_cast<time_t>(totalNs / 1000000000);
            ts.tv_nsec            = static_cast<long>(totalNs % 1000000000);
            return true;
        }
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

    if (st.locked) {
        if (st.anchorAgeUs > kLockWindowUs) {
            /* Still servable, and worth saying why in a line somebody reads
             * later: this is the holdover the servo bought. */
            return std::format("GPS/PPS, {} fix, {} satellites, holding over {:.1f} s",
                               fixText(st.fixType), st.satellites,
                               static_cast<double>(st.anchorAgeUs) / 1e6);
        }
        return std::format("GPS/PPS, {} fix, {} satellites", fixText(st.fixType), st.satellites);
    }
    if (st.fixType >= 2 && st.timeValid) {
        /* A fix but no edge we can trust: the receiver is fine and the pulse is
         * not, which is a different fault from "no fix" and worth saying
         * differently in the boot log. The refused case is not the same as the
         * missing one either -- edges are arriving, they just are not seconds --
         * and reading "no fresh edge" on a box whose LED is blinking would send
         * the reader to the wrong wire. */
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

    /* The fallback arbiter, once a second and from boot: a device whose antenna
     * is missing has to be able to hand over without anybody asking it to.
     *
     * What it is told is "the pulse is back and trustworthy", which is
     * deliberately *not* the lock. Lock also needs the phase inside
     * kLockOffsetNs, and while a fallback holds the clock the phase is the
     * distance between the upstream's clock and the receiver's -- it cannot come
     * back into the band until this servo steers again, and the servo is gated
     * off until the arbiter hands back. Judging the return on the lock alone
     * therefore never returns: each waits for the other. */
    const bool holding = st.locked && st.anchorAgeUs > kLockWindowUs;
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
            /* Edges arriving and being refused is the one PPS fault that looks
             * healthy from a distance: the counter climbs and nothing works. */
            ESP_LOGW(TAG, "not locked: %u PPS edges in a row refused (wrong spacing, or the "
                          "module's second did not advance with them)",
                     st.edgeRejects);
        } else {
            ESP_LOGW(TAG, "not locked (fix %s, last edge %lld ms ago): stratum 16 until it returns",
                     fixText(st.fixType), static_cast<long long>(st.ppsAgeUs / 1000));
        }
    }

    /* Inside a holdover nothing above changes for ten minutes, which would
     * otherwise look like a healthy clock that has quietly lost its PPS. Say it
     * when the edge goes stale, and say it again when it comes back. */
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

/* The most uncertainty this reference will serve through while still calling
 * itself locked.
 *
 * A healthy box states about 130 us of dispersion against a serve bound near
 * 40 us, so 400 us is ten times the bound and three times the healthy floor --
 * far enough out that only a real fault reaches it. Past it the reply becomes
 * stratum 16 / INIT / leap 3, which every client already reads as "ignore me".
 *
 * The alternative, which this box ran until now, is a stratum-1 badge held
 * through a 258 ms error with 19 ms of dispersion underneath it. chrony had to
 * work the falseticker out for itself; a client that trusts stratum alone would
 * have taken the 258 ms. */
constexpr int64_t kServeHonestNs = 400000;
/* What clients report about the path may widen the claim, never close the
 * gate: only the reference's own terms get to say it is no longer one. */
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
    /* Since the last edge that became an anchor, not since the last edge. The
     * interrupt stamps every pulse whether or not a frame ever dated it, so
     * with the receiver's frames gone and its pulse still arriving the raw age
     * stayed under a second: "locked", fresh, and no wander charged, on an
     * anchor that was minutes old. */
    in.ppsAgeUs  = !m_anchorValid ? -1 : esp_timer_get_time() - m_anchorMonoUs;
    in.trusted   = m_lastEdgeTrusted;
    in.offsetNs  = m_offsetNs;
    in.rateKnown = m_rateSamples >= kRateMinSamples;
    in.tieMaxNs  = m_tieMaxNs;
    in.jamState  = m_jamState;
    in.ppsCount  = m_ppsCount;
    rf.lastSync  = m_lastFix;
    portEXIT_CRITICAL(&m_mux);

    in.lockWindowUs = (in.rateKnown && in.timeValid && in.fixType >= 2) ? kHoldoverWindowUs
                                                                      : kLockWindowUs;
    rf.locked       = evaluateLock(in);
    rf.stratum      = rf.locked ? 1 : 16;
    /* "GPS" while it is a stratum-1 reference, "INIT" once it is not: a client
     * reading stratum 16 needs the refid to look like a refusal, not a
     * reference. */
    rf.refId = rf.locked ? kRefIdGps : kRefIdInit;
    /* The receiver's own leap notice, straight into the packet: only it knows
     * whether the end of the UTC day has 59, 60 or 61 seconds. */
    rf.leap = leapIndicator();
    if (rf.locked) {
        const int64_t budget = dispersionNs(in.ppsAgeUs, unobservedNs());
        if (budget > 0) {
            rf.rootDelayNs  = static_cast<uint32_t>(rootDelayNs());
            rf.dispersionNs = static_cast<uint32_t>(budget);
        }
        /* The gate. Dispersion is the one term the client cannot check for
         * itself, so it is also the one that decides whether this is a
         * reference or a guess. */
        if (budget > kServeHonestNs) {
            rf.locked   = false;
            rf.stratum  = 16;
            rf.refId    = kRefIdInit;
            rf.demoted  = true;
            if (!m_gateLogged) {
                m_gateLogged = true;
                ESP_LOGW(TAG, "demoting: %lld us of dispersion is past the %lld us a reply may "
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

/* The GPS as the responder sees it. Stateless -- everything it answers comes
 * from the GpsTime singleton, which is older than any responder task. */
class GpsTimeRef : public TimeRef {
public:
    /* replyFacts(), not status(): status() assembles nine std::strings for the
     * HTTP side, and the responder was paying for every one of them between its
     * two stamps -- 250 us of allocation, measured, inside the window that
     * decides the T2/T3 pair. The budget's terms are the same ones status()
     * uses. */
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
