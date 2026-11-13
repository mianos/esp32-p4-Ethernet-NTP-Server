/*
 * The stratum-1 reference: a u-blox LEA-8 on UART1, disciplined by its PPS.
 *
 * Wiring, in the board header's GPIO numbers:
 *
 *   module TX  -> GPIO16   UART1 RX   (time of day)
 *   module RX  -> GPIO15   UART1 TX   (configuration, sent once)
 *   module PPS -> GPIO14   rising edge, interrupt (when the second starts)
 *
 * The split between those two wires is the whole accuracy story:
 *
 *  - the UART carries UBX-NAV-PVT once a second: UTC date and time, fix type,
 *    satellite count. It arrives tens of milliseconds late, and that does not
 *    matter, because it only says *which* second it is;
 *  - the PPS edge says *when* that second starts. The interrupt handler stamps
 *    esp_timer_get_time() as it fires -- microseconds of latency, not the
 *    milliseconds a 92-byte frame takes to dribble out of a 9600-baud port --
 *    and the NAV-PVT that follows names the second that edge began.
 *
 * Together they give an anchor: (UTC second, monotonic instant). Timestamps
 * come from the anchor plus elapsed monotonic time, so what the LAN is served
 * is not the system clock at all. The system clock is still slewed towards the
 * same anchor with adjtime(), because the log lines, /healthz and the HTTP Date
 * headers all read it -- but a correction in flight there never reaches a reply.
 *
 * What this refuses to do, because the alternative is lying to clients:
 *
 *  - serve without the PPS. The UART alone dates the second but cannot say
 *    where in it we are: that is a tens-of-milliseconds server, i.e. worse than
 *    the upstream SNTP clock it would be replacing. Lock needs an edge from the
 *    last 3 s -- or, once the servo below knows the crystal's rate, from the
 *    last 10 minutes, with root dispersion growing the whole time so a client
 *    can see the holdover decay instead of having to trust it;
 *  - fall back to SNTP when the fix goes away. A reference that silently turns
 *    into a different reference is one whose stratum means nothing. Settings
 *    says which source is authoritative; if that source is not healthy, the
 *    honest answer is "do not use me";
 *  - pretend to interpolate leap seconds. The u-blox UTC fields already include
 *    them, so the epoch this builds is the same one lwIP's SNTP client sets: at
 *    the end of a leap day the second after 23:59:59 is served twice, which is
 *    what an NTP server is supposed to do. What the module adds is *notice* --
 *    UBX-NAV-TIMELS says a leap is pending and how far off it is, and the leap
 *    indicator goes to 1 (or 2) for the last minute, so a client is told rather
 *    than surprised. Nothing here smooths a second away.
 *
 * Two measurements run on top of the anchor, and they are the difference
 * between a clock that is right and a clock that stays right:
 *
 *  - the servo. Every anchor is a sample of esp_timer against the GPS, so the
 *    slope of those samples is the crystal's rate error, fitted over the last
 *    five minutes. Served time is corrected by it, which removes the sawtooth
 *    between edges and, far more importantly, turns a holdover from "3 seconds
 *    then demote" into minutes of microsecond-level time;
 *  - the error budget. Locked, the dispersion this reference states is ten
 *    microseconds plus one ppm for every second since the last edge: not the
 *    fit's own confidence (which is parts per billion and would be a boast) but
 *    a cheap crystal's wander over temperature, which is the term that actually
 *    decides how long a holdover is worth serving.

 *
 * The receiver is configured over UART on every boot -- UBX-NAV-PVT at 1 Hz,
 * UBX-NAV-TIMELS at 1 Hz for the leap-second notice, NMEA off -- and
 * deliberately not saved to its flash: the module keeps its own defaults, so it
 * can still be plugged into u-center afterwards. The PPS configuration is
 * likewise left at the module's default (1 Hz, UTC-aligned, rising edge at the
 * top of the second); if it has been changed with u-center, CFG-TP5 wants
 * syncMode UTC and a period of 1 s.
 *
 * Without a module on the pins (or with the wires crossed) nothing here fails:
 * no UBX frames arrive, the baud scan says so once, and the reference answers
 * stratum 16 like any other reference that cannot vouch for its clock.
 */
#pragma once

#include <cstdint>
#include <ctime>
#include <memory>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "time_ref.h"

class GpsTime {
public:
    /* What /healthz reports about the receiver, and what the reference is
     * allowed to claim. Copied out under the same lock the PPS interrupt takes:
     * the HTTP task reads these while the GPS task writes them. */
    struct Status {
        bool     timeValid  = false; /* the NAV-PVT date and time fields are good */
        uint8_t  fixType    = 0;     /* u-blox fix type: 0 none, 2 2D, 3 3D */
        uint8_t  satellites = 0;
        uint32_t ppsCount   = 0;
        int64_t  ppsAgeUs   = -1;    /* since the last edge; -1 means never */
        bool     anchored   = false; /* an edge and a NAV-PVT have been paired */
        time_t   lastFix    = 0;     /* UTC second of the last accepted anchor */
        int64_t  offsetUs   = 0;     /* GPS minus the system clock, last measured */
        bool     locked     = false; /* a stratum-1 reference right now */

        /* The servo: esp_timer's rate error against the GPS in parts per
         * billion (positive means the counter runs fast), how many anchors the
         * fit is based on, and how stale an edge may be and still count as a
         * lock -- 3 s before the rate is known, 10 minutes after. */
        int32_t  ratePpb     = 0;
        uint16_t rateSamples = 0;
        bool     rateKnown   = false;
        int64_t  lockWindowUs = 0;

        /* The error budget this reference states for itself, in nanoseconds:
         * the anchor's own accuracy while fresh, plus wander since the edge. */
        int64_t  dispersionNs = 0;

        /* The last holdover, and what it cost: how long the edge was missing
         * and what the servo's prediction was worth when it came back. The only
         * number that can honestly justify serving through a gap -- and the one
         * a README should quote instead of a promise. */
        int64_t  lastHoldoverUs    = 0;
        int64_t  lastHoldoverErrNs = 0;

        /* Leap seconds, from UBX-NAV-TIMELS: the current GPS-UTC offset, the
         * pending change (+1, -1, or 0 for none) and how far off it is. leapValid
         * is false until the module has said anything, so a board whose module
         * ignores the message reports nothing rather than a guess. */
        bool     leapValid   = false;
        int8_t   leapSeconds = 0;
        int8_t   leapChange  = 0;
        int32_t  leapInS     = 0;

        /* Time left on the POST /gps/pps hook, which swallows edges on purpose
         * so a holdover can be watched with the PPS wire soldered on. Report
         * it, so nobody mistakes an artificial state for a fault. */
        int64_t  ppsSuppressUs = 0;
    };

    static GpsTime &instance();

    /* Install UART1, the PPS interrupt and the task. Idempotent, and safe on a
     * board with nothing wired to those pins: a missing module is a log line
     * and a stratum-16 reference, not a failed boot. */
    esp_err_t begin();

    Status status() const;

    /* The anchor plus elapsed monotonic time. False when no edge has been
     * paired with a NAV-PVT yet, which is what the reference turns into
     * "not locked". */
    bool now(struct timespec &ts) const;

    /* "GPS/PPS, 3D fix, 9 satellites" or "GPS/PPS (no fix yet)". */
    std::string describe() const;

    /* The leap indicator this reference would put in a reply: 1 for "the last
     * minute has 61 seconds", 2 for 59, 0 the rest of the time. Only from the
     * receiver's own TIMELS, and only in the minute before the event: a server
     * that guessed "there might be a leap" would be warning about nothing. */
    uint8_t leapIndicator() const;

    /* Test hook for POST /gps/pps: stop counting PPS edges for this many
     * seconds, so a holdover can be measured on a board whose PPS line is
     * soldered on. Not a way to run without the PPS -- the anchor stops
     * advancing and the reference demotes itself exactly as it would if the
     * wire had come off. Returns the number of seconds now in force. */
    uint32_t suppressPps(uint32_t seconds);

private:
    static constexpr const char *TAG = "gps_time";

    GpsTime() = default;

    static void ppsIsr(void *arg);
    static void taskEntry(void *arg);

    void task();
    void advanceBaud();
    void configureModule();
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
    void logLockChange();

    /* UBX framing, one byte at a time. NMEA sentences and rubbish are skipped
     * by the same loop: the parser only ever looks for 0xB5 0x62. */
    enum class Parse { Sync1, Sync2, Class, Id, LenLo, LenHi, Payload, CkA, CkB };
    void feed(uint8_t byte);
    void onFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
    void onNavPvt(const uint8_t *p, uint16_t len);
    void onNavTimels(const uint8_t *p, uint16_t len);
    void anchor(time_t second);

    /* The servo. Each anchor is one sample of esp_timer against the GPS; the
     * slope of the last five minutes of them is the crystal's rate error, and
     * that is what lets a holdover last minutes instead of seconds. */
    struct RateSample {
        int64_t monoUs = 0; /* the PPS edge, as esp_timer saw it */
        int64_t utcNs  = 0; /* the UTC second that edge began */
    };
    void pushRateSample(int64_t monoUs, int64_t utcNs);
    void fitRate();

    /* Slew the system clock towards the anchor, but only while this source is
     * the configured one -- two owners of CLOCK_REALTIME is worse than either. */
    void steer();

    /* The edge, written by the interrupt and read by the task and the responder.
     * A spinlock rather than atomic<int64_t>: 64-bit atomics on this core go
     * through libatomic, and the section is a handful of instructions. Mutable
     * because status() and now() are const and still have to take it. */
    mutable portMUX_TYPE m_mux          = portMUX_INITIALIZER_UNLOCKED;
    int64_t      m_ppsMonoUs    = 0;
    uint32_t     m_ppsCount     = 0;
    int64_t      m_anchorUtcNs  = 0;
    int64_t      m_anchorMonoUs = 0;
    bool         m_anchorValid  = false;
    time_t       m_lastFix      = 0;
    bool         m_timeValid    = false;
    uint8_t      m_fixType      = 0;
    uint8_t      m_satellites   = 0;
    int64_t      m_offsetUs     = 0;
    int32_t      m_ratePpb      = 0; /* the servo's answer, used by now() */
    uint16_t     m_rateSamples  = 0;
    int8_t       m_leapSeconds  = 0;
    int8_t       m_leapChange   = 0;
    int32_t      m_leapInS      = 0;
    bool         m_leapValid    = false;
    int64_t      m_ppsSuppressUntilUs = 0; /* the POST /gps/pps hook */
    int64_t      m_lastHoldoverUs     = 0;
    int64_t      m_lastHoldoverErrNs  = 0;

    /* The GPS task's own state; no other task reads it. */
    Parse    m_parse        = Parse::Sync1;
    uint8_t  m_cls          = 0;
    uint8_t  m_id           = 0;
    uint16_t m_len          = 0;
    uint16_t m_got          = 0;
    uint8_t  m_ckA          = 0;
    uint8_t  m_ckB          = 0;
    bool     m_overflow     = false;
    uint8_t  m_payload[100] = {};
    uint32_t m_pairedCount  = 0; /* the PPS edge the current anchor came from */
    bool     m_stepped      = false;
    bool     m_frameSeen    = false;
    bool     m_fixLogged    = false;
    bool     m_wasLocked    = false;
    bool     m_ackLogged    = false;
    bool     m_nakLogged    = false;
    bool     m_noDataLogged = false;
    int      m_baudIndex    = 0;
    uint32_t m_baud         = 0;
    bool     m_begun        = false;
    int64_t  m_nextScanUs   = 0;
    int64_t  m_nextCheckUs  = 0;

    /* Five minutes of anchors: long enough that the fit's own confidence is
     * parts per billion, short enough that the crystal's temperature wander is
     * averaged rather than fitted as a curve. */
    static constexpr int kRateWindow     = 300;
    static constexpr int kRateMinSamples = 60;
    RateSample m_rate[kRateWindow] = {};
    int        m_rateCount         = 0; /* how many slots are filled */
    int        m_rateNext          = 0; /* where the next sample goes */
    int64_t    m_lastRateOffsetNs  = 0;
    bool       m_lastRateOffsetSet = false;
    bool       m_rateLogged        = false;
    int64_t    m_lastLoggedPpb     = 0;
    bool       m_holdoverLogged    = false;
    int8_t     m_leapLoggedSeconds = -1;
    int8_t     m_leapLoggedChange  = 0;
};

/* The GPS as the responder sees it: stratum 1 and refid "GPS" while the PPS is
 * fresh and the fix is valid, stratum 16 otherwise. */
std::unique_ptr<TimeRef> makeGpsTimeRef();
