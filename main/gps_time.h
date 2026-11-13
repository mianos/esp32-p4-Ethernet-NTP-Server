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
 *  - the error budget. Locked, the dispersion this reference states is what it
 *    has measured about its own stamping -- the receiver's stated pulse
 *    quantisation and the observed spread between edges -- plus one ppm for
 *    every second since the last edge: not the fit's own confidence (which is
 *    parts per billion and would be a boast) but a cheap crystal's wander over
 *    temperature, which is the term that actually decides how long a holdover is
 *    worth serving.
 *
 * The receiver is configured over UART on every boot -- NAV-PVT at 1 Hz for the
 * date, NAV-TIM-TP at 1 Hz for the pulse's own qualities, NAV-TIMELS at 1 Hz for
 * the leap-second notice, NMEA off -- and deliberately not saved to its flash.
 * (This unit rejects the TIM-TP enable, so that half of it is dormant here; the
 * jitter the message would have described is measured from the servo instead.) It is
 * the module keeps its own defaults, so it can still be plugged into u-center
 * afterwards. The timepulse is asserted with CFG-TP5 rather than assumed: 1 Hz,
 * UTC-aligned, rising edge, 100 ms wide. A pulse somebody moved in u-center is
 * the one fault that makes this server confidently wrong, and a 32-byte frame
 * per boot is cheaper than a README telling the reader to go and look.
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
         * the anchor's own accuracy while fresh, plus wander since the edge.
         * rootDelayNs is the half of it that sits in the measurement path --
         * the interrupt's latency and the receiver's own pulse quantisation --
         * and dispersionNs is that plus the edge-to-edge jitter actually seen
         * plus wander since the edge. Neither is a constant any more: see
         * onNavTimTp() for what is measured and what stays named. */
        int64_t  rootDelayNs  = 0;
        int64_t  dispersionNs = 0;

        /* What UBX-NAV-TIM-TP says about the pulse itself: the receiver's
         * stated quantisation error (how well it knows where the edge truly
         * is, picoseconds on the wire, ns here), the measured spread of our own
         * stamping, and whether the pulse is where the top of the second puts
         * it. timTpCount is a plain "is it flowing" counter, so a module that
         * ignores the message is distinguishable from a quiet one. */
        int32_t  qErrNs         = 0;
        bool     qErrValid      = false;
        int64_t  jitterNs       = 0;
        uint32_t timTpCount     = 0;
        bool     pulseOnSecond  = false;

        /* What the receiver says it is, from UBX-MON-VER, and which of this
         * firmware's two configuration messages it refused. An empty moduleId
         * with both flags false means the question was never answered -- a
         * module that ignores the poll is not the same as one that answered and
         * refused, and TIM-TP needs protocol 15, which an old unit will not
         * have. All three go out over /healthz rather than only to a log that
         * drops bytes at exactly the wrong moment. */
        std::string moduleId;
        /* The bias switch, driven by the ESP because the module will not. */
        bool     antBiasOn     = false;
        bool     antShort      = false;
        bool     antLatched    = false;

        bool     tp5Rejected   = false;
        bool     timTpRejected = false;
        bool     timTpAcked    = false;

        /* The antenna, from UBX-MON-HW, and the signal quality behind it, from
         * UBX-NAV-SAT. Comparing antennas needs C/N0 and the AGC/noise/jammer
         * figures, not satellite counts: a high-gain antenna on a preamp can
         * push this receiver into overload, and the two numbers that show it are
         * a falling agcCnt and a jamming state above "ok". antPower is what the
         * bias tee is actually doing -- OFF, ON, or DONTKNOW when the firmware
         * has no supervisor -- and antStatus reports the fault the supervisor
         * found: SHORT, OPEN, or OK. */
        uint8_t  antStatus    = 0; /* MON-HW aStatus: 0 init, 1 dontknow, 2 ok, 3 short, 4 open */
        uint8_t  antPower     = 0; /* MON-HW aPower: 0 off, 1 on, 2 dontknow */
        uint16_t noisePerMs   = 0;
        uint16_t agcCnt       = 0;
        uint8_t  jamInd       = 0;
        uint8_t  jamState     = 0; /* MON-HW flags 2:3: 0 unknown, 1 ok, 2 warning, 3 critical */
        bool     monHwValid   = false;
        bool     antCfgRejected = false;
        bool     antCfgAcked    = false;
        uint8_t  cn0Max       = 0; /* strongest used satellite, dBHz */
        uint8_t  cn0Mean      = 0; /* mean over used satellites */
        uint8_t  cn0Used      = 0; /* how many satellites were used in that sample */
        int64_t  navSatAgeUs  = -1;
        int64_t  monHwAgeUs   = -1;
        int64_t  fitUs        = 0; /* how long the last rate fit took */
        uint32_t bridgeSessions = 0;
        uint32_t bridgeRx       = 0;
        uint32_t bridgeTx       = 0;
        std::string bridgeEnd;
        std::string bridgeLast;
        /* MON-HW's pin map: which module pins the receiver has assigned, their
         * direction and level. The antenna supervisor is a pin pair, so this is
         * where its numbers come from when CFG-ANT has to name them. */
        uint32_t pinSel       = 0;
        uint32_t pinDir       = 0;
        uint32_t pinVal       = 0;

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

    /* Bench hooks for an antenna that is being swapped. setAntennaBias() sends
     * CFG-ANT with supply control on or off, so a preamp can be powered and
     * unpowered without a reflash and the C/N0 either side compared; the
     * automatic case never turns it off, so this only ever describes a
     * deliberate experiment. resetReceiver() is a controlled hot start: it
     * restarts the navigation engine without touching the almanac, for a module
     * that has been reconfigured into a corner. Both return false if the module
     * refused. */
    void requestAntennaBias(bool on);
    void requestReset(bool cold = false, bool hardware = false, bool powerCycle = false);
    void requestDefaults();

    /* A transparent TCP<->UART1 bridge for u-center, on kBridgePort. While a
     * client is attached the GPS task stops reading the port so exactly one
     * thread owns it, and everything -- polls, configuration, the lot -- is the
     * client's. On disconnect the task resumes and re-applies its own
     * configuration, so a session cannot leave the receiver somewhere the
     * firmware does not expect. */
    static constexpr int kBridgePort = 3333;

    /* The antenna switch, on the ESP side: AADET_N is pulled low to power the
     * preamp, SCS1_N reports a short. The module's own supervisor does not
     * exist on this firmware (CFG-ANT is refused for every flag and pin
     * combination tried), so the firmware does the job -- including cutting the
     * bias on a short, which is the one part of it that protects hardware. */
    static constexpr int kAntDrivePin = 17;
    static constexpr int kAntSensePin = 18;

private:
    static constexpr const char *TAG = "gps_time";

    GpsTime() = default;

    static void ppsIsr(void *arg);
    static void taskEntry(void *arg);
    static void bridgeTaskEntry(void *arg);
    void bridgeTask();

    void task();
    void advanceBaud();
    void configureModule();
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

    /* One configuration message at a time, waiting for its own ACK or NAK
     * before the next is sent. The reply's payload names the class and id of
     * the message it answers, so a single outstanding message is unambiguous --
     * and it has to be, because this receiver does not answer everything (it
     * never answers MON-VER) and an order-based guess drifts apart within one
     * boot. pump() keeps parsing while it waits; the GPS task would otherwise
     * be blind for the duration. */
    int8_t  sendAndWait(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                        uint32_t timeoutUs = 250000);
    void    pump(int64_t timeoutUs);
    uint8_t m_awaitCls      = 0;
    uint8_t m_awaitId       = 0;
    int8_t  m_awaitResult   = -1; /* -1 pending, 1 ACK, 0 NAK, -2 timed out */

    void logLockChange();

    /* UBX framing, one byte at a time. NMEA sentences and rubbish are skipped
     * by the same loop: the parser only ever looks for 0xB5 0x62. */
    enum class Parse { Sync1, Sync2, Class, Id, LenLo, LenHi, Payload, CkA, CkB };
    void feed(uint8_t byte);
    void onFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
    void onNavPvt(const uint8_t *p, uint16_t len);
    void onNavTimels(const uint8_t *p, uint16_t len);
    void onNavTimTp(const uint8_t *p, uint16_t len);
    void onMonVer(const uint8_t *p, uint16_t len);
    void onMonHw(const uint8_t *p, uint16_t len);
    void onNavSat(const uint8_t *p, uint16_t len);
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

    /* What TIM-TP has said, when the receiver has it at all, plus the spread
     * our own stamps show. The spread comes from the servo's residuals rather
     * than from TIM-TP, because this module refuses TIM-TP: the fit already
     * computes how far each edge sits from the line through all of them, and
     * that is the same jitter the message would have described. */
    uint32_t     m_timTpCount      = 0;
    int32_t      m_qErrNs          = 0;
    bool         m_qErrValid       = false;
    bool         m_pulseOnSecond   = false;
    int64_t      m_jitterNs        = 0;
    int64_t      m_fitUs           = 0;

    /* The GPS task's own state; no other task reads it. */
    Parse    m_parse        = Parse::Sync1;
    uint8_t  m_cls          = 0;
    uint8_t  m_id           = 0;
    uint16_t m_len          = 0;
    uint16_t m_got          = 0;
    uint8_t  m_ckA          = 0;
    uint8_t  m_ckB          = 0;
    bool     m_overflow     = false;
    uint8_t  m_payload[512] = {}; /* NAV-SAT with 32 satellites is 392 bytes */
    uint32_t m_pairedCount  = 0; /* the PPS edge the current anchor came from */
    uint32_t m_framesLogged = 0;
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
    bool       m_timTpLogged       = false;
    bool       m_timTpGoneLogged   = false;
    uint32_t   m_timTpLastCount    = 0;
    int64_t    m_timTpLastUs       = 0;
    bool       m_pulseOnSecondSeen = true;
    bool       m_tp5Acked          = false;
    bool       m_tp5Naked          = false;
    bool       m_timTpNaked        = false;
    bool       m_timTpAcked        = false;
    bool       m_antCfgAcked       = false;
    bool       m_antCfgNaked       = false;
    bool       m_antLogged         = false;
    uint8_t    m_antStatus         = 0;
    uint8_t    m_antPower          = 0;
    uint16_t   m_noisePerMs        = 0;
    uint16_t   m_agcCnt            = 0;
    uint8_t    m_jamInd            = 0;
    uint8_t    m_jamState          = 0;
    uint32_t   m_pinSel            = 0;
    uint32_t   m_pinDir            = 0;
    uint32_t   m_pinVal            = 0;
    uint32_t   m_pinsLogged        = 0;
    bool       m_monHwValid        = false;
    int64_t    m_nextHwUs          = 0;
    int64_t    m_warmRetryAtUs     = 0;
    uint8_t    m_warmRetries       = 0;
    /* Requests from the HTTP task, serviced by the GPS task on its next pass so
     * that only one thread ever writes the UART or advances the parser. */
    volatile int8_t m_biasReq      = -1; /* -1 none, 0 off, 1 on */
    volatile bool   m_resetReq     = false;
    volatile bool   m_resetCold    = false;
    volatile bool   m_resetHard    = false;
    volatile bool   m_resetPower   = false;
    volatile bool   m_defaultsReq  = false;
    volatile bool   m_bridgeActive = false;
    bool           m_antBiasOn     = false;
    bool           m_antShort      = false;
    bool           m_antLatched    = false;
    int            m_shortSamples  = 0;
    int            m_senseLogged   = -1;
    void applyBias(bool on);
    int64_t         m_reconfigAtUs = 0;
    int64_t    m_monHwAtUs         = 0;
    volatile uint32_t m_bridgeRx   = 0;
    volatile uint32_t m_bridgeTx   = 0;
    volatile uint32_t m_bridgeSessions = 0;
    char       m_bridgeEnd[48]     = "none";
    char       m_bridgeLast[128]   = "";
    uint8_t    m_cn0Max            = 0;
    uint8_t    m_cn0Mean           = 0;
    uint8_t    m_cn0Used           = 0;
    int64_t    m_navSatAtUs        = 0;
    bool       m_verLogged         = false;
    bool       m_timTpProbed       = false;
    int64_t    m_timTpProbeAtUs    = 0;
    char       m_moduleId[160]     = {};
};

/* The GPS as the responder sees it: stratum 1 and refid "GPS" while the PPS is
 * fresh and the fix is valid, stratum 16 otherwise. */
std::unique_ptr<TimeRef> makeGpsTimeRef();
