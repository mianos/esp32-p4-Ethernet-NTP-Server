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

#include "driver/mcpwm_cap.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "text.h"
#include "time_ref.h"
#include "ubx.h"

class GpsTime : private ubx::Listener {
public:
    /* One satellite out of UBX-NAV-SAT, for the sky view. Field names follow
     * the message: gnssId and svId identify it, cno/elev/azim are what a
     * polar plot needs, and flags carry quality, svUsed, health and ephAvail
     * in the receiver's own bit order so nothing has to be inferred twice.
     *
     * 32 is a ceiling, not an expectation: the payload holds 42 at 12 bytes
     * each, but a sky that big means everything marginal is included and the
     * extra rows tell a plot nothing. Multiband receivers put one satellite in
     * here several times, once per band, which is why this is not deduplicated
     * by svId. */
    static constexpr uint8_t kMaxSats = 32;
    struct Sat {
        uint8_t  gnss  = 0;
        uint8_t  sv    = 0;
        uint8_t  cno   = 0;
        int8_t   elev  = 0;
        int16_t  azim  = 0;
        uint32_t flags = 0;
        /* UBX-NAV-SAT pseudorange residual, centimetres. The receiver's own
         * statement of how far this satellite's measurement sits from a
         * consistent solution, which is what multipath actually looks like --
         * C/N0 only proxies it, and a strong reflection is loud and wrong.
         * 0x8000 is the message's "not available". */
        int16_t  prRes      = 0;
        bool     prResValid = false;
    };

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
        /* The same offset in nanoseconds. Microseconds hide the one step a cable
         * delay change makes -- a few metres of lead is a few tens of ns -- and
         * that step is the only way to check the correction went the right way
         * without a reference the box does not have. */
        int64_t  offsetNs   = 0;
        /* Framing counters, straight off the decoder. */
        uint32_t frames          = 0;
        uint32_t droppedChecksum = 0;
        uint32_t droppedLength   = 0;
        uint32_t rescans         = 0;
        uint32_t bridgeDroppedCrc = 0;
        bool     locked     = false; /* a stratum-1 reference right now */

        /* The servo: esp_timer's rate error against the GPS in parts per
         * billion (positive means the counter runs fast), how many anchors the
         * fit is based on, and how stale an edge may be and still count as a
         * lock -- 3 s before the rate is known, 10 minutes after. */
        int32_t  ratePpb     = 0;
        uint16_t rateSamples = 0;
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
        /* The part of dispersion that exists because TIM-TP has gone silent and
         * the pulse position is therefore unobserved. Published on its own so
         * the term is deniable in public rather than buried in a total. */
        int64_t  pulseUnobservedNs = 0;

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
        int64_t  pulseOffsetNs  = 0; /* where TIM-TP says the pulse sits, ns */
        int32_t  pvtNanoNs      = 0; /* NAV-PVT's offset into its own second */
        uint32_t pvtItowMs      = 0; /* the same instant, on the GPS time base */
        uint8_t  timTpFlags     = 0; /* timeBase and qErrInvalid live here */
        /* TIM-TOS: the pulse against the UTC top of second, with an
         * uncertainty, and whether it is locked at all. */
        uint32_t tosFlags       = 0;
        int32_t  tosUtcOffsetNs = 0;
        uint32_t tosUtcUncNs    = 0;
        uint8_t  tosCfg         = 0;
        uint32_t timTpTimN      = 0; /* replies to the TIM spelling */
        uint32_t timTpNavN      = 0; /* replies to the NAV spelling */
        uint8_t  rateCfg        = 0; /* CFG-RATE: 1 accepted, 2 rejected */
        uint8_t  nav5Cfg        = 0; /* CFG-NAV5: 1 accepted, 2 rejected */
        int32_t  tp5CableNs     = 0; /* as the module holds them, not as sent */
        int32_t  tp5UserNs      = 0;
        uint32_t tp5Flags       = 0;
        uint32_t tp5PeriodUs    = 0;
        uint32_t tp5Ratio       = 0;

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
        /* What the receiver has been told the lead delays the signal by, so the
         * installed figure can be read back rather than remembered. */
        double   antDelayNs    = 0;
        double   userDelayNs   = 0; /* as stored, not as the module echoes it */

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

        /* Geometry and clock quality: UBX-NAV-DOP, UBX-NAV-CLOCK and
         * UBX-NAV-STATUS. DOP is the one that separates "the sky is blocked"
         * from "the visible constellation is strung out in a bad line" -- with
         * C/N0 alone those look identical, and only one of them is fixed by
         * moving the antenna. tAcc is the receiver's own statement of how well
         * it can place the pulse, which no derived figure here can replace.
         * ttff and the status flags tell a cold start from a loss of lock. */
        float    gDop         = 0;
        float    pDop         = 0;
        float    hDop         = 0;
        float    vDop         = 0;
        int64_t  clkBiasNs    = 0;
        int32_t  clkDriftNsPerS = 0;
        uint32_t tAccNs       = 0;
        uint32_t fAccPsPerS   = 0;
        uint8_t  navStatusFlags  = 0;
        uint8_t  navStatusFlags2 = 0;
        uint32_t ttffMs       = 0;
        int64_t  dopAgeUs     = -1;
        int64_t  clockAgeUs   = -1;
        int64_t  navStatusAgeUs = -1;

        /* UBX-MON-RF: the same front-end story as MON-HW but per band, so the
         * L1 and L2 halves can disagree. Kept as the worst case over the bands
         * rather than an array, since the status struct is copied by value. */
        uint8_t  rfBands      = 0;
        uint8_t  rfJamWorst   = 0;
        uint8_t  rfAntWorst   = 0;
        uint16_t rfAgcMin     = 0;
        bool     rfSeen       = false;
        int64_t  rfAgeUs      = -1;
        uint8_t  rfCfg        = 0;
        /* Worst and RMS phase excursion in the fit window, nanoseconds. */
        /* How long after the edge the frame that dated it arrived, at the last
         * accepted pair, and the high-water since boot. This is the margin the
         * pairing window is actually spending; pairAgeMaxUs is the figure,
         * because a mean hides the tail that decides whether a sentence could
         * pair with the wrong edge. pairAgeS is how old the measurement is. */
        int64_t  pairAgeUs    = 0;
        int64_t  pairAgeMaxUs = 0;
        double   pairAgeS     = -1;
        double   tieMaxNs     = 0;
        double   tieRmsNs     = 0;
        /* Overlapping Allan deviation in ppb (nanoseconds per second), octave
         * k at 2^k seconds. Zero where the window is too short to reach it, and
         * the count below says how many samples stand behind the numbers. */
        uint32_t ticksPerUs   = 0; /* systimer ticks in a microsecond, as IDF converts it */
        int64_t  tickUsDelta  = 0; /* ticks-derived microseconds minus esp_timer's */

        /* Link health, which was the one thing with no observation at all. At
         * 9600 baud the wire is the scarcest resource here, and until now a
         * saturated or mistimed line showed up only as CRC and length drops
         * with nothing behind them. MON-IO is the port counters (parity,
         * framing, overrun, bytes), the buffer messages are the module's own
         * queue depths, and MSGPP is how many of each class it actually sent --
         * which is also how a silently-ignored enable becomes visible. */
        uint32_t ioRxBytes    = 0;
        uint32_t ioTxBytes    = 0;
        uint16_t ioParity     = 0;
        uint16_t ioFraming    = 0;
        uint16_t ioOverrun    = 0;
        uint16_t rbufPending  = 0;
        uint8_t  rbufPeak     = 0;
        uint16_t tbufPending  = 0;
        uint8_t  tbufPeak     = 0;
        uint8_t  tbufErrors   = 0;
        uint16_t msgUbx       = 0;
        uint32_t msgSkipped   = 0;
        bool     ioSeen       = false;
        int64_t  ioAgeUs      = -1;
        /* The last MSGPP payload as hex. The layout is subtle enough -- port
         * outer, protocol inner -- that being able to see the bytes is worth 240
         * characters of payload. */
        std::string msgppHex;
        /* The last frames by class:id, so "what is it actually sending" is
         * answerable from the API rather than only from the serial console. */
        std::string recentFrames;
        std::string ioHex; /* string in Status; the 241-byte buffer is the member */
        /* The host's current rate, so utilisation can be computed against it. */
        uint32_t baud         = 0;

        /* The servo's own work counters. A mean offset hides a servo fighting
         * the oscillator: these three show it happening. Steps are corrections
         * past the slew threshold, clamps are fits that hit the rate limit (a
         * bad fit, not a bad crystal), and edgeSpikes counts edges that landed
         * more than 1 ms from where the last one predicted -- the signature of
         * a pulse that is being corrupted rather than merely late. */
        uint32_t servoSteps   = 0;
        uint32_t servoClamps  = 0;
        uint32_t edgeSpikes   = 0;
        /* The trust test's verdict on the newest edge, and how many edges in a
         * row it has refused. Spikes are edges that arrived where nothing
         * predicted; rejects are edges that failed the spacing or the
         * second-continuity test and were therefore never used at all. A count
         * climbing towards the re-acquire threshold is the pulse failing
         * slowly, which no single offset figure shows. */
        bool     lastEdgeTrusted = true;
        uint32_t edgeRejects     = 0;
        uint32_t bridgeSessions = 0;
        uint32_t bridgeRefused  = 0;
        /* Whether a client holds the port right now, and how many. The point of
         * saying so is that "u-center does nothing" and "u-center is not
         * attached at all" are different faults and used to look the same from
         * here. */
        bool     bridgeActive  = false;
        int      bridgeClients = 0;
        uint32_t bridgeRx       = 0;
        uint32_t bridgeTx       = 0;
        std::string bridgeEnd;
        std::string bridgeLast;
        /* What CFG-GNSS asked for, and what the receiver made of it. Reported
         * because the answer is site and firmware dependent rather than a
         * constant: BeiDou is refused while GLONASS or Galileo is enabled. */
        std::string gnss;
        std::string gnssCfg;
        /* Time mode: disabled, surveying, or fixed. With the survey's own
         * numbers beside it, since "how far along is it and how good is it" is
         * the whole question once one is running. */
        std::string timeMode;
        double   svinDurS   = 0;
        double   svinAccMm  = 0;
        double   svinVarMm2 = 0;
        double   svinObs    = 0;
        /* The mean the survey has so far, in ECEF centimetres, which is the
         * unit the message carries and the unit CFG-TMODE2 takes. Published
         * because "action":"fix" stores exactly this: without it the decision at
         * the end of a survey is made blind, and a mean a few metres from the
         * provisioned position would be locked over the top of it unremarked. */
        int32_t  svinMeanCm[3] = {};
        /* What the survey was asked for, beside what it has. Without the pair
         * there is no way to tell "nearly there" from "asked for six hours and
         * the limit is unmet", which is the whole decision at the end. */
        double   svinTargetS = 0;
        double   svinLimitMm = 0;
        bool     svinValid  = false;
        /* True only when the held fixed position came from a survey that had
         * met its own accuracy limit. False is usable but unsignoff, and is
         * what a lock taken from an abandoned survey looks like. */
        bool     holdValidated = false;
        /* Whether a hold is wanted and when it will next be asserted. A hold
         * that is wanted but never sent is invisible otherwise, which is how one
         * was restored and dropped on every boot. */
        bool     holdWanted = false;
        double   holdAssertInS = -1;
        /* The three terms the retry waits on, so a hold that never gets sent can
         * be told from one that is simply not due. */
        bool     holdReady = false;
        double   lastEdgeAgeMs = -1;
        double   svinAgeS   = -1;
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

    /* One epoch of the clock, for a client to drain by sequence.
     *
     * The ring exists because the client is a poller: without it a viewer
     * restart costs every epoch in between, and the only stamp a row can carry
     * is when the HTTP reply happened to arrive. With it a row carries the
     * receiver's own second and a sequence number, so a client can prove it
     * missed nothing -- and the box says so when it did.
     *
     * seq advances on every epoch including a holdover, so contiguity is over
     * epochs rather than over good measurements. kind says which this is. */
    struct Sample {
        uint32_t seq;
        uint32_t epoch; /* the receiver second this describes */
        int32_t  offsetNs;
        int32_t  ratePpb;
        uint32_t dispersionNs;
        uint16_t jitterNs;
        uint8_t  flags; /* TIM-TP flags, as published elsewhere */
        uint8_t  kind;  /* 0 measured, 1 gap (no offset), 2 holdover */
        uint8_t  sats;
        uint8_t  fix;
    };
    static constexpr size_t kSampleRing = 900;

    /* Copies up to max records at or after since, oldest first, and reports the
     * range and how far since fell behind the ring. since == 0 means "from the
     * oldest you still have", which is the restart and resync path. Returns the
     * count copied. */
    size_t drainSamples(uint32_t since, Sample *out, size_t max, uint32_t &first, uint32_t &last,
                        uint32_t &lost) const;

    /* The per-satellite sample behind status().cn0Mean, copied into a caller
     * buffer, returning the count copied.
     *
     * Deliberately NOT a member of Status: the array is 256 bytes and Status is
     * returned by value from every status() call, which happens on the NTP
     * responder and on every /healthz. Folding it in grew each of those frames
     * by the whole array and overflowed the 2304-byte sys_evt task, which
     * panicked the board on boot. Here the caller decides where the bytes live
     * and pays for them only on the one endpoint that wants them. */
    uint8_t copySats(Sat *out, uint8_t max) const;

    /* The scalars a reply needs and nothing else. status() assembles nine
     * std::strings for the HTTP side; the responder was paying for all of them
     * between its two stamps, which measured 250 us of allocation inside the
     * T2..T3 window. Same gate logic, evaluated here rather than copied. */
    struct ReplyFacts {
        bool     locked       = false;
        uint8_t  stratum      = 16;
        uint32_t refId        = 0;
        time_t   lastSync     = 0;
        uint8_t  leap         = 0;
        uint32_t rootDelayNs  = 0;
        uint32_t dispersionNs = 0;
        /* Held together, but the budget says it is past what a reply may claim:
         * locked is then false and stratum 16, and this is only the reason. */
        bool     demoted      = false;
    };
    ReplyFacts replyFacts() const;

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
    /* Replace the constellation mask and re-apply it. Bits are gnssId: GPS 0,
     * SBAS 1, Galileo 2, BeiDou 3, IMES 4, QZSS 5, GLONASS 6. A mask the
     * receiver refuses is reported rather than retried, because which
     * combinations it takes is the receiver's business -- BeiDou and GLONASS
     * cannot both be set on this one. */
    void requestGnss(uint8_t mask);
    /* Survey-in of the antenna position, and locking it as a fixed one. A fixed
     * position is what a timing receiver wants: the position stops being solved
     * from the clock, so the pulse stops moving with it. Only worth doing
     * because position error is cheap in time -- 3.3 ns per metre, averaged
     * over the constellation -- so a survey that lands metres is still better
     * than none. */
    void requestSurvey(bool start, uint32_t durS = 86400, uint32_t accMm = 30000);
    void requestFixPosition();
    /* Re-send the pulse frame with a new antenna cable delay, in nanoseconds.
     * Persisted by the caller: this only queues the frame for the task. */
    void requestAntDelay();
    void requestPulseMs(uint32_t ms);
    void requestPeriodUs(uint32_t us);
    /* Set the host side's rate on its own, without telling the receiver. That
     * is how a line that has been desynchronised by something other than this
     * firmware -- or deliberately, to watch the scan -- is put back. */
    void requestBaud(uint32_t baud, bool moduleToo);
    int8_t sendCfgPrt(uint32_t baud);

    /* The same port configuration as an NMEA sentence, for a receiver whose UBX
     * input has been switched off. */
    void sendPubxPortConfig(uint32_t baud);
    int8_t saveCfgIo();
    /* Learn the time mode from the receiver, and put a known position hold back
     * after something has cleared it. */
    void adoptTimeMode();
    void reassertFix();
    bool startSurveyFromSettings();
    void restoreTimeMode();
    void saveTimeMode();
    /* Tell the receiver to hold the adopted position. Separate from adopting it:
     * the two used to be the same act and only one of them was happening at
     * boot. */
    bool sendFixedPosition();

    /* A transparent TCP<->UART1 bridge for u-center, on kBridgePort.
     *
     * UBX has no arbitration: on one UART, whoever writes while another writes
     * collides and whoever reads takes the bytes. There are two ways to have a
     * bench tool and this firmware on the same receiver, and only one of them is
     * available here.
     *
     * The one that is: the client is the single master for the duration, the
     * firmware stops writing anything the client did not cause, and the bridge
     * is the single reader, tapping what it reads into the same parser the
     * clock is driven from. u-center configures the receiver; the firmware
     * consumes the result rather than competing with it. That is why pump()
     * waits instead of reading while a client is attached, and why
     * configureModule() stands aside until the last client has gone.
     *
     * The one that is not: port separation. UBX keeps a separate output rate
     * per port, so a monitor on UART2 could ask for its own messages without
     * disturbing UART1 -- this carrier wires only UART1, so it would need a
     * second pair to the module. If that is ever added, the session's CFG-MSG
     * in bridgeTask() wants to name port 2 rather than port 1. */
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
    static bool IRAM_ATTR ppsCapIsr(mcpwm_cap_channel_handle_t channel,
                                    const mcpwm_capture_event_data_t *edge, void *arg);
    /* Both edge paths end here: one from a plain interrupt stamp, one from the
     * capture's latched tick. The gates are shared so the suppression hook and
     * the spacing rule cannot be bypassed by one of them. */
    void recordEdge(int64_t edgeNs);
    esp_err_t beginCapture();
    esp_err_t beginInterruptEdge();
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
     * answers MON-VER but ignores MON-COMMS) and an order-based guess drifts
     * apart within one boot. pump() keeps parsing while it waits; the GPS task
     * would otherwise be blind for the duration -- and while a client holds the
     * port, pump() only parses what the bridge feeds it, because a second
     * reader would take the client's bytes. */
    int8_t  sendAndWait(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                        uint32_t timeoutUs = 250000);
    int8_t  sendTp5();
    void    pump(int64_t timeoutUs);
    uint8_t m_awaitCls      = 0;
    uint8_t m_awaitId       = 0;
    int8_t  m_awaitResult   = -1; /* -1 pending, 1 ACK, 0 NAK, -2 timed out */

    void logLockChange();

    /* UBX framing, one byte at a time. NMEA sentences and rubbish are skipped
     * by the same loop: the parser only ever looks for 0xB5 0x62. */
    void onFrame(const ubx::Frame &frame) override;
    void onDrop(ubx::Drop why, const ubx::Frame &frame) override;
    void onFrame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
    void onNavPvt(const ubx::Frame &f);
    void onNavTimels(const ubx::Frame &f);
    void onNavTimTp(const ubx::Frame &f);
    void onTimTos(const ubx::Frame &f);
    void onCfgTp5(const ubx::Frame &f);
    void onMonVer(const ubx::Frame &f);
    void onMonHw(const ubx::Frame &f);
    void onNavSat(const ubx::Frame &f);
    void onNavSvin(const ubx::Frame &f);
    void onNavDop(const ubx::Frame &f);
    void onNavClock(const ubx::Frame &f);
    void onNavStatus(const ubx::Frame &f);
    void onMonRf(const ubx::Frame &f);
    void onMonIo(const ubx::Frame &f);
    void onMonRxbuf(const ubx::Frame &f);
    void onMonTxbuf(const ubx::Frame &f);
    void onMonMsgpp(const ubx::Frame &f);
    void onCfgTmode(const ubx::Frame &f);
    void anchor(time_t second);

    /* The servo. Each anchor is one sample of esp_timer against the GPS; the
     * slope of the last five minutes of them is the crystal's rate error, and
     * that is what lets a holdover last minutes instead of seconds. */
    struct RateSample {
        /* The PPS edge in nanoseconds: esp_timer's microseconds plus the
         * sub-microsecond part from the CPU cycle counter. Nanoseconds because
         * the fit's residual is the jitter being measured, and esp_timer alone
         * quantises each stamp to +/-0.5 us -- 289 ns RMS, which is larger than
         * the jitter itself. */
        int64_t monoNs = 0;
        int64_t utcNs  = 0; /* the UTC second that edge began */
    };
    void pushRateSample(int64_t monoNs, int64_t utcNs);

    /* Drop the rate fit and seed it again from the next edge. */
    void resetRateWindow();

    /* Everything the lock decision depends on, so that status() and
     * replyFacts() can share one implementation of it -- including the
     * per-edge debounce, which must advance once per edge whichever caller
     * arrives first. */
    struct LockInputs {
        bool     anchored   = false;
        bool     timeValid  = false;
        uint8_t  fixType    = 0;
        int64_t  ppsAgeUs   = -1;
        bool     trusted    = true;
        int64_t  offsetNs   = 0;
        bool     rateKnown  = false;
        double   tieMaxNs   = 0;
        uint8_t  jamState   = 0;
        uint32_t ppsCount   = 0;
        int64_t  lockWindowUs = 0;
    };
    bool evaluateLock(const LockInputs &in) const;

    /* The stated error budget, in ns. rootDelayNs() is the measurement path --
     * the named interrupt latency plus half the receiver's own pulse
     * quantisation. dispersionNs() is that plus the jitter the fit measures
     * plus the wander of ageUs, and carries the unobserved-pulse term its
     * caller computed. One implementation, used by status() and by
     * recordSample(), so the served figure and the recorded one cannot drift
     * apart. */
    int64_t rootDelayNs() const;
    /* What our own stamping is worth: capture latency, receive lag, quantisation.
     * Dispersion material, not delay -- see rootDelayNs(). */
    int64_t sampleErrorNs() const;
    int64_t unobservedNs() const;
    int64_t dispersionNs(int64_t ageUs, int64_t unobservedNs) const;

    /* Correct the realtime clock by adjtime() rather than by a set. */
    void slew(int64_t offsetNs);
    /* Appends one epoch, filling in any epochs missed since the last one. Runs
     * on the servo task with the same lock discipline as the rate ring. */
    void recordSample(uint32_t epoch, int64_t offsetNs);
    Sample   m_samples[kSampleRing];
    uint32_t m_sampleSeq       = 1; /* seq 0 is the "from the oldest" sentinel */
    uint32_t m_sampleOldest    = 1;
    uint32_t m_lastSampleEpoch = 0;
    bool     m_lastSampleSet   = false;
    void fitRate();

    /* Measure the offset against the anchor, then slew or step the system clock
     * towards it if this source owns CLOCK_REALTIME. The measurement happens
     * whether it owns it or not, so a fallback's divergence stays visible;
     * ownership is ClockSource's to decide, and under timeSource=gps it is
     * always ours. */
    void steer();

    /* The edge, written by the interrupt and read by the task and the responder.
     * A spinlock rather than atomic<int64_t>: 64-bit atomics on this core go
     * through libatomic, and the section is a handful of instructions. Mutable
     * because status() and now() are const and still have to take it. */
    mutable portMUX_TYPE m_mux          = portMUX_INITIALIZER_UNLOCKED;
    /* The capture timer's own counter, and when this handler last ran. The
     * latter separates a real edge from the echo of our own software catch,
     * which is how "now" is read in the same counter. */
    mcpwm_cap_timer_handle_t   m_capTimer   = nullptr;
    mcpwm_cap_channel_handle_t m_capChannel = nullptr;
    int64_t                    m_lastEdgeUs = -1;
    /* The capture timer's clock, in Hz, as reported by the driver. The APB
     * is not the same speed on every part, so this is measured rather than
     * assumed -- 80 MHz was assumed once and the clock stayed on time only
     * because the servo absorbed a factor of two. */
    uint32_t                   m_capHz      = 0;
    /* The capture timer cannot be read, so a second free-running count in the
     * same group supplies "now" with one register access. */
    bool                       m_capNowValid = false;

    int64_t      m_ppsMonoUs    = 0;
    /* The same edge with the sub-microsecond part resolved from the cycle
     * counter. Only the rate fit reads it; everything that sets the clock or
     * measures age keeps the microsecond field. */
    int64_t      m_ppsMonoNs    = 0;
    uint32_t     m_ppsCount     = 0;
    int64_t      m_anchorUtcNs  = 0;
    int64_t      m_anchorMonoUs = 0;
    bool         m_anchorValid  = false;
    time_t       m_lastFix      = 0;
    bool         m_timeValid    = false;
    uint8_t      m_fixType      = 0;
    uint8_t      m_satellites   = 0;
    int64_t      m_offsetUs     = 0;
    int64_t      m_offsetNs     = 0;
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
    ubx::Decoder m_decoder;
    int64_t      m_lastFrameUs = 0;
    uint32_t     m_rescans    = 0;
    uint32_t     m_badChecksumLogged = 0;
    uint32_t     m_badLengthLogged   = 0;
    uint32_t m_pairedCount  = 0; /* the PPS edge the current anchor came from */
    uint32_t m_framesLogged = 0;
    bool     m_stepped      = false;
    bool     m_frameSeen    = false;
    bool     m_fixLogged    = false;
    bool     m_wasLocked    = false;
    bool     m_ackLogged    = false;
    bool     m_noDataLogged = false;
    int      m_baudIndex    = 0;
    /* Rates tried since the pass began; a pass is kCount of them. */
    int      m_scanSteps    = 0;
    /* Where the scan starts and where a wrap returns to: the rate the receiver
     * was last heard on. Bounds-checked against kBauds at begin(). */
    int      m_preferredIndex = 0;
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
    int64_t    m_lastRateMonoNs    = 0;
    bool       m_lastRateMonoSet   = false;
    bool       m_rateLogged        = false;
    int64_t    m_lastLoggedPpb     = 0;
    bool       m_holdoverLogged    = false;
    mutable bool m_pulseUnobservedLogged = false;
    int8_t     m_leapLoggedSeconds = -1;
    int8_t     m_leapLoggedChange  = 0;
    bool       m_timTpLogged       = false;
    bool       m_timTpGoneLogged   = false;
    uint32_t   m_timTpLastCount    = 0;
    int64_t    m_timTpLastUs       = 0;
    bool       m_pulseOnSecondSeen = true;
    bool       m_tp5Acked          = false;
    uint8_t    m_nav5Cfg           = 0; /* 0 unanswered, 1 accepted, 2 rejected */
    bool       m_tp5Naked          = false;
    bool       m_timTpNaked        = false;
    bool       m_timTpAcked        = false;
    /* MON-RF's fate, in the same three states the other optional messages use.
     * It is a newer message than this module's protocol, so "rejected" and
     * "accepted then silent" are both real answers and only this tells them
     * apart -- otherwise the absent fields look like a firmware bug. */
    int8_t     m_rfCfg             = 0; /* 0 unanswered, 1 accepted, 2 rejected */
    uint32_t   m_ioRxBytes         = 0;
    uint32_t   m_ioTxBytes         = 0;
    uint16_t   m_ioParity          = 0;
    uint16_t   m_ioFraming         = 0;
    uint16_t   m_ioOverrun         = 0;
    uint16_t   m_rbufPending       = 0;
    uint8_t    m_rbufPeak          = 0;
    uint16_t   m_tbufPending       = 0;
    uint8_t    m_tbufPeak          = 0;
    uint8_t    m_tbufErrors        = 0;
    uint16_t   m_msgUbx            = 0;
    uint32_t   m_msgSkipped        = 0;
    bool       m_ioSeen            = false;
    int64_t    m_ioAtUs            = -1;
    /* Fixed storage: written per message, and as std::string each byte was a
     * std::format -- 120 allocations per message, in the GPS task, forever. 241
     * holds the 120-byte payload as 240 hex digits plus a terminator, and
     * sizeof() on it is the real buffer, which is what the copies rely on. */
    text::Buf<241> m_msgppHex;
    uint16_t    m_recentFrames[16] = {};
    uint8_t     m_recentAt         = 0;
    text::Buf<241> m_ioHex;
    bool       m_antCfgAcked       = false;
    bool       m_antCfgNaked       = false;
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
    /* Geometry, clock quality and per-band front end, from NAV-DOP, NAV-CLOCK,
     * NAV-STATUS and MON-RF. Ages are stamped from esp_timer so a message that
     * stops is visible as a growing age rather than as a stale number that
     * still looks current. */
    float      m_gDop              = 0;
    float      m_pDop              = 0;
    float      m_hDop              = 0;
    float      m_vDop              = 0;
    /* Whether the last reply was demoted by the dispersion gate, so the log
     * says it once per episode rather than on every reply. */
    mutable bool m_gateLogged = false;
    int64_t    m_clkBiasNs         = 0;
    int32_t    m_clkDriftNsPerS    = 0;
    uint32_t   m_tAccNs            = 0;
    uint32_t   m_fAccPsPerS        = 0;
    uint8_t    m_navStatusFlags    = 0;
    uint8_t    m_navStatusFlags2   = 0;
    uint32_t   m_ttffMs            = 0;
    int64_t    m_dopAtUs           = -1;
    int64_t    m_clockAtUs         = -1;
    int64_t    m_navStatusAtUs     = -1;
    uint8_t    m_rfBands           = 0;
    uint8_t    m_rfJamWorst        = 0;
    uint8_t    m_rfAntWorst        = 0;
    uint16_t   m_rfAgcMin          = 0;
    bool       m_rfSeen            = false;
    int64_t    m_rfAtUs            = -1;
    uint32_t   m_servoSteps        = 0;
    uint32_t   m_servoClamps       = 0;
    uint32_t   m_edgeSpikes        = 0;
    /* The trust test's history: how many edges in a row it has refused, and
     * the module's second at the previous edge (kept separately from m_lastFix,
     * which only advances on an edge that was actually anchored). */
    uint32_t   m_edgeRejects       = 0;
    bool       m_lastEdgeTrusted   = true;
    time_t     m_lastEdgeSecond    = 0;
    bool       m_lastEdgeSecondSet = false;
    int64_t    m_lastEdgeMonoUs    = 0;
    bool       m_lastEdgeMonoSet   = false;
    /* The lock gates' debounce, written from status(), which is const. */
    mutable uint32_t m_gateBad     = 0;
    mutable uint32_t m_gateLastPps = 0;
    /* Phase relative to the fit's base sample, for the deviation below. The
     * absolute offsets are ~1.8e18 ns and a double resolves only ~256 ns there,
     * so a second difference taken on them cancels to exactly zero -- measured
     * on the host before it was believed. Relative, the numbers are seconds and
     * the arithmetic is exact. Static, because 300 doubles does not belong on
     * the GPS task's stack. */
    /* The first 120 payload bytes of a MON frame as hex, for the panel. Both
     * MON-IO and MON-MSGPP carry their numbers past where the eye can follow,
     * and the frame start is what says which message it was. */
    void hexPreview(const ubx::Frame &f, text::Buf<241> &out);

    /* Take a position as the hold, validated or not. Two callers: restoring one
     * from NVS, and adopting one the module reported. */
    void adoptFixedPosition(long long x, long long y, long long z, unsigned long acc,
                            bool validated);

    int64_t    m_pairAgeUs         = 0;
    int64_t    m_pairAgeMaxUs      = 0;
    int64_t    m_pairAgeAtUs       = 0;
    double     m_tieMaxNs          = 0;
    double     m_tieRmsNs          = 0;
    int64_t    m_pulseOffsetNs     = 0; /* where TIM-TP says the pulse sits */
    int64_t    m_qErrAtUs          = 0;  /* when the last qErr arrived */
    int32_t    m_pvtNanoNs         = 0;
    uint32_t   m_pvtItowMs         = 0;
    uint8_t    m_timTpFlags        = 0;
    uint32_t   m_tosFlags          = 0;
    int32_t    m_tosUtcOffset      = 0;
    uint32_t   m_tosUtcUnc         = 0;
    uint8_t    m_tosCfg            = 0;
    uint8_t    m_timTpCfg          = 0;
    int64_t    m_timTpPolledUs     = 0;
    uint32_t   m_timTpTimN         = 0;
    uint32_t   m_timTpNavN         = 0;
    uint8_t    m_rateCfg           = 0;
    int32_t    m_tp5CableNs        = 0;
    int32_t    m_tp5UserNs         = 0;
    uint32_t   m_tp5Flags          = 0;
    uint32_t   m_tp5Period         = 0;
    uint32_t   m_tp5Ratio          = 0;
    uint32_t   m_pulseMs           = 100;
    uint32_t   m_periodUs          = 1000000;
    bool       m_tosLogged         = false;
    uint32_t   m_secFrames         = 0; /* frames in the current second */
    int64_t    m_secStartUs        = 0;
    uint8_t    m_poorSecs          = 0; /* consecutive seconds under the floor */
    /* configureModule() takes about three seconds and its own traffic is the
     * only thing on the wire while it runs. Testing the rate underneath it
     * cancels the configuration that would have raised the rate. */
    int64_t    m_warmRetryAtUs     = 0;
    uint8_t    m_warmRetries       = 0;
    /* Requests from the HTTP task, serviced by the GPS task on its next pass so
     * that only one thread ever writes the UART or advances the parser. */
    volatile int8_t m_biasReq      = -1; /* -1 none, 0 off, 1 on */
    /* Constellation mask, and a flag saying one has been asked for. The mask is
     * one bit per gnssId, which is also how CFG-GNSS orders its blocks: GPS 0,
     * SBAS 1, Galileo 2, BeiDou 3, IMES 4, QZSS 5, GLONASS 6. */
    volatile uint8_t m_gnssReqMask = 0;
    volatile bool    m_gnssReq     = false;
    uint8_t          m_gnssMask    = 0x65; /* GPS | Galileo | QZSS | GLONASS */
    bool             m_gnssAcked   = false;
    bool             m_gnssNaked   = false;
    /* Survey-in (CFG-TMODE3). 0 idle, 1 start, 2 stop, 3 lock the current mean
     * position as a fixed one. The reported position is the receiver's own mean
     * from NAV-SVIN, kept here so "lock it" does not have to invent one. */
    volatile uint8_t m_svinCmd     = 0;
    volatile bool    m_tp5Cmd      = false;
    volatile uint32_t m_baudReq    = 0;
    /* Set when the request also writes CFG-PRT to the receiver. The revert is
     * the safety net: a rate the module does not take leaves it talking into a
     * host listening at the wrong speed, and nothing but the scan would bring
     * it back. The old rate is held until frames resume, then written back. */
    volatile bool     m_baudReqModule = false;
    uint32_t          m_baudPrev   = 0;
    uint32_t          m_baudRevertTo = 0; /* non-zero: write this back when frames resume */
    int64_t           m_baudRevertAtUs = 0;
    bool              m_baudRevertPending = false;
    volatile uint32_t m_svinDurS   = 86400;
    volatile uint32_t m_svinAccMm  = 3000;
    uint8_t          m_tmode        = 0; /* 0 disabled, 1 surveying, 2 fixed */
    int32_t          m_svinMean[3]  = {};
    uint32_t         m_svinDur      = 0;
    uint32_t         m_svinAcc      = 0;
    uint32_t         m_svinObs      = 0;
    bool             m_svinValid    = false;
    bool             m_holdValidated = false;
    bool             m_svinActive   = false;
    int64_t          m_svinAtUs     = 0;
    /* A position hold lives in the module's RAM and this firmware's copy of it
     * does not survive a reboot, so the receiver is asked what its time mode
     * actually is rather than the firmware assuming. When it answers "fixed"
     * the position is kept here, which is what lets a reconfigure put it back
     * after a reset has taken it away. */
    int32_t          m_fixedMean[3] = {};
    uint32_t         m_fixedAcc     = 0;
    /* Two separate questions: is there a position worth putting back, and does
     * anything want it held. Adoption inside configureModule() runs before a
     * re-assert and would otherwise clear the first one by reading the "off"
     * that the reset it is recovering from has just put there. */
    bool             m_fixedValid   = false;
    bool             m_holdWanted   = false;
    /* Set around the reconfigure that follows a reset: the module is often
     * still booting then, so its answer describes the state *before* the hold
     * has been put back and reading it would undo the re-assert. */
    bool             m_skipAdopt    = false;
    /* When the hold next gets asserted at the receiver, and the interval to wait
     * after the last failure. A receiver still booting NAKs the frame, and a
     * receiver left moving spends the survey the hold came from while the box
     * still believes it is holding -- so the assertion is retried rather than
     * attempted once and forgotten. */
    int64_t          m_holdAssertAtUs = 0;
    int64_t          m_holdRetryUs    = 0;
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
    bool           m_reconfigForce = false;
    int64_t         m_reconfigAtUs = 0;
    int64_t    m_monHwAtUs         = 0;
    volatile uint32_t m_bridgeRx   = 0;
    volatile uint32_t m_bridgeTx   = 0;
    volatile uint32_t m_bridgeSessions = 0;
    volatile uint32_t m_bridgeRefused  = 0;
    volatile uint32_t m_bridgeDroppedCrc = 0;
    volatile int      m_bridgeClients  = 0;
    text::Buf<48>  m_bridgeEnd     = {"none"};
    text::Buf<128> m_bridgeLast    = {""};
    uint8_t    m_cn0Max            = 0;
    uint8_t    m_cn0Mean           = 0;
    uint8_t    m_cn0Used           = 0;
    uint8_t    m_satCount          = 0;
    Sat        m_sats[kMaxSats]    = {};
    int64_t    m_navSatAtUs        = 0;
    bool       m_verLogged         = false;
    bool       m_timTpProbed       = false;
    int64_t    m_timTpProbeAtUs    = 0;
    text::Buf<160> m_moduleId      = {};
};

/* The GPS as the responder sees it: stratum 1 and refid "GPS" while the PPS is
 * fresh and the fix is valid, stratum 16 otherwise. */
std::unique_ptr<TimeRef> makeGpsTimeRef();
