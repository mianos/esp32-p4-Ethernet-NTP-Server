/*
 * The stratum-1 reference: a u-blox LEA-M8T on UART1, disciplined by its PPS.
 *
 *   module TX  -> GPIO16   UART1 RX
 *   module RX  -> GPIO15   UART1 TX
 *   module PPS -> GPIO14   rising edge
 *
 * NAV-PVT names the second; the PPS edge says when it starts. Each paired edge
 * is an anchor (UTC second, local instant), and served time is the anchor plus
 * elapsed time on the 1588 counter, or on esp_timer when that is not running.
 * The system clock is slewed to the same anchor for logs and HTTP but is not
 * what replies are built from.
 *
 * Lock needs an edge within 3 s, or within 10 minutes once the rate is known,
 * with dispersion growing 1 ppm per second of holdover; otherwise this reference
 * answers stratum 16 (under timeSource=auto, ClockSource may hand over to
 * upstream SNTP). Leap seconds come from NAV-TIMELS and are announced, not
 * smeared.
 *
 * The receiver is configured on every boot; only its IO section is saved.
 * CFG-TP5 is always sent so a pulse changed in u-center cannot go unnoticed.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <ctime>
#include <memory>
#include <string>

#include "driver/mcpwm_cap.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "app/text.h"
#include "clock/time_ref.h"
#include "gps/ubx.h"

class GpsTime : private ubx::Listener {
public:
    /* One UBX-NAV-SAT satellite for the sky view; flags are the receiver's own
     * bits. Multiband receivers list a satellite once per band, so rows are not
     * deduplicated by svId. */
    static constexpr uint8_t kMaxSats = 32;
    struct Sat {
        uint8_t  gnss  = 0;
        uint8_t  sv    = 0;
        uint8_t  cno   = 0;
        int8_t   elev  = 0;
        int16_t  azim  = 0;
        uint32_t flags = 0;
        /* Pseudorange residual, cm: a better multipath indicator than C/N0.
         * 0x8000 in the message means not available. */
        int16_t prRes      = 0;
        bool    prResValid = false;
    };

    /* Published once a second as a snapshot; see publishStatus() and status(). */
    struct StatusData {
        bool     timeValid   = false; /* the NAV-PVT date and time fields are good */
        uint8_t  fixType     = 0;     /* u-blox fix type: 0 none, 2 2D, 3 3D */
        uint8_t  satellites  = 0;
        uint32_t ppsCount    = 0;
        int64_t  ppsAgeUs    = -1; /* since the last edge; -1 means never */
        int64_t  anchorAgeUs = -1; /* since the last edge that was paired and trusted */
        /* The last edge on the 1588 counter; zero before the counter runs. */
        int64_t ppsPtpNs = 0;
        /* Served time from the 1588 anchor minus served time from the esp_timer
         * anchor, for the same edge. Should be near zero. */
        int64_t phcMonoDeltaNs = 0;
        bool    anchored       = false; /* an edge and a NAV-PVT have been paired */
        time_t  lastFix        = 0;     /* UTC second of the last accepted anchor */
        int64_t offsetUs       = 0;     /* GPS minus the system clock, last measured */
        /* The same in ns, fine enough to see a cable delay change. */
        int64_t offsetNs = 0;

        /* 1588 counter minus the edge's second at the last anchor, which every
         * PTP stamp carries, and how often it has been stepped. */
        int64_t  phcPhaseNs = 0;
        uint32_t phcSteps   = 0;

        /* Framing counters, straight off the decoder. */
        uint32_t frames           = 0;
        uint32_t droppedChecksum  = 0;
        uint32_t droppedLength    = 0;
        uint32_t rescans          = 0;
        uint32_t bridgeDroppedCrc = 0;
        bool     locked           = false; /* a stratum-1 reference right now */

        /* esp_timer's rate error against GPS in ppb (positive is fast), the
         * anchors in the fit, and the current lock window. */
        int32_t  ratePpb      = 0;
        uint16_t rateSamples  = 0;
        int64_t  lockWindowUs = 0;

        /* rootDelayNs is 0 (a local refclock has no path). dispersionNs is the
         * stamping allowance, half the pulse quantisation, the served-path
         * bound, fit jitter and wander since the edge. */
        int64_t rootDelayNs  = 0;
        int64_t dispersionNs = 0;
        /* The part of dispersion added while TIM-TP is silent. */
        int64_t pulseUnobservedNs = 0;

        /* From UBX-TIM-TP: the receiver's quantisation error (ps on the wire,
         * ns here), our measured stamping spread, and whether the pulse is on
         * the second. timTpCount shows whether the message is arriving. */
        int32_t  qErrNs        = 0;
        bool     qErrValid     = false;
        int64_t  jitterNs      = 0;
        uint32_t timTpCount    = 0;
        bool     pulseOnSecond = false;
        int64_t  pulseOffsetNs = 0; /* where TIM-TP says the pulse sits, ns */
        int32_t  pvtNanoNs     = 0; /* NAV-PVT's offset into its own second */
        uint32_t pvtItowMs     = 0; /* the same instant, on the GPS time base */
        uint8_t  timTpFlags    = 0; /* timeBase and qErrInvalid live here */
        /* UBX-TIM-TOS, when the receiver supports it. */
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

        /* Antenna bias, switched by the ESP. */
        bool antBiasOn  = false;
        bool antShort   = false;
        bool antLatched = false;
        /* Delays as configured in settings. */
        double antDelayNs  = 0;
        double userDelayNs = 0; /* as stored, not as the module echoes it */

        bool tp5Rejected   = false;
        bool timTpRejected = false;
        bool timTpAcked    = false;

        /* Antenna and front end from UBX-MON-HW, signal from UBX-NAV-SAT. A
         * falling agcCnt with jamState above ok indicates front-end overload. */
        uint8_t  antStatus      = 0; /* MON-HW aStatus: 0 init, 1 dontknow, 2 ok, 3 short, 4 open */
        uint8_t  antPower       = 0; /* MON-HW aPower: 0 off, 1 on, 2 dontknow */
        uint16_t noisePerMs     = 0;
        uint16_t agcCnt         = 0;
        uint8_t  jamInd         = 0;
        uint8_t  jamState       = 0; /* MON-HW flags 2:3: 0 unknown, 1 ok, 2 warning, 3 critical */
        bool     monHwValid     = false;
        bool     antCfgRejected = false;
        bool     antCfgAcked    = false;
        uint8_t  cn0Max         = 0; /* strongest used satellite, dBHz */
        uint8_t  cn0Mean        = 0; /* mean over used satellites */
        uint8_t  cn0Used        = 0; /* how many satellites were used in that sample */
        int64_t  navSatAgeUs    = -1;
        int64_t  monHwAgeUs     = -1;
        int64_t  fitUs          = 0; /* how long the last rate fit took */

        /* UBX-NAV-DOP, UBX-NAV-CLOCK and UBX-NAV-STATUS. */
        float    gDop            = 0;
        float    pDop            = 0;
        float    hDop            = 0;
        float    vDop            = 0;
        int64_t  clkBiasNs       = 0;
        int32_t  clkDriftNsPerS  = 0;
        uint32_t tAccNs          = 0;
        uint32_t fAccPsPerS      = 0;
        uint8_t  navStatusFlags  = 0;
        uint8_t  navStatusFlags2 = 0;
        uint32_t ttffMs          = 0;
        int64_t  dopAgeUs        = -1;
        int64_t  clockAgeUs      = -1;
        int64_t  navStatusAgeUs  = -1;

        /* UBX-MON-RF, worst case over the bands. */
        uint8_t  rfBands    = 0;
        uint8_t  rfJamWorst = 0;
        uint8_t  rfAntWorst = 0;
        uint16_t rfAgcMin   = 0;
        bool     rfSeen     = false;
        int64_t  rfAgeUs    = -1;
        uint8_t  rfCfg      = 0;
        /* Edge-to-NAV-PVT delay at the last pair and its maximum since boot,
         * against kPairWindowUs; pairAgeS is how old that pair is. */
        int64_t pairAgeUs    = 0;
        int64_t pairAgeMaxUs = 0;
        double  pairAgeS     = -1;
        /* Worst and RMS phase excursion in the fit window. */
        double  tieMaxNs     = 0;
        double  tieRmsNs     = 0;
        uint32_t ticksPerUs  = 0; /* systimer ticks in a microsecond, as IDF converts it */

        /* UART link health: MON-IO port counters, MON-RXBUF/TXBUF queue depths,
         * and MON-MSGPP message counts (which also show an ignored enable). */
        uint32_t ioRxBytes   = 0;
        uint32_t ioTxBytes   = 0;
        uint16_t ioParity    = 0;
        uint16_t ioFraming   = 0;
        uint16_t ioOverrun   = 0;
        uint16_t rbufPending = 0;
        uint8_t  rbufPeak    = 0;
        uint16_t tbufPending = 0;
        uint8_t  tbufPeak    = 0;
        uint8_t  tbufErrors  = 0;
        uint16_t msgUbx      = 0;
        uint32_t msgSkipped  = 0;
        bool     ioSeen      = false;
        int64_t  ioAgeUs     = -1;

        uint32_t baud = 0;

        /* Servo counters: steps past kStepThresholdNs, fits clamped at
         * kRateLimitPpb, and edges beyond kEdgeSpikeNs of the prediction. */
        uint32_t servoSteps  = 0;
        uint32_t servoClamps = 0;
        uint32_t edgeSpikes  = 0;
        /* Edges refused for landing beyond kEdgeOutlierNs from the fit. */
        uint32_t edgeOutliers = 0;
        /* Which snapshot this is, and its age when read. */
        uint32_t snapshotSeq   = 0;
        int64_t  snapshotAgeUs = -1;
        /* Whether the newest edge passed the spacing and continuity tests, and
         * the consecutive refusals (re-acquire at kRejectStreakReacquire). */
        bool     lastEdgeTrusted = true;
        uint32_t edgeRejects     = 0;
        uint32_t bridgeSessions  = 0;
        uint32_t bridgeRefused   = 0;
        /* u-center bridge clients and traffic. */
        bool     bridgeActive  = false;
        int      bridgeClients = 0;
        uint32_t bridgeRx      = 0;
        uint32_t bridgeTx      = 0;
        /* Survey-in progress from UBX-TIM-SVIN. */
        double svinDurS   = 0;
        double svinAccMm  = 0;
        double svinVarMm2 = 0;
        double svinObs    = 0;
        /* Survey mean in ECEF cm, the unit CFG-TMODE2 takes; "action":"fix"
         * stores exactly this. */
        int32_t svinMeanCm[3] = {};
        /* The survey's requested duration and accuracy limit. */
        double svinTargetS = 0;
        double svinLimitMm = 0;
        bool   svinValid   = false;
        /* The held position came from a survey that met its accuracy limit. */
        bool holdValidated = false;
        /* Whether a hold is wanted and when it is next asserted. */
        bool   holdWanted    = false;
        double holdAssertInS = -1;
        /* The conditions the hold retry waits on. */
        bool   holdReady     = false;
        double lastEdgeAgeMs = -1;
        double svinAgeS      = -1;
        /* MON-HW pin map, for naming the supervisor pins in CFG-ANT. */
        uint32_t pinSel = 0;
        uint32_t pinDir = 0;
        uint32_t pinVal = 0;

        /* The last holdover's length and the prediction error when the edge
         * came back. */
        int64_t lastHoldoverUs    = 0;
        int64_t lastHoldoverErrNs = 0;

        /* From UBX-NAV-TIMELS: GPS-UTC offset, pending change (+1, -1, 0) and
         * seconds until it. leapValid is false until the first message. */
        bool    leapValid   = false;
        int8_t  leapSeconds = 0;
        int8_t  leapChange  = 0;
        int32_t leapInS     = 0;

        /* Time left on the POST /gps/pps holdover test, which drops edges. */
        int64_t ppsSuppressUs = 0;
    };

    /* The same, with the text fields a reader wants as strings. They are apart
     * because StatusData has to stay plain data: it is what the GPS task
     * publishes into the snapshot ring, and a reader copies it out with no lock
     * at all -- which a std::string cannot survive. */
    struct Status : StatusData {
        /* What the receiver says it is, from UBX-MON-VER, and which of this
         * firmware's two configuration messages it refused. An empty moduleId
         * with both flags false means the question was never answered -- a
         * module that ignores the poll is not the same as one that answered and
         * refused, and TIM-TP needs protocol 15, which an old unit will not
         * have. All three go out over /healthz rather than only to a log that
         * drops bytes at exactly the wrong moment. */
        std::string moduleId;
        /* The last MSGPP payload as hex. The layout is subtle enough -- port
         * outer, protocol inner -- that being able to see the bytes is worth 240
         * characters of payload. */
        std::string msgppHex;
        /* The last frames by class:id, so "what is it actually sending" is
         * answerable from the API rather than only from the serial console. */
        std::string recentFrames;
        std::string ioHex; /* string in Status; the 241-byte buffer is the member */
        std::string bridgeEnd;
        std::string bridgeLast;
        /* What CFG-GNSS asked for, and what the receiver made of it. Reported
         * because the answer is site and firmware dependent rather than a
         * constant: BeiDou is refused while GLONASS or Galileo is enabled. */
        std::string gnss;
        std::string gnssCfg;
        std::string timeMode;
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

    /* The satellites behind status().cn0Mean, into a caller buffer. Kept out of
     * Status, which is copied by value onto small task stacks. */
    uint8_t copySats(Sat *out, uint8_t max) const;

    /* What a reply needs, without status()'s string building, which cost the
     * responder about 250 us between its receive and transmit stamps. */
    struct ReplyFacts {
        bool     locked       = false;
        uint8_t  stratum      = 16;
        uint32_t refId        = 0;
        time_t   lastSync     = 0;
        uint8_t  leap         = 0;
        uint32_t rootDelayNs  = 0;
        uint32_t dispersionNs = 0;
        /* Locked apart from the error budget; locked is then false. */
        bool demoted = false;
    };
    ReplyFacts replyFacts() const;

    /* GPS-UTC from NAV-TIMELS; zero before the first message. */
    int8_t leapSeconds() const { return m_leapSeconds; }

    /* The anchor plus elapsed time on the 1588 counter (esp_timer when it is
     * off). False before the first paired edge. */
    bool now(struct timespec &ts) const;

    /* "GPS/PPS, 3D fix, 9 satellites" or "GPS/PPS (no fix yet)". */
    std::string describe() const;

    /* NTP leap indicator: 1 or 2 in the minute before a leap NAV-TIMELS
     * announced, otherwise 0. */
    uint8_t leapIndicator() const;

    /* POST /gps/pps: ignore PPS edges for this long, to exercise holdover with
     * the wire attached. Returns the seconds now in force. */
    uint32_t suppressPps(uint32_t seconds);

    /* Bench requests, carried out by the GPS task: antenna bias on or off, a
     * receiver reset (hot by default), and factory defaults. */
    void requestAntennaBias(bool on);
    void requestReset(bool cold = false, bool hardware = false, bool powerCycle = false);
    void requestDefaults();
    /* Constellation mask by gnssId: GPS 0, SBAS 1, Galileo 2, BeiDou 3, IMES 4,
     * QZSS 5, GLONASS 6. A mask the receiver refuses is reported, not retried. */
    void requestGnss(uint8_t mask);
    /* Survey-in, then hold the position fixed so it is no longer solved
     * together with the clock. */
    void requestSurvey(bool start, uint32_t durS = 86400, uint32_t accMm = 30000);
    void requestFixPosition();
    /* Queue CFG-TP5 with the delays from settings; the caller saves them. */
    void requestAntDelay();
    void requestPulseMs(uint32_t ms);
    void requestPeriodUs(uint32_t us);
    /* Change the host baud rate, and the module's too when moduleToo is set. */
    void   requestBaud(uint32_t baud, bool moduleToo);
    int8_t sendCfgPrt(uint32_t baud);

    /* CFG-PRT as a PUBX sentence, for a receiver with UBX input turned off. */
    void   sendPubxPortConfig(uint32_t baud);
    int8_t saveCfgIo();
    /* Read the time mode from the receiver, and restore a known hold. */
    void adoptTimeMode();
    void reassertFix();
    bool startSurveyFromSettings();
    void restoreTimeMode();
    void saveTimeMode();
    /* Send CFG-TMODE2 for the adopted position. */
    bool sendFixedPosition();

    /* TCP<->UART1 bridge for u-center. One UART has no arbitration, so while a
     * client is attached it is the only writer and the bridge the only reader;
     * the bridge feeds what it reads to the parser, pump() stops reading, and
     * configureModule() waits for the client to leave. */
    static constexpr int kBridgePort = 3333;

    /* Antenna bias on the ESP side: AADET_N low powers the preamp, SCS1_N
     * reports a short. The pin does the switching and cuts the bias on a short;
     * CFG-ANT is sent as well. */
    static constexpr int kAntDrivePin = 17;
    static constexpr int kAntSensePin = 18;

private:
    static constexpr const char *TAG = "gps_time";

    GpsTime() = default;

    static void           ppsIsr(void *arg);
    static bool IRAM_ATTR ppsCapIsr(mcpwm_cap_channel_handle_t        channel,
                                    const mcpwm_capture_event_data_t *edge, void *arg);
    /* Both edge paths end here, so the suppression hook and spacing rule apply
     * to each. The PHC reading is taken by the handler before any lock. */
    void        recordEdge(int64_t edgeNs, int64_t edgePhcNs);
    esp_err_t   beginCapture();
    esp_err_t   beginInterruptEdge();
    esp_err_t   beginEdge();
    esp_err_t   beginEdgeOnTimingCore();
    static void taskEntry(void *arg);
    static void bridgeTaskEntry(void *arg);
    void        bridgeTask();

    void task();
    void advanceBaud();
    void configureModule();
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

    /* Send one configuration message and wait for its ACK or NAK. One at a
     * time, because the receiver does not answer every message and matching by
     * order goes wrong. pump() keeps parsing while it waits. */
    int8_t  sendAndWait(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                        uint32_t timeoutUs = 250000);
    int8_t  sendTp5();
    void    pump(int64_t timeoutUs);
    uint8_t m_awaitCls    = 0;
    uint8_t m_awaitId     = 0;
    int8_t  m_awaitResult = -1; /* -1 pending, 1 ACK, 0 NAK, -2 timed out */

    void logLockChange();

    /* Decoded UBX frames; anything not starting 0xB5 0x62 is skipped. */
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

    /* The servo: the slope of esp_timer against GPS over kRateWindow anchors is
     * the crystal's rate error. */
    struct RateSample {
        /* The edge on esp_timer in ns, with the interrupt wait taken out by
         * pps_latency. */
        int64_t monoNs = 0;
        int64_t utcNs  = 0; /* the UTC second that edge began */
    };
    void pushRateSample(int64_t monoNs, int64_t utcNs);

    /* Drop the rate fit and seed it again from the next edge. */
    void resetRateWindow();

    /* Inputs to the lock decision, shared by status() and replyFacts(). The
     * per-edge debounce advances once per edge whichever calls first. */
    struct LockInputs {
        bool     anchored     = false;
        bool     timeValid    = false;
        uint8_t  fixType      = 0;
        int64_t  ppsAgeUs     = -1;
        bool     trusted      = true;
        int64_t  offsetNs     = 0;
        bool     rateKnown    = false;
        double   tieMaxNs     = 0;
        uint8_t  jamState     = 0;
        uint32_t ppsCount     = 0;
        int64_t  lockWindowUs = 0;
    };
    bool evaluateLock(const LockInputs &in) const;

    /* The error budget in ns, shared by status() and recordSample().
     * sampleErrorNs(): kIsrLatencyNs plus half the pulse quantisation.
     * dispersionNs(): that plus the ServePath bound, fit jitter, wander over
     * ageUs and the unobserved-pulse term. rootDelayNs() is 0. */
    int64_t rootDelayNs() const;

    int64_t sampleErrorNs() const;
    int64_t unobservedNs() const;
    int64_t dispersionNs(int64_t ageUs, int64_t unobservedNs) const;

    void slew(int64_t offsetNs);
    /* Append one epoch, filling any missed since the last. */
    void   recordSample(uint32_t epoch, int64_t offsetNs);
    Sample m_samples[kSampleRing];
    /* /gps/samples reads without a lock: a record is complete once the
     * sequence has passed it. */
    std::atomic<uint32_t> m_sampleSeq{1}; /* seq 0 is the "from the oldest" sentinel */
    std::atomic<uint32_t> m_sampleOldest{1};
    uint32_t              m_lastSampleEpoch = 0;
    bool                  m_lastSampleSet   = false;
    void                  fitRate();

    /* Measure the system clock against the anchor, and slew or step it when
     * ClockSource says this reference owns it. */
    void steer();

    /* Guards the edge and anchor. A spinlock rather than atomic<int64_t>, which
     * on RV32 goes through libatomic. */
    mutable portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
    /* Capture path state. m_lastEdgeUs also tells a real edge from the echo of
     * the software catch. */
    mcpwm_cap_timer_handle_t   m_capTimer   = nullptr;
    mcpwm_cap_channel_handle_t m_capChannel = nullptr;
    int64_t                    m_lastEdgeUs = -1;
    /* Capture timer clock as the driver reports it; it differs between parts. */
    uint32_t m_capHz = 0;

    bool m_capNowValid = false;

    int64_t m_ppsMonoUs = 0;
    /* The same edge in ns, latency-corrected; only the rate fit reads it. */
    int64_t m_ppsMonoNs = 0;
    /* Last silent-baud reset sweep, to rate-limit it. */
    int64_t  m_lastSweepUs  = 0;
    int64_t  m_ppsPtpNs     = 0;
    uint32_t m_ppsCount     = 0;
    int64_t  m_anchorUtcNs  = 0;
    int64_t  m_anchorMonoUs = 0;
    /* The anchor on the 1588 counter, which served time runs from; zero when
     * the counter is off. */
    int64_t  m_anchorPhcNs = 0;

    /* PHC phase step state, servo task only. m_phcStepLagNs is what a step's
     * read-to-set gap loses, learned from the edge after each step. */
    int64_t  m_phcPhaseNs     = 0;
    int      m_phcOffEdges    = 0;
    uint32_t m_phcSteps       = 0;
    int64_t  m_phcStepLagNs   = 0;
    bool     m_phcJustStepped = false;

    bool     m_anchorValid = false;
    time_t   m_lastFix     = 0;
    bool     m_timeValid   = false;
    uint8_t  m_fixType     = 0;
    uint8_t  m_satellites  = 0;
    int64_t  m_offsetUs    = 0;
    int64_t  m_offsetNs    = 0;
    int32_t  m_ratePpb     = 0; /* the servo's answer, used by now() */
    uint16_t m_rateSamples = 0;

    int8_t  m_leapSeconds        = 0;
    int8_t  m_leapChange         = 0;
    int32_t m_leapInS            = 0;
    bool    m_leapValid          = false;
    int64_t m_ppsSuppressUntilUs = 0; /* the POST /gps/pps hook */
    int64_t m_lastHoldoverUs     = 0;
    int64_t m_lastHoldoverErrNs  = 0;

    /* From TIM-TP, plus m_jitterNs, the stamping spread taken from the fit's
     * residuals. */
    uint32_t m_timTpCount    = 0;
    int32_t  m_qErrNs        = 0;
    bool     m_qErrValid     = false;
    bool     m_pulseOnSecond = false;
    int64_t  m_jitterNs      = 0;
    int64_t  m_fitUs         = 0;

    /* GPS task only. */
    ubx::Decoder m_decoder;
    int64_t      m_lastFrameUs       = 0;
    uint32_t     m_rescans           = 0;
    uint32_t     m_badChecksumLogged = 0;
    uint32_t     m_badLengthLogged   = 0;
    uint32_t     m_pairedCount       = 0; /* the PPS edge the current anchor came from */
    uint32_t     m_framesLogged      = 0;
    bool         m_stepped           = false;
    bool         m_frameSeen         = false;
    bool         m_fixLogged         = false;
    bool         m_wasLocked         = false;
    bool         m_ackLogged         = false;
    bool         m_noDataLogged      = false;
    int          m_baudIndex         = 0;
    /* Rates tried in the current pass. */
    int m_scanSteps    = 0;
    int m_silentPasses = 0; /* whole passes of the list with no frame */
    /* The rate the receiver was last heard on, where the scan starts. */
    int      m_preferredIndex = 0;
    uint32_t m_baud           = 0;
    bool     m_begun          = false;
    int64_t  m_nextScanUs     = 0;
    int64_t  m_nextCheckUs    = 0;

    /* Five minutes: long enough for a ppb-level fit, short enough that
     * temperature wander is not fitted as a line. */
    static constexpr int kRateWindow             = 300;
    static constexpr int kRateMinSamples         = 60;
    RateSample           m_rate[kRateWindow]     = {};
    int                  m_rateCount             = 0; /* how many slots are filled */
    int                  m_rateNext              = 0; /* where the next sample goes */
    int64_t              m_lastRateOffsetNs      = 0;
    bool                 m_lastRateOffsetSet     = false;
    int64_t              m_lastRateMonoNs        = 0;
    bool                 m_lastRateMonoSet       = false;
    bool                 m_rateLogged            = false;
    int64_t              m_lastLoggedPpb         = 0;
    bool                 m_holdoverLogged        = false;
    mutable bool         m_pulseUnobservedLogged = false;
    int8_t               m_leapLoggedSeconds     = -1;
    int8_t               m_leapLoggedChange      = 0;
    bool                 m_timTpLogged           = false;
    bool                 m_timTpGoneLogged       = false;
    uint32_t             m_timTpLastCount        = 0;
    int64_t              m_timTpLastUs           = 0;
    bool                 m_pulseOnSecondSeen     = true;
    bool                 m_tp5Acked              = false;
    uint8_t              m_nav5Cfg               = 0; /* 0 unanswered, 1 accepted, 2 rejected */
    bool                 m_tp5Naked              = false;
    bool                 m_timTpNaked            = false;
    bool                 m_timTpAcked            = false;
    /* MON-RF may be newer than the module's protocol version. */
    int8_t   m_rfCfg       = 0; /* 0 unanswered, 1 accepted, 2 rejected */
    uint32_t m_ioRxBytes   = 0;
    uint32_t m_ioTxBytes   = 0;
    uint16_t m_ioParity    = 0;
    uint16_t m_ioFraming   = 0;
    uint16_t m_ioOverrun   = 0;
    uint16_t m_rbufPending = 0;
    uint8_t  m_rbufPeak    = 0;
    uint16_t m_tbufPending = 0;
    uint8_t  m_tbufPeak    = 0;
    uint8_t  m_tbufErrors  = 0;
    uint16_t m_msgUbx      = 0;
    uint32_t m_msgSkipped  = 0;
    bool     m_ioSeen      = false;
    int64_t  m_ioAtUs      = -1;
    /* 120-byte payloads as hex plus a terminator, in fixed buffers to avoid
     * allocating per message. */
    text::Buf<241> m_msgppHex;
    uint16_t       m_recentFrames[16] = {};
    uint8_t        m_recentAt         = 0;
    text::Buf<241> m_ioHex;
    bool           m_antCfgAcked = false;
    bool           m_antCfgNaked = false;
    uint8_t        m_antStatus   = 0;
    uint8_t        m_antPower    = 0;
    uint16_t       m_noisePerMs  = 0;
    uint16_t       m_agcCnt      = 0;
    uint8_t        m_jamInd      = 0;
    uint8_t        m_jamState    = 0;
    uint32_t       m_pinSel      = 0;
    uint32_t       m_pinDir      = 0;
    uint32_t       m_pinVal      = 0;
    uint32_t       m_pinsLogged  = 0;
    bool           m_monHwValid  = false;
    int64_t        m_nextHwUs    = 0;
    /* NAV-DOP, NAV-CLOCK, NAV-STATUS and MON-RF, with arrival times so a message
     * that stops shows as a growing age. */
    float m_gDop = 0;
    float m_pDop = 0;
    float m_hDop = 0;
    float m_vDop = 0;
    /* Log the dispersion-gate demotion once per episode. */
    mutable bool m_gateLogged      = false;
    int64_t      m_clkBiasNs       = 0;
    int32_t      m_clkDriftNsPerS  = 0;
    uint32_t     m_tAccNs          = 0;
    uint32_t     m_fAccPsPerS      = 0;
    uint8_t      m_navStatusFlags  = 0;
    uint8_t      m_navStatusFlags2 = 0;
    uint32_t     m_ttffMs          = 0;
    int64_t      m_dopAtUs         = -1;
    int64_t      m_clockAtUs       = -1;
    int64_t      m_navStatusAtUs   = -1;
    uint8_t      m_rfBands         = 0;
    uint8_t      m_rfJamWorst      = 0;
    uint8_t      m_rfAntWorst      = 0;
    uint16_t     m_rfAgcMin        = 0;
    bool         m_rfSeen          = false;
    int64_t      m_rfAtUs          = -1;
    uint32_t     m_servoSteps      = 0;
    uint32_t     m_servoClamps     = 0;
    uint32_t     m_edgeSpikes      = 0;
    uint32_t     m_edgeOutliers    = 0;
    /* Edge trust test state. m_lastEdgeSecond is the module's second at the
     * previous edge; m_lastFix only advances on anchored edges. */
    uint32_t m_edgeRejects       = 0;
    bool     m_lastEdgeTrusted   = true;
    time_t   m_lastEdgeSecond    = 0;
    bool     m_lastEdgeSecondSet = false;
    int64_t  m_lastEdgeMonoUs    = 0;
    bool     m_lastEdgeMonoSet   = false;
    /* Lock gate debounce, written from const status(): the edge count last
     * voted on in the low 24 bits, the run of bad votes above. */
    mutable std::atomic<uint32_t> m_gate{0};

    /* ---- The published status ----------------------------------------- */

    /* What status() needs beyond StatusData: raw times the reader turns into
     * ages, and text in fixed buffers. */
    struct StatusExtras {
        int64_t  ppsMonoUs        = 0;
        int64_t  anchorMonoUs     = 0;
        int64_t  anchorPhcNs      = 0;
        int64_t  suppressUntil    = 0;
        int64_t  navSatAtUs       = 0;
        int64_t  pairAgeAtUs      = 0;
        int64_t  monHwAtUs        = 0;
        int64_t  dopAtUs          = 0;
        int64_t  clockAtUs        = 0;
        int64_t  navStatusAtUs    = 0;
        int64_t  rfAtUs           = 0;
        int64_t  ioAtUs           = 0;
        int64_t  holdAssertAtUs   = 0;
        int64_t  lastEdgeUs       = -1;
        int64_t  svinAtUs         = 0;
        int64_t  qErrAtUs         = 0;
        int64_t  publishedUs      = 0;
        uint32_t number           = 0;
        uint32_t svinDur          = 0;
        uint32_t svinAcc          = 0;
        uint32_t svinObs          = 0;
        uint32_t svinDurS         = 0;
        uint32_t svinAccMm        = 0;
        uint16_t recentFrames[16] = {};
        uint8_t  recentAt         = 0;
        uint8_t  gnssMask         = 0;
        uint8_t  tmode            = 0;
        uint8_t  satCount         = 0;
        bool     gnssNaked        = false;
        bool     gnssAcked        = false;
        char     msgpp[241]       = {};
        char     io[241]          = {};
        char     module[160]      = {};
        char     bridgeEnd[48]    = {};
        char     bridgeLast[128]  = {};
    };
    struct StatusSlot {
        std::atomic<uint32_t> seq{0}; /* odd while the GPS task is writing it */
        StatusData            data;
        StatusExtras          extra;
    };
    /* With four slots, the one a reader copies is not rewritten for three
     * seconds. */
    static constexpr uint32_t kStatusSlots = 4;
    StatusSlot                m_statusRing[kStatusSlots];
    std::atomic<uint32_t>     m_statusHead{0}; /* the newest finished slot */
    uint32_t                  m_statusNumber      = 0;
    int64_t                   m_statusPublishedUs = 0;
    bool                      m_statusDue         = false;
    void                      publishStatus();

    /* What the PPS interrupt writes, read under the m_edgeSeq sequence count. */
    struct EdgeWords {
        uint32_t count  = 0;
        int64_t  monoUs = 0;
        int64_t  ptpNs  = 0;
        int64_t  lastUs = -1;
    };
    std::atomic<uint32_t> m_edgeSeq{0};
    void                  readEdge(EdgeWords &out) const;
    /* The first 120 payload bytes of a MON frame as hex, for the web panel. */
    void hexPreview(const ubx::Frame &f, text::Buf<241> &out);

    /* Take a position as the hold: restored from NVS or reported by the module. */
    void adoptFixedPosition(long long x, long long y, long long z, unsigned long acc,
                            bool validated);

    int64_t  m_pairAgeUs     = 0;
    int64_t  m_pairAgeMaxUs  = 0;
    int64_t  m_pairAgeAtUs   = 0;
    double   m_tieMaxNs      = 0;
    double   m_tieRmsNs      = 0;
    int64_t  m_pulseOffsetNs = 0; /* where TIM-TP says the pulse sits */
    int64_t  m_qErrAtUs      = 0; /* when the last qErr arrived */
    int32_t  m_pvtNanoNs     = 0;
    uint32_t m_pvtItowMs     = 0;
    uint8_t  m_timTpFlags    = 0;
    uint32_t m_tosFlags      = 0;
    int32_t  m_tosUtcOffset  = 0;
    uint32_t m_tosUtcUnc     = 0;
    uint8_t  m_tosCfg        = 0;
    uint8_t  m_timTpCfg      = 0;
    int64_t  m_timTpPolledUs = 0;
    uint32_t m_timTpTimN     = 0;
    uint32_t m_timTpNavN     = 0;
    uint8_t  m_rateCfg       = 0;
    int32_t  m_tp5CableNs    = 0;
    int32_t  m_tp5UserNs     = 0;
    uint32_t m_tp5Flags      = 0;
    uint32_t m_tp5Period     = 0;
    uint32_t m_tp5Ratio      = 0;
    uint32_t m_pulseMs       = 100;
    uint32_t m_periodUs      = 1000000;
    bool     m_tosLogged     = false;
    uint32_t m_secFrames     = 0; /* frames in the current second */
    int64_t  m_secStartUs    = 0;
    uint8_t  m_poorSecs      = 0; /* consecutive seconds under the floor */
    /* Frame-rate checks pause while configureModule() runs, since its own
     * traffic is all that is on the wire. */
    int64_t m_warmRetryAtUs = 0;
    uint8_t m_warmRetries   = 0;
    /* Requests from the HTTP task. The GPS task carries them out so only one
     * task writes the UART or runs the parser. */
    volatile int8_t m_biasReq = -1; /* -1 none, 0 off, 1 on */
    /* Constellation mask, one bit per gnssId (see requestGnss()). */
    volatile uint8_t m_gnssReqMask = 0;
    volatile bool    m_gnssReq     = false;
    uint8_t          m_gnssMask    = 0x65; /* GPS | Galileo | QZSS | GLONASS */
    bool             m_gnssAcked   = false;
    bool             m_gnssNaked   = false;
    /* Survey command: 0 idle, 1 start, 2 stop, 3 hold the current mean
     * (CFG-TMODE2; the mean comes from TIM-SVIN). */
    volatile uint8_t  m_svinCmd = 0;
    volatile bool     m_tp5Cmd  = false;
    volatile uint32_t m_baudReq = 0;
    /* A baud change that also writes CFG-PRT keeps the old rate so it can be
     * restored if the module stops talking at the new one. */
    volatile bool     m_baudReqModule     = false;
    uint32_t          m_baudPrev          = 0;
    uint32_t          m_baudRevertTo      = 0; /* non-zero: write this back when frames resume */
    int64_t           m_baudRevertAtUs    = 0;
    bool              m_baudRevertPending = false;
    volatile uint32_t m_svinDurS          = 86400;
    volatile uint32_t m_svinAccMm         = 3000;
    uint8_t           m_tmode             = 0; /* 0 disabled, 1 surveying, 2 fixed */
    int32_t           m_svinMean[3]       = {};
    uint32_t          m_svinDur           = 0;
    uint32_t          m_svinAcc           = 0;
    uint32_t          m_svinObs           = 0;
    bool              m_svinValid         = false;
    bool              m_holdValidated     = false;
    bool              m_svinActive        = false;
    int64_t           m_svinAtUs          = 0;
    /* The hold the receiver reported, kept so it can be re-sent after a reset
     * clears it. */
    int32_t  m_fixedMean[3] = {};
    uint32_t m_fixedAcc     = 0;
    /* Kept apart because adoption in configureModule() would otherwise read the
     * "off" a reset left behind and drop the position before it is re-sent. */
    bool m_fixedValid = false;
    bool m_holdWanted = false;
    /* Skip adoption in the reconfigure after a reset, when the module still
     * reports its pre-hold state. */
    bool m_skipAdopt = false;
    /* Next hold assertion and the current retry interval; a booting receiver
     * NAKs it. */
    int64_t               m_holdAssertAtUs = 0;
    int64_t               m_holdRetryUs    = 0;
    volatile bool         m_resetReq       = false;
    volatile bool         m_resetCold      = false;
    volatile bool         m_resetHard      = false;
    volatile bool         m_resetPower     = false;
    volatile bool         m_defaultsReq    = false;
    volatile bool         m_bridgeActive   = false;
    bool                  m_antBiasOn      = false;
    bool                  m_antShort       = false;
    bool                  m_antLatched     = false;
    int                   m_shortSamples   = 0;
    int                   m_senseLogged    = -1;
    void                  applyBias(bool on);
    bool                  m_reconfigForce    = false;
    int64_t               m_reconfigAtUs     = 0;
    int64_t               m_monHwAtUs        = 0;
    volatile uint32_t     m_bridgeRx         = 0;
    volatile uint32_t     m_bridgeTx         = 0;
    volatile uint32_t     m_bridgeSessions   = 0;
    volatile uint32_t     m_bridgeRefused    = 0;
    volatile uint32_t     m_bridgeDroppedCrc = 0;
    volatile int          m_bridgeClients    = 0;
    text::Buf<48>         m_bridgeEnd        = {"none"};
    text::Buf<128>        m_bridgeLast       = {""};
    uint8_t               m_cn0Max           = 0;
    uint8_t               m_cn0Mean          = 0;
    uint8_t               m_cn0Used          = 0;
    uint8_t               m_satCount         = 0;
    std::atomic<uint32_t> m_satsSeq{0}; /* odd while the rows are being rewritten */
    Sat                   m_sats[kMaxSats] = {};
    int64_t               m_navSatAtUs     = 0;
    bool                  m_verLogged      = false;
    bool                  m_timTpProbed    = false;
    int64_t               m_timTpProbeAtUs = 0;
    text::Buf<160>        m_moduleId       = {};
};

/* The GPS reference for the responder: stratum 1, refid "GPS" when locked. */
std::unique_ptr<TimeRef> makeGpsTimeRef();
