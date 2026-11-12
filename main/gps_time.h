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
 *    last 3 s, so a module that stops pulsing demotes this server to stratum 16
 *    within three seconds instead of holding over silently;
 *  - fall back to SNTP when the fix goes away. A reference that silently turns
 *    into a different reference is one whose stratum means nothing. Settings
 *    says which source is authoritative; if that source is not healthy, the
 *    honest answer is "do not use me";
 *  - pretend to interpolate leap seconds. The u-blox UTC fields already include
 *    them, so the epoch this builds is the same one lwIP's SNTP client sets, and
 *    the leap indicator stays 0. A leap second would be stepped by the module,
 *    not smoothed by us.
 *
 * The receiver is configured over UART on every boot -- UBX-NAV-PVT at 1 Hz,
 * NMEA off -- and deliberately not saved to its flash: the module keeps its own
 * defaults, so it can still be plugged into u-center afterwards. The PPS
 * configuration is likewise left at the module's default (1 Hz, UTC-aligned,
 * rising edge at the top of the second); if it has been changed with u-center,
 * CFG-TP5 wants syncMode UTC and a period of 1 s.
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
    void anchor(time_t second);

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
};

/* The GPS as the responder sees it: stratum 1 and refid "GPS" while the PPS is
 * fresh and the fix is valid, stratum 16 otherwise. */
std::unique_ptr<TimeRef> makeGpsTimeRef();
