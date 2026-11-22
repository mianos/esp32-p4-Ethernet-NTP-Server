/*
 * The NTP responder: UDP/123, one dual-stack socket, basic and interleaved
 * modes, and mode 6 control. The time and stratum come from a TimeRef: the
 * upstream SNTP clock (stratum 3, the default) or the GPS/PPS reference
 * (stratum 1), chosen by Settings::timeSource.
 *
 *   locked      the TimeRef's stratum, leap from the reference, its refid
 *   not locked  stratum 16, leap indicator 3, refid "INIT"
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <ctime>
#include <memory>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ntp/ntp_control.h"
#include "ntp/tx_lag.h"
#include "clock/time_ref.h"

/* Forward-declared to keep lwIP out of this header. */
struct sockaddr_storage;

/* The upstream SNTP clock as a TimeRef; locked only while its last sync is
 * recent. */
std::unique_ptr<TimeRef> makeUpstreamTimeRef();

/* The TimeRef Settings::timeSource() names, re-read on every call so a config
 * change applies at once. */
std::unique_ptr<TimeRef> makeTimeRef();

class NtpServer {
public:
    static NtpServer &instance();

    /* Bind :123 and start the task. Idempotent; a no-op while serveNtp is off.
     * Called once there is an address. */
    esp_err_t begin();

    /* serveNtp changed under a running device: start or stop accordingly. */
    esp_err_t applySettings();

    bool running() const { return m_running.load(std::memory_order_relaxed); }

    /* Counters for /healthz. */
    uint32_t requests() const { return m_requests.load(std::memory_order_relaxed); }
    uint32_t interleavedServed() const { return m_interleaved.load(std::memory_order_relaxed); }
    /* Requests that could be interleaved (non-zero origin, receive differing
     * from transmit), and those whose origin matched a saved receive stamp. */
    uint32_t interleavedTried() const { return m_ilTry.load(std::memory_order_relaxed); }
    /* Replies sent through sendto() with the TxLag correction; zero while the
     * raw 1588 path is used. */
    uint32_t txCorrected() const { return m_txCorrected.load(std::memory_order_relaxed); }
    uint32_t interleavedHit() const { return m_ilHit.load(std::memory_order_relaxed); }
    uint32_t interleavedSaved() const { return m_ilSaved.load(std::memory_order_relaxed); }
    uint32_t interleavedKeyOnly() const { return m_ilKeyOnly.load(std::memory_order_relaxed); }
    /* TxLag's correction and spread; zero until measured. */
    double txLagUs() const { return TxLag::instance().lagUs(); }
    double txScatterUs() const { return TxLag::instance().scatterUs(); }

    struct Counters {
        uint32_t answered    = 0;  /* cumulative */
        uint32_t kod         = 0;  /* cumulative KoD replies */
        uint32_t dropped     = 0;  /* cumulative: malformed, or rate-limited */
        double   perMinute   = 0;  /* answered per minute, last window */
        double   lastServedS = -1; /* since the last answered request */
        uint8_t  clients     = 0;  /* distinct sources in the recent window */
    };
    Counters counters() const;

    /* What a reply would claim right now (also what /healthz reports). */
    TimeReference reference() const;

    /* Root delay and dispersion a reply would carry now, in us, before 16.16
     * rounding. */
    struct RootFigures {
        double delayUs      = 0.0;
        double dispersionUs = 0.0;
    };
    RootFigures wireRootFigures() const;

private:
    static constexpr const char *TAG = "ntp_server";

    NtpServer() = default;

    /* Task body: receive, decide, answer. Exits when m_running goes false. */
    void serveLoop();

    /* The device facts for a mode 6 reply. */
    void        fillControlFacts(ControlFacts &facts, const TimeReference &ref) const;
    static void taskEntry(void *arg);

    /* Per-client, per-minute extremes of (our receive stamp - client transmit
     * stamp), for ServePath. Responder task only. */
    struct ServePathSlot {
        uint32_t addr   = 0;
        uint32_t count  = 0;
        int64_t  minNs  = 0;
        int64_t  maxNs  = 0;
        int64_t  lastUs = 0;
    };
    static constexpr int kServeSlots          = 6;
    ServePathSlot        m_serve[kServeSlots] = {};
    /* ServePath sample outcomes: kept, wrong family, zero origin, out of
     * range. */
    uint32_t m_serveWhy[4]   = {};
    int64_t  m_serveReportUs = 0;
    void     recordServePath(const sockaddr_storage &from, const struct timespec &received,
                             const uint8_t *origin);
    void     reportServePath(int64_t nowUs);

    std::unique_ptr<TimeRef> m_ref;
    std::atomic<bool>        m_running{false};
    std::atomic<uint32_t>    m_requests{0};
    std::atomic<uint32_t>    m_answered{0};
    std::atomic<uint32_t>    m_kod{0};
    std::atomic<uint32_t>    m_dropped{0};
    std::atomic<uint32_t>    m_control{0};
    std::atomic<uint32_t>    m_interleaved{0};
    std::atomic<uint32_t>    m_ilTry{0};
    std::atomic<uint32_t>    m_txCorrected{0};
    std::atomic<uint32_t>    m_ilHit{0};
    std::atomic<uint32_t>    m_ilSaved{0};
    std::atomic<uint32_t>    m_ilKeyOnly{0};
    TxLag                    m_txLag;
    std::atomic<uint32_t>    m_ratePerMin{0};
    std::atomic<int64_t>     m_lastServedUs{-1};
    std::atomic<uint8_t>     m_clients{0};
    /* Recent source hashes, for the client count. Responder task only. */
    static constexpr uint8_t kRecentSources           = 16;
    uint32_t                 m_recent[kRecentSources] = {};
    uint8_t                  m_recentAt               = 0;
    TaskHandle_t             m_task                   = nullptr;
    int                      m_sock                   = -1;
};
