/*
 * The NTP responder: UDP/123, one socket, 48-byte replies, dual-stack.
 *
 * This is the milestone the rest of the firmware was plumbing for. What it
 * serves is decided by a TimeRef, and that split is the whole design: the reply
 * builder knows the protocol and nothing about where the time came from, so the
 * discipline path (GPS/PPS, in gps_time.cpp) plugs in behind the same three
 * calls instead of editing the packet code.
 *
 * Honesty is a protocol requirement, not politeness. A stratum 1 server whose
 * GPS has lost lock must advertise stratum 16 with the leap indicator at 3 --
 * "do not use me" -- because every client downstream believes whatever it is
 * told. So:
 *
 *   locked      stratum = TimeRef's, leap indicator 0, refid names the reference
 *   not locked  stratum 16,              leap indicator 3, refid "INIT"
 *
 * Two TimeRefs exist: the upstream SNTP clock (stratum 3, the default) and the
 * GPS/PPS reference (stratum 1, gps_time.cpp). Settings::timeSource decides
 * which one is served, and the responder itself does not know the difference --
 * that is the point of the split.
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

/* Only a reference is taken to it, and lwIP is a heavy include for a header. */
struct sockaddr_storage;

/* The upstream SNTP clock as a TimeRef. Locked means the clock was set from an
 * upstream server recently enough to still be worth serving: "was set once, at
 * boot" is not a lock, and a responder that cannot tell the difference will
 * hand out a drifting clock with a straight face. */
std::unique_ptr<TimeRef> makeUpstreamTimeRef();

/* The reference the responder actually serves: whichever Settings::timeSource()
 * names, re-read on every call so POST /config takes effect without swapping
 * anything out from under the serving task. */
std::unique_ptr<TimeRef> makeTimeRef();

class NtpServer {
public:
    static NtpServer &instance();

    /* Bind :123 and start the answering task. Idempotent; a no-op while
     * serveNtp is off, and needs an address, so it is called on the lease. */
    esp_err_t begin();

    /* serveNtp changed under a running device: start or stop accordingly. */
    esp_err_t applySettings();

    bool running() const { return m_running.load(std::memory_order_relaxed); }

    /* Counters for /healthz. Read from the HTTP task while the responder task
     * writes them, hence the atomics. */
    uint32_t requests() const { return m_requests.load(std::memory_order_relaxed); }
    uint32_t interleavedServed() const { return m_interleaved.load(std::memory_order_relaxed); }
    /* How often a request could have been an interleaved one (a non-zero origin
     * and a receive that differs from its transmit), and how often the origin
     * matched a saved receive timestamp. A basic client that echoes the
     * previous transmit counts as a try and never as a hit. */
    uint32_t interleavedTried() const { return m_ilTry.load(std::memory_order_relaxed); }
    /* Replies whose transmit stamp had the socket-lag correction applied. With
     * the raw send that is none of them, and the healthz figure is then a
     * description of the fallback rather than of the wire. */
    uint32_t txCorrected() const { return m_txCorrected.load(std::memory_order_relaxed); }
    uint32_t interleavedHit() const { return m_ilHit.load(std::memory_order_relaxed); }
    uint32_t interleavedSaved() const { return m_ilSaved.load(std::memory_order_relaxed); }
    uint32_t interleavedKeyOnly() const { return m_ilKeyOnly.load(std::memory_order_relaxed); }
    /* Measured transmit-path delay, so a reader can see the correction the
     * replies carry. Zero means "not measured yet".
     *
     * Paired with the spread the same window produced. The two answer different
     * questions -- what a reply is shifted by, and how much that shift is
     * trusted -- and a run that watches only the first cannot tell a path that
     * moved from a loop reporting its own noise. */
    double txLagUs() const { return TxLag::instance().lagUs(); }
    double txScatterUs() const { return TxLag::instance().scatterUs(); }

    /* What the responder has done with requests, beyond counting the answered
     * ones. `answered` alone cannot tell a client that is being rate-limited
     * from one that has stopped asking, and cumulative numbers cannot tell a
     * busy minute from a busy week -- hence the minute rate and the two
     * rejection counters, all of which move on their own. */
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

    /* The root delay and dispersion a reply would carry right now, in
     * microseconds, before the 16.16 field rounds them: the reference's own
     * figures where it states them (the GPS), the LAN guesses where it cannot
     * (the upstream). /healthz reports these so the endpoint and the wire cannot
     * disagree about how stale the claim is. */
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

    /* The device side of a mode-6 reply: which reference is serving, what it
     * claims, and the counters. */
    void        fillControlFacts(ControlFacts &facts, const TimeReference &ref) const;
    static void taskEntry(void *arg);

    /* Per-client, per-minute extremes of (our receive stamp - the client's
     * send stamp). The constant part of that difference is the client's clock
     * offset and the path; the range is how far the served time moved against
     * that client while it was watching, which is the only view of our own
     * error available without hardware stamps. Fixed storage and task-owned. */
    struct ServePathSlot {
        uint32_t addr   = 0;
        uint32_t count  = 0;
        int64_t  minNs  = 0;
        int64_t  maxNs  = 0;
        int64_t  lastUs = 0;
    };
    static constexpr int kServeSlots          = 6;
    ServePathSlot        m_serve[kServeSlots] = {};
    /* Why samples are being dropped, while the instrument settles: family,
     * empty origin, out-of-range delta. Index 0 counts the ones kept. */
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
    /* Recent source hashes, so "how many clients" is a fact rather than a guess
     * from the packet count. Fixed storage: the responder task owns it. */
    static constexpr uint8_t kRecentSources           = 16;
    uint32_t                 m_recent[kRecentSources] = {};
    uint8_t                  m_recentAt               = 0;
    TaskHandle_t             m_task                   = nullptr;
    int                      m_sock                   = -1;
};
