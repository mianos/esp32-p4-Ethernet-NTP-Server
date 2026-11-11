/*
 * The NTP responder: UDP/123, one socket, 48-byte replies, dual-stack.
 *
 * This is the milestone the rest of the firmware was plumbing for. What it
 * serves is decided by a TimeRef, and that split is the whole design: the reply
 * builder knows the protocol and nothing about where the time came from, so the
 * discipline path (GPS/PPS next) plugs in behind the same three calls instead
 * of editing the packet code.
 *
 * Honesty is a protocol requirement, not politeness. A stratum 1 server whose
 * GPS has lost lock must advertise stratum 16 with the leap indicator at 3 --
 * "do not use me" -- because every client downstream believes whatever it is
 * told. So:
 *
 *   locked      stratum = TimeRef's, leap indicator 0, refid names the reference
 *   not locked  stratum 16,              leap indicator 3, refid "INIT"
 *
 * Until GPS/PPS lands, the only TimeRef is the upstream server's clock, which
 * makes this a stratum 3 server: the upstream's own stratum is not visible
 * through lwIP's SNTP client, and claiming closer than we know is the worse
 * error of the two.
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

/* What the responder may claim, and what it needs to say it. */
struct TimeReference {
    bool     locked   = false; /* false => stratum 16 + leap indicator 3 */
    uint8_t  stratum  = 16;    /* 1 with a real reference; 3 from upstream SNTP */
    uint32_t refId    = 0;     /* NTP reference id, network order */
    time_t   lastSync = 0;     /* when the reference last agreed with us */
};

/* Where the time comes from. One implementation today (the upstream SNTP
 * clock); GPS/PPS becomes a second one and the responder does not change. */
class TimeRef {
public:
    virtual ~TimeRef() = default;

    virtual TimeReference reference() const = 0;

    /* The instant to stamp a reply with. */
    virtual void now(struct timespec &ts) const = 0;

    /* One phrase for the boot log: "upstream 131.84.1.1", "GPS/PPS". */
    virtual std::string describe() const = 0;
};

/* The only reference there is until the GPS arrives. Locked means the clock was
 * set from an upstream server recently enough to still be worth serving: "was
 * set once, at boot" is not a lock, and a responder that cannot tell the
 * difference will hand out a drifting clock with a straight face. */
std::unique_ptr<TimeRef> makeUpstreamTimeRef();

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
     * writes them, hence the atomic. */
    uint32_t requests() const { return m_requests.load(std::memory_order_relaxed); }

    /* What a reply would claim right now (also what /healthz reports). */
    TimeReference reference() const;

private:
    static constexpr const char *TAG = "ntp_server";

    NtpServer() = default;

    /* Task body: receive, decide, answer. Exits when m_running goes false. */
    void serveLoop();
    static void taskEntry(void *arg);

    std::unique_ptr<TimeRef>   m_ref;
    std::atomic<bool>          m_running{false};
    std::atomic<uint32_t>      m_requests{0};
    TaskHandle_t               m_task = nullptr;
    int                        m_sock = -1;
};
