/*
 * NTP mode 6 (RFC 9327), read side only, so `ntpq` works: READSTAT, READVAR
 * and READCLOCK. WRITEVAR is refused with "permission denied". The serving
 * reference is the one association, associd 1.
 *
 * ntpq drops a reply that does not echo the request's sequence, opcode and
 * mode, or is not padded to 32 bits. The data is a comma-separated name=value
 * list, so each pair is written whole or not at all.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>

/* The device facts a reply needs, filled by the responder. */
struct ControlFacts {
    /* Which reference is serving, and what the reply would claim about it. */
    bool        upstream = false; /* the upstream, not the receiver */
    std::string peer;             /* the upstream's address; empty for the receiver */
    uint8_t     leap         = 0;
    uint8_t     stratum      = 16;
    uint32_t    refId        = 0;
    bool        locked       = false;
    time_t      lastSync     = 0;
    int64_t     rootDelayNs  = 0;
    int64_t     dispersionNs = 0;
    int64_t     jitterNs     = 0;
    int64_t     offsetNs     = 0;

    /* Counters, from the same totals /healthz publishes. */
    uint32_t requests        = 0;
    uint32_t answered        = 0;
    uint32_t kod             = 0;
    uint32_t dropped         = 0;
    uint32_t controlRequests = 0;
    int64_t  uptimeS         = 0;

    const char *version = ""; /* image version, for sys.version */
};

/* Build one mode 6 datagram from `offset` into the data field and return its
 * length, or 0 for nothing to send. The caller repeats with a larger offset
 * until `total` is reached; `out` must hold the whole data field plus a header,
 * since the list is built whole and then fragmented. */
constexpr size_t kControlChunk      = 468; /* 4-aligned: fragments must be */
constexpr size_t kControlReplyBytes = 12 + 1280 + 3;
size_t           buildControlReply(uint8_t *out, size_t cap, const uint8_t *req, size_t len,
                                   const ControlFacts &facts, uint16_t offset, uint16_t &total);
