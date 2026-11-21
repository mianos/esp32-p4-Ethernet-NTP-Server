/*
 * NTP mode 6 control queries, the read-only half.
 *
 * RFC 9327 is the reference and calls the interface historic; it is still what
 * `ntpq` speaks, and a server that ignores it looks broken to anyone whose
 * first move is `ntpq -p`. So this answers, and answers only the read side:
 * READSTAT, READVAR and READCLOCK. WRITEVAR is refused with "permission
 * denied" rather than half-applied, because a write here changes nothing that
 * the settings already own.
 *
 * What it reports is the one reference that is serving, as associd 1. There is
 * no selection algorithm to describe, so there is one association rather than
 * a plausible-looking list, and its status word says whether it is reachable so
 * `ntpq -p` hides an unlocked clock the way it hides an unreachable peer.
 *
 * Two constraints from the client decide the shape of the replies:
 *
 *  - `ntpq` drops any response whose sequence number, opcode or mode does not
 *    echo the request, and one whose datagram is not padded to a 32-bit
 *    boundary. Both are enforced here.
 *  - the data field is a comma-separated `name=value` list and the client's
 *    parser splits on commas, so a pair is written whole or not at all: a
 *    truncated pair would be read as the next one's name.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>

/* Everything the reply needs from the device. Filled by the responder so this
 * file stays free of the settings, the servo and the clock. */
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

/* Builds one mode-6 datagram into `out`, starting `offset` octets into the data
 * field, and returns its length. 0 means nothing to send: the request is too
 * short, or speaks a version this device does not answer.
 *
 * A variable list longer than one datagram is split by the caller, which calls
 * this again with a larger offset while `total` (the whole data field) is not
 * reached; the M bit is set on every fragment but the last, so `out` has to
 * hold the whole data field plus a header. */
constexpr size_t kControlChunk      = 468; /* 4-aligned: fragments must be */
constexpr size_t kControlReplyBytes = 12 + 1280 + 3;
size_t           buildControlReply(uint8_t *out, size_t cap, const uint8_t *req, size_t len,
                                   const ControlFacts &facts, uint16_t offset, uint16_t &total);
