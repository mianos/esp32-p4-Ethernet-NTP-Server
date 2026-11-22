/* PPS interrupt latency, and the stamps with its variable part removed.
 *
 * A GPIO ETM event captures a GPTimer at the edge; the handler reads the same
 * timer, and the difference is its latency. The floor (the fixed path) cancels
 * in the fit; the excess over it is subtracted from the stamps. */
#pragma once

#include <cstdint>
#include "esp_err.h"

namespace pps_latency {

/* Six buckets, as upper bounds in ns: under 1 us, 1-2, 2-5, 5-10, 10-20, over
 * 20. The last one is unbounded. */
struct Stats {
    uint32_t edges;
    uint32_t unmatched; /* the capture did not land between the reads */
    uint32_t dropped;   /* a clock read was itself stalled, so the edge was not used */
    int64_t  lastNs;
    int64_t  minNs; /* the floor: the fixed path, which the fit absorbs */
    int64_t  maxNs;
    uint32_t hist[6];
};

/* Call once, from the core the PPS interrupt is installed on and before the
 * handler is. Returns an error if the timer, the ETM event or the channel
 * cannot be had; the probe is then simply inactive. */
esp_err_t begin(int pin);

/* The edge as the handler should record it. */
struct Edge {
    int64_t edgeNs;    /* the esp_timer stamp, less this edge's wait above the floor */
    int64_t phcLateNs; /* the same for the PHC read, to subtract from it */
    bool    placed;    /* false: a read stalled between its brackets, so drop the edge */
};

/* The handler's three probe reads, around its esp_timer stamp and its PHC read.
 * A stall inside a bracket cannot be placed, so the brackets are what decide
 * whether the edge is used at all. */
void beforeStamp();
void afterStamp();
void afterPhc();

/* Publishes this edge's figures and returns its corrected stamps. Without a
 * running probe or a usable capture the stamps come back as taken. Safe with
 * the flash cache off. */
Edge finish(int64_t stampUs);

/* The last edge and its latency, for the anchor to compare with the fit error.
 * Keyed by the corrected stamp in us, as recorded. False when that stamp is not
 * the last one, or it had no usable capture. */
bool forEdge(int64_t stampUs, int64_t *ns);

/* A consistent copy of the counters. False when the probe is not running. */
bool stats(Stats *out);

}  // namespace pps_latency
