/* How long the PPS edge waited for its interrupt handler.
 *
 * The edge stamps itself when the handler starts, so a stamp can be late for
 * two unrelated reasons: the pulse was late, or the handler was. This tells
 * them apart by latching a timer in hardware at the edge and reading it in the
 * handler, both with no CPU in between. The difference is the interrupt
 * latency, and the smallest one seen is the fixed cost of the two reads.
 *
 * A GPIO event drives a GPTimer capture through the ETM, so neither the GPIO
 * driver nor a second ISR is involved. The timer is only a probe: nothing is
 * corrected with it. */
#pragma once

#include <cstdint>
#include "esp_err.h"

namespace pps_latency {

/* Six buckets, as upper bounds in ns: under 1 us, 1-2, 2-5, 5-10, 10-20, over
 * 20. The last one is unbounded. */
struct Stats {
    uint32_t edges;
    uint32_t unmatched; /* the capture did not land between the reads */
    int64_t  lastNs;
    int64_t  minNs; /* the read overhead: subtract it to get the wait itself */
    int64_t  maxNs;
    uint32_t hist[6];
};

/* Call once, from the core the PPS interrupt is installed on and before the
 * handler is. Returns an error if the timer, the ETM event or the channel
 * cannot be had; the probe is then simply inactive. */
esp_err_t begin(int pin);

/* The stamp the handler took on entry. Reads the hardware capture and the
 * timer, and publishes the difference. Safe with the flash cache off. */
void onEdge(int64_t stampUs);

/* The last edge and its latency, for the anchor to compare with the fit error.
 * False when that stamp is not the last one, or it had no usable capture. */
bool forEdge(int64_t stampUs, int64_t *ns);

/* A consistent copy of the counters. False when the probe is not running. */
bool stats(Stats *out);

}  // namespace pps_latency
