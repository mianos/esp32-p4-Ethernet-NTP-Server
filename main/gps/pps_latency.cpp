#include "gps/pps_latency.h"

#include <atomic>
#include <cstring>
#include <inttypes.h>

#include "driver/gpio_etm.h"
#include "driver/gptimer.h"
#include "driver/gptimer_etm.h"
#include "esp_etm.h"
#include "esp_attr.h"
#include "esp_log.h"

namespace pps_latency {
namespace {

constexpr const char *TAG = "pps_latency";

/* Half of PLL_F80M, because the P4's timer prescaler cannot divide by one: 80 MHz
 * asserts in timer_ll_set_clock_prescale. */
constexpr int64_t  kTickNs       = 25;
constexpr uint32_t kResolutionHz = 40000000;

/* An interrupt that took longer than this did not merely wait behind a masked
 * stretch, so the capture belongs to an earlier edge or none at all. */
constexpr int64_t kMaxPlausibleNs = 100000;

/* How much wider than its narrowest a bracket may be before the read inside it
 * cannot be placed. Normal brackets vary by about 100 ns; the stalls seen
 * inside them were 2-4 us. */
constexpr int64_t kStallNs = 500;

/* Upper edges of the buckets, in ns. The sixth bucket is unbounded. */
/* In DRAM: the handler reads it, and .rodata is in flash, gone while OTA writes. */
DRAM_ATTR constexpr int64_t kBucketNs[] = {1000, 2000, 5000, 10000, 20000};
constexpr int               kBuckets    = 6;

gptimer_handle_t         s_timer;
esp_etm_channel_handle_t s_channel;
esp_etm_event_handle_t   s_event;
esp_etm_task_handle_t    s_task;

/* The ISR is the only writer. It brackets a whole update with an odd then an
 * even sequence, because the 64-bit fields are not atomic on RV32 and readers
 * run on the other core. */
std::atomic<uint32_t> s_seq{0};
/* s_timer exists from the first setup step, but an edge read before the capture
 * is wired would record a bogus zero as the floor. */
std::atomic<bool> s_live{false};
uint32_t          s_edges;
uint32_t          s_unmatched;
int64_t           s_lastStampUs;
int64_t           s_lastNs = -1;
int64_t           s_minNs  = INT64_MAX;
int64_t           s_maxNs;
uint32_t          s_hist[kBuckets];

/* Handler-only state: the three reads of this edge and the least each interval
 * has been. The least edge-to-read figures are the floor the correction is
 * measured from, so a normal edge moves by tens of ns, not by the floor. */
uint64_t s_cap;
uint64_t s_read[3];
int64_t  s_minStampAtNs   = INT64_MAX;
int64_t  s_minPhcAtNs     = INT64_MAX;
int64_t  s_minStampSpanNs = INT64_MAX;
int64_t  s_minPhcSpanNs   = INT64_MAX;
int64_t  s_lastStampTrimNs;
int64_t  s_lastPhcLateNs;
int64_t  s_lastPhcReadNs;
uint32_t s_dropped;

void teardown()
{
    if (s_channel != nullptr) {
        esp_etm_channel_disable(s_channel);
        esp_etm_del_channel(s_channel);
        s_channel = nullptr;
    }
    if (s_timer != nullptr) {
        gptimer_stop(s_timer);
        gptimer_disable(s_timer);
    }
    if (s_task != nullptr) {
        esp_etm_del_task(s_task);
        s_task = nullptr;
    }
    if (s_event != nullptr) {
        esp_etm_del_event(s_event);
        s_event = nullptr;
    }
    if (s_timer != nullptr) {
        gptimer_del_timer(s_timer);
        s_timer = nullptr;
    }
}

}  // namespace

esp_err_t begin(const int pin)
{
    if (s_timer != nullptr) {
        return ESP_OK;
    }

    gptimer_config_t timer = {};
    timer.clk_src          = GPTIMER_CLK_SRC_PLL_F80M;
    timer.direction        = GPTIMER_COUNT_UP;
    timer.resolution_hz    = kResolutionHz;
    esp_err_t err          = gptimer_new_timer(&timer, &s_timer);

    if (err == ESP_OK) {
        gpio_etm_event_config_t event = {};
        event.edge                    = GPIO_ETM_EVENT_EDGE_POS;
        err                           = gpio_new_etm_event(&event, &s_event);
    }
    if (err == ESP_OK) {
        err = gpio_etm_event_bind_gpio(s_event, pin);
    }
    if (err == ESP_OK) {
        gptimer_etm_task_config_t task = {};
        task.task_type                 = GPTIMER_ETM_TASK_CAPTURE;
        err                            = gptimer_new_etm_task(s_timer, &task, &s_task);
    }
    if (err == ESP_OK) {
        esp_etm_channel_config_t channel = {};
        err                              = esp_etm_new_channel(&channel, &s_channel);
    }
    if (err == ESP_OK) {
        err = esp_etm_channel_connect(s_channel, s_event, s_task);
    }
    /* The timer is started before the channel is enabled, or an edge in between
     * would capture while it is stopped. */
    if (err == ESP_OK) {
        err = gptimer_enable(s_timer);
    }
    if (err == ESP_OK) {
        err = gptimer_start(s_timer);
    }
    if (err == ESP_OK) {
        err = esp_etm_channel_enable(s_channel);
    }
    if (err != ESP_OK) {
        teardown();
        return err;
    }
    s_live.store(true, std::memory_order_release);

    ESP_LOGI(TAG, "GPIO%d: hardware capture at %" PRIu32 " Hz, latency in %lld ns ticks", pin,
             kResolutionHz, static_cast<long long>(kTickNs));
    return ESP_OK;
}

void IRAM_ATTR beforeStamp()
{
    if (!s_live.load(std::memory_order_acquire)) {
        return;
    }
    /* The captured register first: the raw read is a software capture into that
     * same register and would overwrite it. */
    gptimer_get_captured_count(s_timer, &s_cap);
    gptimer_get_raw_count(s_timer, &s_read[0]);
}

void IRAM_ATTR afterStamp()
{
    if (s_live.load(std::memory_order_acquire)) {
        gptimer_get_raw_count(s_timer, &s_read[1]);
    }
}

void IRAM_ATTR afterPhc()
{
    if (s_live.load(std::memory_order_acquire)) {
        gptimer_get_raw_count(s_timer, &s_read[2]);
    }
}

namespace {

int64_t IRAM_ATTR ticksNs(const uint64_t later, const uint64_t earlier)
{
    return (static_cast<int64_t>(later) - static_cast<int64_t>(earlier)) * kTickNs;
}

void IRAM_ATTR lower(int64_t &least, const int64_t value)
{
    if (value < least) {
        least = value;
    }
}

}  // namespace

Edge IRAM_ATTR finish(const int64_t stampNs, const int64_t phcReadNs)
{
    Edge edge = {stampNs, 0, true};
    if (!s_live.load(std::memory_order_acquire)) {
        return edge;
    }
    const int64_t ns      = ticksNs(s_read[2], s_cap);
    const bool    matched = ns >= 0 && ns <= kMaxPlausibleNs;

    /* What this edge's stamps were corrected by, published below. */
    int64_t stampTrim = 0;
    int64_t phcLate   = 0;

    if (matched) {
        const int64_t stampAtNs   = ticksNs(s_read[0], s_cap);
        const int64_t phcAtNs     = ticksNs(s_read[1], s_cap);
        const int64_t stampSpanNs = ticksNs(s_read[1], s_read[0]);
        const int64_t phcSpanNs   = ticksNs(s_read[2], s_read[1]);
        lower(s_minStampAtNs, stampAtNs);
        lower(s_minPhcAtNs, phcAtNs);
        lower(s_minStampSpanNs, stampSpanNs);
        lower(s_minPhcSpanNs, phcSpanNs);
        /* Each read sits a fixed way into its bracket, so the bracket's start is
         * the read less a constant -- unless the bracket stretched, and then the
         * read may be anywhere in it. */
        edge.placed =
            stampSpanNs - s_minStampSpanNs <= kStallNs && phcSpanNs - s_minPhcSpanNs <= kStallNs;
        stampTrim = stampAtNs - s_minStampAtNs;
        phcLate   = phcAtNs - s_minPhcAtNs;
        edge.edgeNs -= stampTrim;
        edge.phcLateNs = phcLate;
    }

    const uint32_t seq = s_seq.load(std::memory_order_relaxed) + 1;
    s_seq.store(seq, std::memory_order_relaxed);
    s_lastStampUs = edge.edgeNs / 1000;
    s_lastStampTrimNs = stampTrim;
    s_lastPhcLateNs   = phcLate;
    s_lastPhcReadNs   = phcReadNs;
    ++s_edges;
    if (!edge.placed) {
        ++s_dropped;
    }
    if (!matched) {
        ++s_unmatched;
        /* Not this edge's figure, so the next lookup must not use it either. */
        s_lastNs = -1;
    } else {
        s_lastNs = ns;
        if (ns < s_minNs) {
            s_minNs = ns;
        }
        if (ns > s_maxNs) {
            s_maxNs = ns;
        }
        int bucket = 0;
        while (bucket < kBuckets - 1 && ns >= kBucketNs[bucket]) {
            ++bucket;
        }
        ++s_hist[bucket];
    }
    s_seq.store(seq + 1, std::memory_order_release);
    return edge;
}

bool forEdge(const int64_t stampUs, int64_t *ns)
{
    if (!s_live.load(std::memory_order_acquire) || ns == nullptr) {
        return false;
    }
    for (;;) {
        const uint32_t before = s_seq.load(std::memory_order_acquire);
        if ((before & 1u) != 0) {
            continue;
        }
        const bool    match = s_lastNs >= 0 && s_lastStampUs == stampUs;
        const int64_t value = s_lastNs;
        if (s_seq.load(std::memory_order_acquire) == before) {
            if (match) {
                *ns = value;
            }
            return match;
        }
    }
}

bool stats(Stats *out)
{
    if (!s_live.load(std::memory_order_acquire) || out == nullptr) {
        return false;
    }
    for (;;) {
        const uint32_t before = s_seq.load(std::memory_order_acquire);
        if ((before & 1u) != 0) {
            continue;
        }
        Stats copy     = {};
        copy.edges     = s_edges;
        copy.unmatched = s_unmatched;
        copy.dropped   = s_dropped;
        copy.lastNs    = s_lastNs;
        copy.minNs     = s_minNs == INT64_MAX ? 0 : s_minNs;
        copy.maxNs     = s_maxNs;
        copy.stampFloorNs    = s_minStampAtNs == INT64_MAX ? 0 : s_minStampAtNs;
        copy.phcFloorNs      = s_minPhcAtNs == INT64_MAX ? 0 : s_minPhcAtNs;
        copy.lastStampTrimNs = s_lastStampTrimNs;
        copy.lastPhcLateNs   = s_lastPhcLateNs;
        copy.lastPhcReadNs   = s_lastPhcReadNs;
        std::memcpy(copy.hist, s_hist, sizeof copy.hist);
        if (s_seq.load(std::memory_order_acquire) == before) {
            *out = copy;
            return true;
        }
    }
}

}  // namespace pps_latency
