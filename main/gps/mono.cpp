/* The mono counter at the resolution a PPS edge needs: the systimer's own ticks.
 *
 * esp_timer reports microseconds by dividing this counter by 16, so reading it
 * here is a finer look at the same epoch, not a second time base: the rate error
 * the servo measures stays the crystal's, and every consumer that compares a
 * stamp with esp_timer_get_time() still agrees within a microsecond. */
#include "gps/mono.h"

#include <atomic>

#include "esp_log.h"
#include "esp_private/systimer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/systimer_ll.h"
#include "soc/systimer_struct.h"

namespace mono {
namespace {

constexpr const char *TAG = "mono";

/* The P4 wires the systimer to XTAL and divides by a fixed 2.5: 40 MHz becomes
 * 16 MHz, so a tick is 62.5 ns. esp_timer divides the same counter by this to
 * report microseconds. */
constexpr int64_t kTicksPerUs = 16;

std::atomic<bool> s_live{false};

/* systimer_hal_get_counter_value() without the HAL's context: latch, wait for
 * the latch, then read low, high and low again so a carry between the reads
 * cannot tear the value. */
uint64_t IRAM_ATTR ticks()
{
    systimer_ll_counter_snapshot(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    while (!systimer_ll_is_counter_value_valid(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER)) {
    }
    uint32_t low   = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    uint32_t high  = 0;
    uint32_t again = 0;
    do {
        again = low;
        high  = systimer_ll_get_counter_value_high(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        low   = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    } while (low != again);
    return (static_cast<uint64_t>(high) << 32) | low;
}

}  // namespace

int64_t IRAM_ATTR nowNs()
{
    /* Before reading, not after: the wait for the latch inside ticks() would
     * spin here, in the handler, if the counter were not running. */
    if (!s_live.load(std::memory_order_relaxed)) {
        return esp_timer_get_time() * 1000;
    }
    const int64_t now = static_cast<int64_t>(ticks());
    const int64_t us  = now / kTicksPerUs;
    /* The ticks that do not make a microsecond still carry 62.5 ns each. */
    return us * 1000 + (now % kTicksPerUs) * 1000 / kTicksPerUs;
}

bool begin()
{
    /* A wrong divisor would scale every edge, and the servo would answer with a
     * rate error no crystal has. Compare the counter with esp_timer over 20 ms
     * before the first edge is stamped. */
    const int64_t ticksBefore = static_cast<int64_t>(ticks());
    const int64_t usBefore    = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(20));
    const int64_t ticksAfter = static_cast<int64_t>(ticks());
    const int64_t usAfter    = esp_timer_get_time();

    const int64_t deltaTicks = ticksAfter - ticksBefore;
    const int64_t deltaUs    = usAfter - usBefore;
    const int64_t expected   = deltaUs * kTicksPerUs;
    const int64_t error =
        deltaTicks > expected ? deltaTicks - expected : expected - deltaTicks;
    /* One percent: the delay is not exact, and four reads sit between the two
     * samples. */
    const bool ok = deltaUs > 0 && deltaTicks > 0 && error * 100 <= expected;

    s_live.store(ok, std::memory_order_relaxed);
    if (ok) {
        ESP_LOGI(TAG, "mono: %lld ticks in %lld us -- the edge stamp is 62.5 ns",
                 static_cast<long long>(deltaTicks), static_cast<long long>(deltaUs));
    } else {
        ESP_LOGE(TAG,
                 "mono: %lld ticks in %lld us, not %lld a us -- the edge stamp stays "
                 "on esp_timer's microseconds",
                 static_cast<long long>(deltaTicks), static_cast<long long>(deltaUs),
                 static_cast<long long>(kTicksPerUs));
    }
    return ok;
}

}  // namespace mono
