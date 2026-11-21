#pragma once

/* The PPS edge is stamped when its interrupt handler starts, so every stretch
 * of masked interrupts on the handler's core lands in the stamp. Core 1 is
 * therefore kept for that interrupt alone and all tasks run on core 0, which
 * takes their driver interrupts with them: an interrupt belongs to the core
 * that installed it.
 *
 * What IDF still runs on core 1 and cannot be moved: its tick, its idle task,
 * and the IPC task that parks the core for a flash write. */
namespace cores {
constexpr int kTiming = 1;
constexpr int kWork   = 0;
}  // namespace cores
