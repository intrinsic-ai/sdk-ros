#ifndef ICON_UTILS_CURRENT_CYCLE_H_
#define ICON_UTILS_CURRENT_CYCLE_H_

#include <atomic>
#include <cstdint>

namespace intrinsic::icon {

// Singleton.
// Allows accessing the current cycle count without passing references through
// the ICON stack. The three static methods are thread safe, but it's a bad idea
// to call `SetCurrentCycle()` and `IncrementCurrentCycle()` from multiple
// threads. They will overwrite each other and produce unpredictable results
// based on scheduling.
//
// It is assumed that there is
// * one main control loop in charge calling `SetCurrentCycle()` /
//   `IncrementCurrentCycle()`
// * any number of readers using `GetCurrentCycle()`
//
// The value of current cycle is not guaranteed to be static or continuous.
// It can overflow, or jump (if `SetCurrentCycle()` is called).
//
// Expected Usage:
// * During Init: (optionally) Initial value is set using `SetCurrentCycle()`.
// * During realtime operation:
//   * In the main control loop call `IncrementCurrentCycle()`.
//   * Use `GetCurrentCycle()` to get the current cycle where required.
class Cycle final {
 public:
  Cycle() = delete;
  Cycle(Cycle& other) = delete;
  Cycle(const Cycle& other) = delete;
  void operator=(const Cycle&) = delete;

  // Returns the current cycle.
  static uint64_t GetCurrentCycle() noexcept;

  // Adjusts the value of current cycle.
  static void SetCurrentCycle(uint64_t cycle) noexcept;

  // Increments the value of current cycle while handling overruns.
  static void IncrementCurrentCycle() noexcept;

 private:
  inline static std::atomic_uint64_t current_cycle_ = 0;
};

}  // namespace intrinsic::icon

#endif  // ICON_UTILS_CURRENT_CYCLE_H_
