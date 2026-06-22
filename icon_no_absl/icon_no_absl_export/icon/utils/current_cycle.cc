#include "icon/utils/current_cycle.h"

#include <atomic>

namespace intrinsic::icon {

uint64_t Cycle::GetCurrentCycle() noexcept {
  return current_cycle_.load(std::memory_order_acquire);
}

void Cycle::SetCurrentCycle(uint64_t cycle) noexcept {
  current_cycle_.store(cycle, std::memory_order_release);
}

void Cycle::IncrementCurrentCycle() noexcept {
  // Since current_cycle_ is unsigned, it overflows to zero automatically,
  // like one would expect (cf.
  // https://en.cppreference.com/cpp/language/operator_arithmetic#Overflows)
  current_cycle_.fetch_add(1, std::memory_order_acq_rel);
}

}  // namespace intrinsic::icon
