#ifndef ICON_UTILS_CLEANUP_H_
#define ICON_UTILS_CLEANUP_H_

#include <type_traits>

#include "icon/utils/attributes.h"

namespace intrinsic::icon {

// Simple scope guard class that invokes a callback when it exits scope (unless
// it's been cancelled).
//
// Example:
// ```c++
// auto fd = open("my_file.txt", O_RDONLY);
// if (fd == -1) return;
// Cleanup c([fd]() noexcept { close(fd); });
// // Read from file, with potential early returns for error handling. The
// // Cleanup will `close()` the file no matter how we leave the function.
// ```
//
// Substitute for
// https://github.com/abseil/abseil-cpp/blob/master/absl/cleanup/cleanup.h
template <class Fn>
class INTR_MUST_USE_RESULT Cleanup {
 public:
  static_assert(std::is_nothrow_invocable_r_v<void, Fn>,
                "The template argument of Cleanup must be a function that "
                "takes no arguments, returns void and is marked `noexcept`");
  static_assert(
      std::is_nothrow_destructible_v<Fn>,
      "The template argument of Cleanup must be nothrow destructible`");

  Cleanup() = delete;
  Cleanup(const Cleanup&) = delete;
  Cleanup(Cleanup&&) = delete;
  Cleanup& operator=(const Cleanup&) = delete;
  Cleanup& operator=(Cleanup&&) = delete;

  explicit Cleanup(Fn&& cb) : callback_(std::forward<Fn>(cb)), active_(true) {}
  ~Cleanup() {
    if (active_) {
      active_ = false;
      callback_();
    }
  }

  // Cancels execution of the callback.
  //
  // Note that in order to cancel a Cleanup, you must `std::move` it!
  void Cancel() && { active_ = false; }

 private:
  Fn callback_;
  bool active_ = false;
};

}  // namespace intrinsic::icon

#endif  // ICON_UTILS_CLEANUP_H_
