#include "icon/interprocess/binary_futex.h"

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstring>
#include <ctime>

#include "icon/utils/strerror.h"

namespace intrinsic {

namespace {

inline long futex(std::atomic<uint32_t>* uaddr, int futex_op, uint32_t val,
                  bool private_futex, const struct timespec* timeout = nullptr,
                  uint32_t* uaddr2 = nullptr, uint32_t val3 = 0) {
  if (private_futex) {
    // This option bit can be employed with all futex operations. It tells the
    // kernel that the futex is process-private and not shared with another
    // process (i.e., it is being used for synchronization only between threads
    // of the same process). This allows the kernel to make some
    // additional performance optimizations.
    futex_op |= FUTEX_PRIVATE_FLAG;
  }
  return syscall(SYS_futex, uaddr, futex_op, val, timeout, uaddr2,
                 FUTEX_BITSET_MATCH_ANY);
}

// Atomically reads from `val` and sets `val` to kReady *if* it was kPosted.
//
// Returns the value of `val` before the reset.
uint32_t TryWait(std::atomic<uint32_t>& val) {
  uint32_t expected = BinaryFutex::kPosted;
  // If `val != expected`, then this writes the _actual_ value of `val` into
  // `expected`.
  (void)val.compare_exchange_strong(expected, BinaryFutex::kReady);
  return expected;
}

// Wait until `val` changes from kReady to something else, or until
// `ts_absolute`.
//
// `ts_absolute` must be an absolute timeout in CLOCK_MONOTONIC.
RealtimeStatus Wait(std::atomic<uint32_t>& val, const timespec* ts_absolute,
                    bool private_futex) {
  const Time start_time = Now();
  while (true) {
    uint32_t value = TryWait(val);
    switch (value) {
      case BinaryFutex::kReady: {
        break;
      }
      case BinaryFutex::kClosed: {
        RealtimeStatus status{StatusCode::kAborted, {0}};
        (void)std::snprintf(status.message.data(), status.message.size(),
                            "BinaryFutex is closed, aborting Wait()");
        return status;
      }
      case BinaryFutex::kPosted: {
        return RtOkStatus();
      }
      default: {
        RealtimeStatus status{StatusCode::kInternal, {0}};
        (void)std::snprintf(status.message.data(), status.message.size(),
                            "BinaryFutex took unexpected value: %d", value);
        return status;
      }
    }

    // The value is not yet what we expect, but still kReady. Let's wait for it
    // to change. FUTEX_WAIT_BITSET sleeps for as long as the value is still
    // equal to kReady.
    //
    // N.B.: FUTEX_WAIT_BITSET interprets `ts_absolute` as an absolute timeout
    // in CLOCK_MONOTONIC (it would use CLOCK_REALTIME if we set
    // FUTEX_CLOCK_REALTIME).
    auto ret = futex(&val, FUTEX_WAIT_BITSET, BinaryFutex::kReady,
                     private_futex, ts_absolute);
    if (ret == -1 && errno == ETIMEDOUT) {
      RealtimeStatus status{.code = StatusCode::kDeadlineExceeded};
      (void)std::snprintf(
          status.message.data(), status.message.size(), "Timeout after %lld ms",
          static_cast<long long int>(
              std::chrono::duration_cast<std::chrono::milliseconds>(Now() -
                                                                    start_time)
                  .count()));
      return status;
    }
    if (ret == -1 && errno != EAGAIN && errno != EINTR) {
      RealtimeStatus status{.code = StatusCode::kInternal};
      (void)std::snprintf(status.message.data(), status.message.size(),
                          "Futex wait failed with error: %s",
                          StrError(errno).data());
      return status;
    }
  }
}

}  // namespace

BinaryFutex::BinaryFutex(bool posted, bool private_futex)
    : val_(posted ? kPosted : kReady), private_futex_(private_futex) {}

// Make sure to transfer the futex value when moving a BinaryFutex. We cannot
// transmit the actual futex (since that operates on a fixed address, and we do
// not want a heap-allocated std::unique_ptr here), but we _can_ transfer the
// value.
BinaryFutex::BinaryFutex(BinaryFutex&& other) : val_(other.val_.load()) {}
BinaryFutex& BinaryFutex::operator=(BinaryFutex&& other) {
  if (this != &other) {
    val_.store(other.val_.load());
  }
  return *this;
}

BinaryFutex::~BinaryFutex() { Close(); }

RealtimeStatus BinaryFutex::Post() {
  uint32_t expected = kReady;
  // We need to make a copy of the private_futex_ member variable. Otherwise,
  // we might end up with a data race if the thread destructing the futex
  // reads `val_` between the `compare_exchange_strong` and the
  // `futex` call.
  const bool private_futex = private_futex_;
  // Take the address before, since the class instance could be destroyed before
  // `futex()` is called.
  std::atomic<uint32_t>* val_addr = &val_;
  if (val_.compare_exchange_strong(expected, kPosted)) {
    // `futex` could fail with EFAULT, if `val_addr` is not a valid
    // user-space address anymore. This can happen if another thread destroyed
    // the BinaryFutex between the previous line and this one. Therefore, we
    // ignore the return value here. Another error value should only be EINVAL,
    // which should never happen here
    // ("EINVAL: The kernel detected an inconsistency between the user-space
    // state at uaddr and the kernel state—that is, it detected a waiter which
    // waits in FUTEX_LOCK_PI or FUTEX_LOCK_PI2 on uaddr.").
    //
    // One indicating that we wake up at most 1 other client.
    (void)futex(val_addr, FUTEX_WAKE, 1, private_futex);
  } else if (expected == kClosed) {
    // When compare_exchange_strong returns false, it writes the current value
    // of the atomic into `expected`. We can check this to see if the
    // BinaryFutex is closed.
    return RealtimeStatus{
        .code = StatusCode::kAborted,
        .message = {"BinaryFutex is closed, aborting Post()"},
    };
  }
  return RtOkStatus();
}

RealtimeStatus BinaryFutex::WaitUntil(Time deadline) const {
  if (deadline == Time::max()) {
    return Wait(val_, nullptr, private_futex_);
  }

  // Convert `deadline` from a chrono timepoint (in the steady_clock domain)
  // to a timespec (in the CLOCK_MONOTONIC domain).

  // First calculate how far in the future `deadline` is
  auto duration = deadline - Now();
  // Next, read the current CLOCK_MONOTONIC timepoint using clock_gettime()
  struct timespec now_ts;
  if (clock_gettime(CLOCK_MONOTONIC, &now_ts) != 0) {
    RealtimeStatus status{.code = StatusCode::kInternal};
    (void)std::snprintf(status.message.data(), status.message.size(),
                        "clock_gettime failed with error: %d", errno);
    return status;
  }
  // Add the duration to the current time to get the absolute deadline in the
  // CLOCK_MONOTONIC domain.
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration - secs);

  struct timespec deadline_ts;
  deadline_ts.tv_sec = now_ts.tv_sec + secs.count();
  deadline_ts.tv_nsec = now_ts.tv_nsec + ns.count();

  // Carry the nanoseconds into the seconds
  if (deadline_ts.tv_nsec >= 1'000'000'000) {
    deadline_ts.tv_sec += 1;
    deadline_ts.tv_nsec -= 1'000'000'000;
  }

  return Wait(val_, &deadline_ts, private_futex_);
}

RealtimeStatus BinaryFutex::WaitFor(std::chrono::nanoseconds timeout) const {
  if (timeout == std::chrono::nanoseconds::max()) {
    return Wait(val_, nullptr, private_futex_);
  }

  return WaitUntil(Now() + timeout);
}

std::optional<bool> BinaryFutex::TryWait() const {
  switch (::intrinsic::TryWait(val_)) {
    case kReady:
      return false;
    case kPosted:
      return true;
    case kClosed:
    default:
      return std::nullopt;
  }
}

uint32_t BinaryFutex::Value() const { return val_.load(); }

void BinaryFutex::Close() {
  // We need to make a copy of the private_futex_ member variable. Otherwise,
  // we might end up with a data race if the thread destructing the futex
  // reads `val_` between the `compare_exchange_strong` and the
  // `futex` call.
  const bool private_futex = private_futex_;
  // Take the address before, since the class instance could be destroyed before
  // `futex()` is called.
  std::atomic<uint32_t>* val_addr = &val_;
  // Only signal waiters if the value changed (i.e. if it *wasn't already*
  // BinaryFutex::kClosed).
  if (val_.exchange(kClosed) != kClosed) {
    // `futex` could fail with EFAULT, if `val_addr` is not a valid
    // user-space address anymore. This can happen if another thread destroyed
    // the BinaryFutex between the previous line and this one. Therefore, we
    // ignore the return value here. Another error value should only be EINVAL,
    // which should never happen here
    // ("EINVAL: The kernel detected an inconsistency between the user-space
    // state at uaddr and the kernel state—that is, it detected a waiter which
    // waits in FUTEX_LOCK_PI or FUTEX_LOCK_PI2 on uaddr.").
    //
    // INT_MAX indicates that we wake up *all* waiters. We don't do this in
    // `Post()`, because waking an unknown number of waiters could take unbound
    // time and compromise realtime correctness, but since `Close()` happens on
    // shutdown, we're not worried about that here.
    (void)futex(val_addr, FUTEX_WAKE, INT_MAX, private_futex);
  }
}

}  // namespace intrinsic
