#pragma once

namespace intrinsic::hal
{

// Simple scope guard class that invokes a callback when it exits scope (unless
// it's been cancelled).
//
// TODO(nilsb): Harden to only accept nothrow-destructible callbacks
class Cleanup {
private:
  std::function<void()> callback_;
  bool active_ = false;

public:
  Cleanup() = delete;
  Cleanup(const Cleanup &) = delete;
  Cleanup(const Cleanup &&) = delete;
  Cleanup & operator=(const Cleanup &) = delete;
  Cleanup & operator=(Cleanup &&) = delete;

  explicit Cleanup(std::function<void()> && cb)
  : callback_(cb), active_(true) {}
  ~Cleanup()
  {
    if (active_) {
      active_ = false;
      callback_();
    }
  }
  void Cancel() &&
  {
    active_ = false;
  }
};

}
