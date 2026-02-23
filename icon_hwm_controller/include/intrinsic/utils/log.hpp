#pragma once

#include <iostream>
#include <utility>

namespace intrinsic::log
{

// Very basic log stream that logs to std::cerr and automatically adds newlines
// / flushes when exiting scope.
//
// TODO(nilsb): implement more robust logging (thread safety etc)
class LogStream {
public:
  LogStream() = default;
  LogStream(LogStream && other)
  {
    other.moved_from_ = true;
  }
  LogStream & operator=(LogStream && other)
  {
    other.moved_from_ = true;
    return *this;
  }
  LogStream(const LogStream & other) = delete;
  LogStream & operator=(const LogStream & other) = delete;

  // Automatically add a line break when exiting scope.
  // This ensures that log messages are printed immediately, rather than buffered.
  //
  // Because operator<< returns an rvalue of `this`, the destructor only runs at
  // the end of each chain of operator<< thanks to copy elision.
  //
  // But even if copy elision does not take place, the move constructor and
  // assignment operator above ensure that there are no spurious line breaks
  // within a log message.
  ~LogStream()
  {
    if (!moved_from_) {
      std::cerr << std::endl;
    }
  }

  template<typename T>
  LogStream && operator<<(T && t) &&
  {
    std::cerr << std::forward<T>(t);
    return std::move(*this);
  }

private:
  bool moved_from_ = false;
};
}

#define LOG(LEVEL) ::intrinsic::log::LogStream() << ( #LEVEL ": ")
