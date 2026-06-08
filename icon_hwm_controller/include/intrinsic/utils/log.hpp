#pragma once

#include <iostream>
#include <utility>
#include <source_location>
#include <functional>
#include <algorithm>

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
}   //namespace intrinsic::log

#define LOG(LEVEL) ::intrinsic::log::LogStream() << ( #LEVEL ": ")


namespace intrinsic::log
{
// This class provides a printf-like logging function.
//
// It requires a SinkCallback
class Logger {
public:
  static constexpr int kMaxLogLineLength = 4096;

  using Severity = int;
  struct LogEntry
  {
    // The actual log message. This does not have a timestamp or severity level.
    // It is also not zero-terminated!
    std::string_view msg;
    std::source_location loc;
    Severity severity;
  };

  using SinkCallback = std::function<void(const LogEntry &)>;

  static constexpr Severity kFatal = 5;
  static constexpr Severity kError = 4;
  static constexpr Severity kWarning = 3;
  static constexpr Severity kInfo = 2;
  static constexpr Severity kDebug = 1;
  static constexpr Severity kNone = 0;

  // Builds a logger that sends log messages with a level <= `log_level`
  // to `sink_cb`
  Logger(Severity log_level, SinkCallback sink_cb)
  :log_level_(log_level), sink_cb_(std::move(sink_cb)) {}

  // Printf style logging. Auto-appends a newline at the end of the line
  template<typename ... Args>
  void Log(std::source_location loc, Severity severity, const char * format, Args &&... args) const
  {
    if (severity > log_level_) {return;}
    if (!sink_cb_) {return;}
    std::array<char, kMaxLogLineLength> l;
    const int bytes_that_would_be_written_or_error =
      std::snprintf(l.data(), kMaxLogLineLength - 1, format, std::forward<Args>(args)...);
    if (bytes_that_would_be_written_or_error < 0) {
      // Possibly do something nicer here, but I'd like to avoid duplicating
      // the compile-time checks that absl::SnPrintf does...
      return;
    }
    const int line_length = std::min(
        // Leave room for trailing newline
        kMaxLogLineLength - 1,
        // std::snprintf returns the number of bytes it *would* have written
        // (minus null terminator) if the buffer was large enough...
        bytes_that_would_be_written_or_error);
    // Append newline
    l.data()[line_length] = '\n';
    sink_cb_({
        .msg = std::string_view(l.data(), line_length),
        .loc = loc,
        .severity = severity});
  }

private:
  int log_level_ = kNone;
  SinkCallback sink_cb_;
};

namespace internal
{

// These two templates allow us to use the same macros for references and
// (nullable) pointers to a Logger.
//
// This makes it much more convenient to log from places that may or may not
// have a logger, rather than forcing callers to use if statements throughout
// their code.

template<typename ... Args>
void LogWith(
  const Logger & logger, std::source_location loc, Logger::Severity severity,
  const char * format, Args &&... args)
{
  logger.Log(loc, severity, format, std::forward<Args>(args)...);
}

template<typename ... Args>
void LogWith(
  const Logger * logger, std::source_location loc, Logger::Severity severity,
  const char * format, Args &&... args)
{
  if (logger == nullptr) {
    return;
  }
  logger->Log(loc, severity, format, std::forward<Args>(args)...);
}

}  // namespace internal
}  // namespace intrinsic::log

// Similar to absl logging, use token concatenation to ensure that severities
// always work without a namespace.
#define INTRINSIC_SHARED_MEMORY_LOG(SEVERITY, LOGGER, FORMAT, ...) \
  INTRINSIC_SHARED_MEMORY_LOG_ ## SEVERITY(LOGGER, FORMAT, __VA_ARGS__)

#define INTRINSIC_SHARED_MEMORY_LOG_DEBUG(LOGGER, FORMAT, ...) \
  ::intrinsic::log::internal::LogWith( \
       LOGGER, std::source_location::current(), \
       ::intrinsic::log::Logger::kDebug, \
       FORMAT __VA_OPT__( , ) __VA_ARGS__);

#define INTRINSIC_SHARED_MEMORY_LOG_INFO(LOGGER, FORMAT, ...) \
  ::intrinsic::log::internal::LogWith( \
       LOGGER, std::source_location::current(), \
       ::intrinsic::log::Logger::kInfo, \
       FORMAT __VA_OPT__( , ) __VA_ARGS__);

#define INTRINSIC_SHARED_MEMORY_LOG_WARNING(LOGGER, FORMAT, ...) \
  ::intrinsic::log::internal::LogWith( \
       LOGGER, std::source_location::current(), \
       ::intrinsic::log::Logger::kWarning, \
       FORMAT __VA_OPT__( , ) __VA_ARGS__);

#define INTRINSIC_SHARED_MEMORY_LOG_ERROR(LOGGER, FORMAT, ...) \
  ::intrinsic::log::internal::LogWith( \
       LOGGER, std::source_location::current(), \
       ::intrinsic::log::Logger::kError, \
       FORMAT __VA_OPT__( , ) __VA_ARGS__);

// FATAL has the distinction of exiting the program (if the sink callback didn't
// already do so)
//
// NOTE: This exits **even if `logger` is nullptr**.
#define INTRINSIC_SHARED_MEMORY_LOG_FATAL(LOGGER, FORMAT, ...) \
  ::intrinsic::log::internal::LogWith( \
       LOGGER, std::source_location::current(), \
       ::intrinsic::log::Logger::kFatal, \
       FORMAT __VA_OPT__( , ) __VA_ARGS__); \
  std::exit(EXIT_FAILURE);
