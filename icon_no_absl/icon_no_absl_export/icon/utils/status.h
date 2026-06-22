#ifndef ICON_UTILS_STATUS_H_
#define ICON_UTILS_STATUS_H_

#include <algorithm>
#include <array>
#include <string>
#include <string_view>

#include "icon/utils/attributes.h"

namespace intrinsic {

// Same status codes as Abseil, for compatibility.
enum class StatusCode : int {
  kOk = 0,
  kCancelled = 1,
  kUnknown = 2,
  kInvalidArgument = 3,
  kDeadlineExceeded = 4,
  kNotFound = 5,
  kAlreadyExists = 6,
  kPermissionDenied = 7,
  kResourceExhausted = 8,
  kFailedPrecondition = 9,
  kAborted = 10,
  kOutOfRange = 11,
  kUnimplemented = 12,
  kInternal = 13,
  kUnavailable = 14,
  kDataLoss = 15,
  kUnauthenticated = 16,
};

constexpr std::string_view StatusCodeName(StatusCode c) {
  switch (c) {
    case StatusCode::kOk:
      return "OK";
    case StatusCode::kCancelled:
      return "CANCELLED";
    case StatusCode::kUnknown:
      return "UNKNOWN";
    case StatusCode::kInvalidArgument:
      return "INVALID_ARGUMENT";
    case StatusCode::kDeadlineExceeded:
      return "DEADLINE_EXCEEDED";
    case StatusCode::kNotFound:
      return "NOT_FOUND";
    case StatusCode::kAlreadyExists:
      return "ALREADY_EXISTS";
    case StatusCode::kPermissionDenied:
      return "PERMISSION_DENIED";
    case StatusCode::kResourceExhausted:
      return "RESOURCE_EXHAUSTED";
    case StatusCode::kFailedPrecondition:
      return "FAILED_PRECONDITION";
    case StatusCode::kAborted:
      return "ABORTED";
    case StatusCode::kOutOfRange:
      return "OUT_OF_RANGE";
    case StatusCode::kUnimplemented:
      return "UNIMPLEMENTED";
    case StatusCode::kInternal:
      return "INTERNAL";
    case StatusCode::kUnavailable:
      return "UNAVAILABLE";
    case StatusCode::kDataLoss:
      return "DATA_LOSS";
    case StatusCode::kUnauthenticated:
      return "UNAUTHENTICATED";
    default:
      return "INVALID_ERROR_CODE";
  }
}

struct INTR_MUST_USE_RESULT Status {
  StatusCode code = StatusCode::kOk;
  std::string message = "";

  constexpr bool ok() const { return code == StatusCode::kOk; }
};

inline std::string ToString(const Status& s) {
  const std::string_view code_name = StatusCodeName(s.code);
  if (s.message.empty()) {
    return std::string(code_name);
  }

  std::string result;
  result.reserve(code_name.size() + 2 + s.message.size());
  result.append(code_name).append(": ").append(s.message);
  return result;
}

// Allows streaming `Status` to `str`.
//
// This is a template, rather than a regular function that takes `const
// std::ostream&`, to preserve the type of `str`.
//
// Otherwise, it would not be possible to, for instance, use `Status`
// with an "inline" stringstream like so:
//
// ```c++
// Status s = DoNonrealtimeThing();
// if (!s.ok()) {
//   std::string output = (std::stringstream()
//       << "Oh no, something went wrong! " << s).str();
// }
// ```
template <class Ostream>
inline Ostream&& operator<<(Ostream&& str, const Status& status) {
  str << ToString(status);
  return std::forward<Ostream>(str);
}

constexpr Status OkStatus() { return {}; }

struct INTR_MUST_USE_RESULT RealtimeStatus {
  static constexpr size_t kMaxMessageLength = 100;
  // Expect this to be zero terminated.
  using MessageType = std::array<char, kMaxMessageLength>;
  StatusCode code = StatusCode::kOk;
  MessageType message = {0};

  constexpr bool ok() const { return code == StatusCode::kOk; }

  std::string_view GetMessage() const {
    // Find terminator, if any
    return std::string_view(message.begin(),
                            std::find(message.begin(), message.end(), '\0'));
  }
};

constexpr RealtimeStatus RtOkStatus() { return {}; }

inline Status ToStatus(const RealtimeStatus& s) {
  return Status{
      .code = s.code,
      .message = std::string(s.GetMessage()),
  };
}

inline std::string ToString(const RealtimeStatus& s) {
  return ToString(ToStatus(s));
}

// Allows streaming `RealtimeStatus` to `str`.
//
// This is a template, rather than a regular function that takes `const
// std::ostream&`, to preserve the type of `str`.
//
// Otherwise, it would not be possible to, for instance, use `RealtimeStatus`
// with an "inline" stringstream like so:
//
// ```c++
// RealtimeStatus s = DoRealtimeThing();
// if (!s.ok()) {
//   std::string output = (std::stringstream()
//       << "Oh no, something went wrong! " << s).str();
// }
// ```
template <class Ostream>
inline Ostream&& operator<<(Ostream&& str, const RealtimeStatus& status) {
  str << ToString(status);
  return std::forward<Ostream>(str);
}

}  // namespace intrinsic

#endif  // ICON_UTILS_STATUS_H_
