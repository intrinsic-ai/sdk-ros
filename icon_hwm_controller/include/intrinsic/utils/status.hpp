#pragma once

#include <iostream>
#include <array>
#include <sstream>
#include <string>
#include <algorithm>

namespace intrinsic
{

// same status codes as Abseil, for compatibility.
enum class StatusCode : int
{
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

constexpr std::string_view StatusCodeName(StatusCode c)
{
  switch (c) {
    case StatusCode::kOk:
      return "Ok";
    case StatusCode::kCancelled:
      return "Cancelled";
    case StatusCode::kUnknown:
      return "Unknown";
    case StatusCode::kInvalidArgument:
      return "Invalid Argument";
    case StatusCode::kDeadlineExceeded:
      return "Deadline Exceeded";
    case StatusCode::kNotFound:
      return "Not Found";
    case StatusCode::kAlreadyExists:
      return "Already Exists";
    case StatusCode::kPermissionDenied:
      return "Permission Denied";
    case StatusCode::kResourceExhausted:
      return "Resource Exhausted";
    case StatusCode::kFailedPrecondition:
      return "Failed Precondition";
    case StatusCode::kAborted:
      return "Aborted";
    case StatusCode::kOutOfRange:
      return "Out Of Range";
    case StatusCode::kUnimplemented:
      return "Unimplemented";
    case StatusCode::kInternal:
      return "Internal";
    case StatusCode::kUnavailable:
      return "Unavailable";
    case StatusCode::kDataLoss:
      return "Data Loss";
    case StatusCode::kUnauthenticated:
      return "Unauthenticated";
    default:
      return "Invalid Error Code";
  }
}


struct Status
{
  StatusCode code = StatusCode::kOk;
  std::string message = "";

  bool ok() const {return code == StatusCode::kOk;}
};

inline std::string ToString(const Status & s)
{
  std::stringstream sstr;
  sstr << StatusCodeName(s.code);
  if (!s.message.empty()) {
    sstr << ": " << s.message;
  }
  return sstr.str();
}

template<class Ostream>
inline Ostream && operator<<(Ostream && str, const Status & status)
{
  str << StatusCodeName(status.code);
  if (!status.message.empty()) {
    str << ": " << status.message;
  }
  return std::forward<Ostream>(str);
}

constexpr Status OkStatus() {return {};}

struct RealtimeStatus
{
  static constexpr size_t kMaxMessageLength = 100;
  // Expect this to be zero terminated.
  using MessageType = std::array<char, kMaxMessageLength>;
  StatusCode code = StatusCode::kOk;
  MessageType message = {0};

  bool ok() const {return code == StatusCode::kOk;}

  std::string_view GetMessage() const
  {
    // Find terminator, if any
    return std::string_view(message.begin(), std::find(message.begin(), message.end(), '\0'));
  }
};

template<class Ostream>
inline Ostream && operator<<(Ostream && str, const RealtimeStatus & status)
{
  str << StatusCodeName(status.code);
  if (!status.message.empty()) {
    str << ": " << status.GetMessage();
  }
  return std::forward<Ostream>(str);
}

constexpr RealtimeStatus RtOkStatus() {return {};}

inline Status ToStatus(const RealtimeStatus & s)
{
  return Status{
    .code = s.code,
    .message = std::string(s.GetMessage()),
  };
}

}  // namespace intrinsic
