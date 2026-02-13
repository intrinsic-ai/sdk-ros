#pragma once

#include <string>
#include <string.h>
#include <algorithm>

namespace intrinsic::hal
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
struct Status
{
  StatusCode code = StatusCode::kOk;
  std::string message = "";

  bool ok() const {return code == StatusCode::kOk;}
};

constexpr Status OkStatus() {return {};}

struct RealtimeStatus
{
  static constexpr size_t kMaxMessageLength = 100;
  // Expect this to be zero terminated.
  using MessageType = std::array<char, kMaxMessageLength>;
  StatusCode code = StatusCode::kOk;
  MessageType message = {0};
  std::string_view GetMessage() const
  {
    // Find terminator, if any
    return {message.begin(), std::find(message.begin(), message.end(), '\0')};
  }
};

}
