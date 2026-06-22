#ifndef ICON_INTERPROCESS_REMOTE_TRIGGER_REMOTE_TRIGGER_TEST_COMMON_H_
#define ICON_INTERPROCESS_REMOTE_TRIGGER_REMOTE_TRIGGER_TEST_COMMON_H_

#include <chrono>
#include <thread>

#include "icon/interprocess/remote_trigger/remote_trigger_server.h"

namespace remote_trigger_test_common {

inline bool WaitForServer(intrinsic::icon::RemoteTriggerServer& server) {
  constexpr int kMaxWaitCycles = 10;
  int wait_cycles = 0;
  while (!server.IsStarted() && ++wait_cycles <= kMaxWaitCycles) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return server.IsStarted();
}
}  // namespace remote_trigger_test_common

#endif  // ICON_INTERPROCESS_REMOTE_TRIGGER_REMOTE_TRIGGER_TEST_COMMON_H_
