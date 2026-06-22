#include "icon/hal/interfaces/hardware_module_state_utils.h"

#include <algorithm>
#include <cstring>
#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildHardwareModuleState() {
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  builder.Finish(builder.CreateStruct(HardwareModuleState()));
  return builder.Release();
}

void SetState(HardwareModuleState* hardware_module_state, StateCode code,
              std::string_view message) {
  if (hardware_module_state == nullptr) {
    return;
  }
  const size_t copy_length = std::min(
      static_cast<size_t>(hardware_module_state->message()->size() - 1),
      message.size());
  hardware_module_state->mutate_code(code);
  std::memcpy(hardware_module_state->mutable_message()->Data(), message.data(),
              copy_length);
  hardware_module_state->mutable_message()->Data()[copy_length] = '\0';
}

std::string_view GetMessage(const HardwareModuleState* hardware_module_state) {
  if (hardware_module_state == nullptr ||
      hardware_module_state->message() == nullptr) {
    return "";
  }
  return reinterpret_cast<const char*>(
      hardware_module_state->message()->Data());
}
}  // namespace intrinsic_fbs

namespace intrinsic::icon {

TransitionGuardResult HardwareModuleTransitionGuard(
    intrinsic_fbs::StateCode from, intrinsic_fbs::StateCode to) {
  using intrinsic_fbs::StateCode;
  if (to == StateCode::kFatallyFaulted) {
    return TransitionGuardResult::kAllowed;
  }

  switch (from) {
    case StateCode::kPreparing:
      switch (to) {
        case StateCode::kPrepared:
        case StateCode::kFaulted:
        case StateCode::kFatallyFaulted:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }
    case StateCode::kPrepared:
      switch (to) {
        case StateCode::kActivating:
        case StateCode::kDeactivating:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }
    case StateCode::kDeactivating:
      switch (to) {
        case StateCode::kDeactivated:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }
    case StateCode::kDeactivated:
      switch (to) {
        case StateCode::kDeactivating:
          return TransitionGuardResult::kNoOp;
        case StateCode::kPreparing:
        case StateCode::kInitFailed:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kActivating:
      switch (to) {
        case StateCode::kActivated:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kActivated:
      switch (to) {
        case StateCode::kMotionDisabling:
        case StateCode::kClearingFaults:
          return TransitionGuardResult::kNoOp;
        case StateCode::kMotionEnabling:
        case StateCode::kDeactivating:
        case StateCode::kFaulted:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kMotionEnabling:
      switch (to) {
        case StateCode::kDeactivating:
        case StateCode::kMotionEnabled:
        case StateCode::kFaulted:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kMotionEnabled:
      switch (to) {
        case StateCode::kMotionEnabling:
        case StateCode::kClearingFaults:
          return TransitionGuardResult::kNoOp;
        case StateCode::kMotionDisabling:
        case StateCode::kFaulted:
        case StateCode::kDeactivating:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kMotionDisabling:
      switch (to) {
        case StateCode::kFaulted:
        case StateCode::kActivated:
        case StateCode::kDeactivating:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kFaulted:
      switch (to) {
        case StateCode::kClearingFaults:
        case StateCode::kDeactivating:
        case StateCode::kFaulted:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kClearingFaults:
      switch (to) {
        case StateCode::kDeactivating:
        case StateCode::kFaulted:
        case StateCode::kActivated:
          return TransitionGuardResult::kAllowed;
        default:
          return TransitionGuardResult::kProhibited;
      }

    case StateCode::kInitFailed:
      return TransitionGuardResult::kProhibited;

    case StateCode::kFatallyFaulted:
      return TransitionGuardResult::kProhibited;
  }
  return TransitionGuardResult::kProhibited;
}

}  // namespace intrinsic::icon
