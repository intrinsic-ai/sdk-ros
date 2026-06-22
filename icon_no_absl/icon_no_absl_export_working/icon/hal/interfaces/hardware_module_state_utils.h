#ifndef ICON_HAL_INTERFACES_HARDWARE_MODULE_STATE_UTILS_H_
#define ICON_HAL_INTERFACES_HARDWARE_MODULE_STATE_UTILS_H_

#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "icon/utils/attributes.h"

namespace intrinsic_fbs {

INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildHardwareModuleState();

// Updates the code and message of the state.
//
// Expects `hardware_module_state` to have a non-nullptr `message` member!
//
// Truncates `message` if it is longer than the underlying flatbuffer.
void SetState(HardwareModuleState* hardware_module_state, StateCode code,
              std::string_view message);

// Returns the message associated with the given state.
//
// Returns an empty string if `hardware_module_state` or
// `hardware_module_state->message()` is null.
std::string_view GetMessage(const HardwareModuleState* hardware_module_state);

}  // namespace intrinsic_fbs

namespace intrinsic::icon {

enum class TransitionGuardResult { kNoOp, kAllowed, kProhibited };

TransitionGuardResult HardwareModuleTransitionGuard(
    intrinsic_fbs::StateCode from, intrinsic_fbs::StateCode to);

}  // namespace intrinsic::icon
#endif  // ICON_HAL_INTERFACES_HARDWARE_MODULE_STATE_UTILS_H_
