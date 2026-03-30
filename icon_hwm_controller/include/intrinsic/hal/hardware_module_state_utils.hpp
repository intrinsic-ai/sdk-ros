#pragma once 

#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "hwm_fbs/hardware_module_state.fbs.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildHardwareModuleState();

void SetState(HardwareModuleState* hardware_module_state, StateCode code,
              std::string_view message);

// Returns the message associated with the given state.
//
// Returns an empty string if `hardware_module_state` or
// `hardware_module_state->message()` is null.
std::string_view GetMessage(const HardwareModuleState* hardware_module_state);

}  // namespace intrinsic_fbs
