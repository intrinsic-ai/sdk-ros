#ifndef ICON_HAL_ICON_STATE_REGISTER_H_
#define ICON_HAL_ICON_STATE_REGISTER_H_

#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/interfaces/icon_state_utils.h"

namespace intrinsic::icon {

// Reserved name of the ICON state interface.
static constexpr char kIconStateInterfaceName[] = "icon_state";

namespace hardware_interface_traits {

// Registers the IconState hardware interface.
// Allows transparently depending on IconState and can be included in multiple
// files.
//
// Usage:
// #include "icon/hal/icon_state_register.h"  // IWYU pragma: keep
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::IconState,
                                 intrinsic_fbs::BuildIconState,
                                 "intrinsic_fbs.IconState")
}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon
#endif  // ICON_HAL_ICON_STATE_REGISTER_H_
