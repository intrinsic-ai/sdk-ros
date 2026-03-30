#pragma once

#include "intrinsic/hal/hardware_interface_traits.hpp"
#include "hwm_fbs/icon_state.fbs.h"
#include "intrinsic/hal/icon_state_utils.hpp"

namespace intrinsic::hal
{

// Reserved name of the ICON state interface.
static constexpr char kIconStateInterfaceName[] = "icon_state";

namespace hardware_interface_traits
{

// Registers the IconState hardware interface.
// Allows transparently depending on IconState and can be included in multiple
// files.
//
// Usage:
// #include "intrinsic/hal/icon_state_register.hpp"  // IWYU pragma: keep
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::IconState,
                                 intrinsic_fbs::BuildIconState,
                                 "intrinsic_fbs.IconState")
}  // namespace hardware_interface_traits
}  // namespace intrinsic::hal
