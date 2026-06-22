#ifndef ICON_HAL_INTERFACES_ICON_STATE_UTILS_H_
#define ICON_HAL_INTERFACES_ICON_STATE_UTILS_H_

#include "flatbuffers/detached_buffer.h"
#include "icon/utils/attributes.h"

namespace intrinsic_fbs {

// Initializes `current_cycle` with `std::numeric_limits<uint64_t>::max()`, so
// that the IconState flatbuffer is invalid/inconsistent until it receives its
// first update.
INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildIconState();

}  // namespace intrinsic_fbs
#endif  // ICON_HAL_INTERFACES_ICON_STATE_UTILS_H_
