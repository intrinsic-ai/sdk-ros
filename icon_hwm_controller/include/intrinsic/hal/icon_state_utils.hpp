#pragma once

#include "flatbuffers/detached_buffer.h"

namespace intrinsic_fbs
{

// Initializes `current_cycle` with `std::numeric_limits<uint64_t>::max()`, so
// that the IconState flatbuffer is invalid/inconsistent until it receives its
// first update.
flatbuffers::DetachedBuffer BuildIconState();

}  // namespace intrinsic_fbs
