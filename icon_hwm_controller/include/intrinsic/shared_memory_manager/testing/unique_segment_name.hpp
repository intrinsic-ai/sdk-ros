#pragma once


#include <string>

#include "intrinsic/shared_memory_manager/memory_segment.hpp"

namespace intrinsic::hal
{

// Generates a unique hardware module name. This helps avoid memory segment
// naming collisions in tests.
std::string UniqueHardwareModuleName();

// Generates a unique shared memory namespace. This helps avoid memory segment
// naming collisions in tests.
std::string UniqueMemoryNamespace();

}  // namespace intrinsic::hal
