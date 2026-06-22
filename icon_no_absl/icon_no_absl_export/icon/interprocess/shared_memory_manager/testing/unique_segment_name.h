#ifndef ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_TESTING_UNIQUE_SEGMENT_NAME_H_
#define ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_TESTING_UNIQUE_SEGMENT_NAME_H_

#include <string>

#include "icon/interprocess/shared_memory_manager/memory_segment.h"

namespace intrinsic::icon {

// Generates a unique hardware module name. This helps avoid memory segment
// naming collisions in tests.
std::string UniqueHardwareModuleName();

// Generates a unique shared memory namespace. This helps avoid memory segment
// naming collisions in tests.
std::string UniqueMemoryNamespace();

}  // namespace intrinsic::icon
#endif  // ICON_INTERPROCESS_SHARED_MEMORY_MANAGER_TESTING_UNIQUE_SEGMENT_NAME_H_
