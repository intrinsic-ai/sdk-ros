#include "intrinsic/shared_memory_manager/testing/unique_segment_name.hpp"

#include <cstdint>
#include <string>
#include <random>
#include <sstream>

namespace intrinsic::hal
{

std::string UniqueHardwareModuleName()
{
  std::random_device rd;
  std::default_random_engine engine(rd());
  std::uniform_int_distribution<uint64_t> distrib;
  return (std::stringstream() << std::hex << distrib(engine)).str();
}

std::string UniqueMemoryNamespace()
{
  std::random_device rd;
  std::default_random_engine engine(rd());
  std::uniform_int_distribution<uint64_t> distrib;
  return (std::stringstream() << std::hex << distrib(engine)).str();
}

}  // namespace intrinsic::hal
