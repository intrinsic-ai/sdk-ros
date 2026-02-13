#include <gtest/gtest.h>
#include <icon_hwm_controller/shared_memory_manager.hpp>
#include <iostream>

namespace intrinsic::hal
{
TEST(SharedMemoryManager, Log) {
  std::cerr << "HELLO" << std::endl;
}


TEST(SharedMemoryManager, Init) {
  auto man = SharedMemoryManager::Create("my_namespace", "my_module");

}

}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
