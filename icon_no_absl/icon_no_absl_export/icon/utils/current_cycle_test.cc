#include "icon/utils/current_cycle.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace intrinsic::icon {

class CurrentCycleTest : public ::testing::Test {
 public:
  CurrentCycleTest() {
    // Initialize the current cycle to zero before every test, since the cycle
    // count uses a static value that would otherwise persist across tests.
    Cycle::SetCurrentCycle(0);
  }
};

TEST_F(CurrentCycleTest, StartsAtZero) {
  EXPECT_EQ(Cycle::GetCurrentCycle(), 0);
}

TEST_F(CurrentCycleTest, CanIncrement) {
  while (Cycle::GetCurrentCycle() < 2000) {
    Cycle::IncrementCurrentCycle();
  }
  EXPECT_EQ(Cycle::GetCurrentCycle(), 2000);
}

TEST_F(CurrentCycleTest, RollsOver) {
  Cycle::SetCurrentCycle(std::numeric_limits<uint64_t>::max());
  Cycle::IncrementCurrentCycle();
  EXPECT_EQ(Cycle::GetCurrentCycle(), 0);
}

}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
