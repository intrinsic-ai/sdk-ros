#include "icon/utils/time.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace intrinsic {

TEST(Time, FormatTimeResultIsZeroTerminated) {
  auto t = Now();
  auto formatted = FormatTime(t);
  EXPECT_EQ(formatted[formatted.size() - 1], '\0');
}

TEST(Time, FormatTimeResultHasCorrectLength) {
  auto t = Now();
  auto formatted = FormatTime(t);
  EXPECT_EQ(std::strlen(formatted.data()), std::strlen(kTimeFormat));
}

TEST(Time, FormatTimeIsInRightBallpark) {
  auto wall_now = std::chrono::system_clock::now();
  auto intr_now = Now();
  auto formatted = FormatTime(intr_now);

  std::tm t = {};

  std::istringstream is(formatted.data());
  // `std::chrono::parse()` is not implemented in the compiler version we use,
  // so we need to fall back to `std::get_time()`...
  is >> std::get_time(&t, "%FT%TZ");
  std::chrono::system_clock::time_point parsed =
      std::chrono::system_clock::from_time_t(std::mktime(&t));
  EXPECT_LT(std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(
                         parsed - wall_now)
                         .count()),
            1000)
      << "Formatted time and wall time diverge by more than one second. If you "
         "didn't run this test on a day leading into a leap year, or into/out "
         "of daylight savings time, this is a problem. If you did, please "
         "rerun the test.";
}

TEST(StreamingOperator, PrintsCorrectTime) {
  auto t = Now();
  std::stringstream s;
  s << t;
  std::string seconds_str;
  std::string milliseconds_str;
  std::getline(s, seconds_str, '.');
  std::getline(s, milliseconds_str);
  int seconds = std::stoi(seconds_str);
  int milliseconds = std::stoi(milliseconds_str);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::milliseconds>(
                t.time_since_epoch())
                .count(),
            seconds * 1000 + milliseconds);
}

}  // namespace intrinsic

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
