#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/mock_log_sink.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace intrinsic::log
{

TEST(Logger, CallsSink) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::kDebug, MockSink(entries));
  std::source_location loc = std::source_location::current();
  std::string verb = "walk";
  l.Log(loc, Logger::kWarning, "Eyyy, I'm %sing here!", verb);

  EXPECT_EQ(entries.size(), 1);
  const auto & e = entries.front();
  EXPECT_EQ(e.msg, "Eyyy, I'm walking here!");
  EXPECT_EQ(e.severity, Logger::kWarning);
  EXPECT_EQ(e.loc.file_name(), loc.file_name());
  EXPECT_EQ(e.loc.line(), loc.line());
  EXPECT_EQ(e.loc.column(), loc.column());
  EXPECT_EQ(e.loc.function_name(), loc.function_name());
}

TEST(Logger, DoesNotCallSinkIfLogEntryLevelIsLow) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::kError, MockSink(entries));
  l.Log(std::source_location::current(), Logger::kWarning, "You shouldn't see me");

  EXPECT_EQ(entries.size(), 0);
}

TEST(LogMacro, WorksWithReference) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::kDebug, MockSink(entries));
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(FATAL, l, "Hello!");

  EXPECT_EQ(entries.size(), 5);
  for (const auto & e : entries) {
    EXPECT_EQ(e.msg, "Hello!");
  }
}

TEST(LogMacro, WorksWithPointer) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::kDebug, MockSink(entries));
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, &l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, &l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, &l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, &l, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(FATAL, &l, "Hello!");

  EXPECT_EQ(entries.size(), 5);
  for (const auto & e : entries) {
    EXPECT_EQ(e.msg, "Hello!");
  }
}

TEST(LogMacro, SkipsLoggingWithNullptr) {
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(FATAL, nullptr, "Hello!");
}

}  // namespace intrinsic::log

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
