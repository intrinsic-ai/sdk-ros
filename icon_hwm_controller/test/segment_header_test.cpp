#include "intrinsic/shared_memory_manager/segment_header.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "intrinsic/utils/time.hpp"

namespace intrinsic::hal
{

using ::testing::Eq;
using ::testing::Ne;
using ::testing::SizeIs;
using ::testing::StrEq;

// Friend class to SegmentHeader, for testing purposes only. Refer to
// go/totw/135 for more info on the "test peer" design pattern.
class SegmentHeaderTestPeer {
public:
  static void SetUpdateCounter(int64_t value, SegmentHeader & header)
  {
    header.update_counter_ = value;
  }
  static void SetVersion(size_t version, SegmentHeader & header)
  {
    size_t * version_ptr = const_cast<size_t *>(&header.kVersion);
    *version_ptr = version;
  }

  static void SetUpdatedAtCycle(uint64_t value, SegmentHeader & header)
  {
    header.updated_at_cycle_ = value;
  }
};

namespace
{

TEST(SegmentHeaderTest, ModifyReferenceCounters) {
  SegmentHeader header;
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));

  header.IncrementReaderRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(1));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));
  header.IncrementWriterRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(1));
  EXPECT_THAT(header.WriterRefCount(), Eq(1));

  header.DecrementReaderRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(1));
  header.DecrementWriterRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));
}

TEST(SegmentHeaderTest, ReferenceCounterCantBeNegative) {
  SegmentHeader header;
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));

  header.DecrementReaderRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));
  header.DecrementWriterRefCount();
  EXPECT_THAT(header.ReaderRefCount(), Eq(0));
  EXPECT_THAT(header.WriterRefCount(), Eq(0));
}

TEST(SegmentHeaderTest, TypeInfoReturnsCorrectType) {
  SegmentHeader header("my_type");
  EXPECT_THAT(header.Type().TypeID(), StrEq("my_type"));
}

TEST(SegmentHeaderTest, TypeInfoTruncatesTypeId) {
  const std::string kTooLongTypeId(SegmentHeader::TypeInfo::kMaxSize + 2, 'a');
  SegmentHeader header(kTooLongTypeId);
  EXPECT_THAT(header.Type().TypeID(),
              SizeIs(SegmentHeader::TypeInfo::kMaxSize));
}

TEST(SegmentHeaderTest, TypeInfoComparesCorrectly) {
  SegmentHeader header1("my_type");
  SegmentHeader header2("my_type");

  EXPECT_THAT(header1.Type(), Eq(header2.Type()));

  SegmentHeader header3("my_other_type");
  EXPECT_THAT(header1.Type(), Ne(header3.Type()));
}

TEST(SegmentHeaderTest, TruncatedTypeInfoComparesCorrectly) {
  const std::string kMaxSizeTypeId(SegmentHeader::TypeInfo::kMaxSize, 'a');
  SegmentHeader header1(kMaxSizeTypeId + "_first");
  SegmentHeader header2(kMaxSizeTypeId + "_second");

  EXPECT_THAT(header1.Type(), Eq(header2.Type()));

  // Same length as the other two, different content
  const std::string kOtherMaxSizeTypeId(SegmentHeader::TypeInfo::kMaxSize, 'b');
  SegmentHeader header3(kOtherMaxSizeTypeId);
  EXPECT_THAT(header1.Type(), Ne(header3.Type()));
}

TEST(SegmentHeaderTest, QueryReturnsCorrectlySetFlags) {
  SegmentHeader no_flags("my_type1");
  EXPECT_FALSE(no_flags.FlagIsSet(SegmentHeader::Flags::kExclusiveOwnership));

  SegmentHeader no_flags2("my_type1", {});
  EXPECT_FALSE(no_flags2.FlagIsSet(SegmentHeader::Flags::kExclusiveOwnership));

  SegmentHeader exclusive_flag(
    "my_type",
    {SegmentHeader::Flags::kExclusiveOwnership});
  EXPECT_TRUE(
      exclusive_flag.FlagIsSet(SegmentHeader::Flags::kExclusiveOwnership));
}

TEST(SegmentHeaderTest, UpdatedAt) {
  SegmentHeader my_header("my_type1");
  ASSERT_EQ(my_header.LastUpdatedTime(), ::intrinsic::Time());
  ASSERT_EQ(my_header.NumUpdates(), 0);
  ASSERT_EQ(my_header.LastUpdatedCycle(), 0);

  auto now = ::intrinsic::Now();
  my_header.UpdatedAt(now, 42);
  EXPECT_EQ(my_header.LastUpdatedTime(), now);
  EXPECT_EQ(my_header.NumUpdates(), 1);
  EXPECT_EQ(my_header.LastUpdatedCycle(), 42);
}

TEST(SegmentHeaderTest, UpdatedAtOverrunWorks) {
  SegmentHeader my_header("my_type");

  int64_t initial_counter = std::numeric_limits<int64_t>::max();
  SegmentHeaderTestPeer::SetUpdateCounter(initial_counter, my_header);
  EXPECT_EQ(my_header.NumUpdates(), initial_counter);
  // Overruns the counter without issues.
  // Validated using `-c dbg --config=ubsan`.
  my_header.UpdatedAt(::intrinsic::Now(), 0);

  EXPECT_EQ(my_header.NumUpdates(), std::numeric_limits<int64_t>::min());
}

TEST(SegmentHeaderTest, VersionWorks) {
  SegmentHeader my_header("my_type");

  EXPECT_EQ(my_header.Version(), SegmentHeader::ExpectedVersion());

  // Overrides the non static version.
  SegmentHeaderTestPeer::SetVersion(SegmentHeader::ExpectedVersion() + 1,
                                    my_header);
  EXPECT_NE(my_header.Version(), SegmentHeader::ExpectedVersion());
}

}   // namespace
}  // namespace intrinsic::hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
