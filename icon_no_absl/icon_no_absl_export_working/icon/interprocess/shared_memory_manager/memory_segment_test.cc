#include "icon/interprocess/shared_memory_manager/memory_segment.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <utility>

#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/interprocess/shared_memory_manager/segment_header.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "icon/utils/time.h"

namespace intrinsic::icon {
namespace {

using ::testing::AllOf;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::Pointee;

// Feature `using` here to avoid a comma (,) in the ASSERT_OK macro call.
using LargeArrayT = std::array<uint64_t, 1000>;
constexpr absl::string_view kModuleName = "my_test_hardware_module";

struct SharedMemoryTestContext {
  std::unique_ptr<SharedMemoryManager> shm_manager;
  std::string int_id = UniqueMemoryNamespace();
  std::string array_id = UniqueMemoryNamespace();
};

inline tl::expected<SharedMemoryTestContext, Status> SetupSharedManager(
    bool must_be_used = false) {
  SharedMemoryTestContext ctx;
  // Ensures the objects are not used after the move.
  INTR_ASSIGN_OR_RETURN_UNEXPECTED(
      ctx.shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), kModuleName,
                                  /*logger=*/nullptr));

  INTR_RETURN_UNEXPECTED_IF_ERROR(
      ctx.shm_manager->AddSegment<int>(ctx.int_id, must_be_used, 123));

  INTR_RETURN_UNEXPECTED_IF_ERROR(
      ctx.shm_manager->AddSegmentWithDefaultValue<LargeArrayT>(ctx.array_id,
                                                               must_be_used));
  return ctx;
}

TEST(TestMemorySegment, GetChecksSizeForSegmentHeader) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  // Shrinks the segment to an invalid size.
  ASSERT_EQ(ftruncate(ctx.shm_manager->SegmentNameToFileDescriptorMap().at(
                          ctx.int_id),
                      sizeof(SegmentHeader)),
            0);
  auto segment = ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
      ctx.int_id, /*logger=*/nullptr);
  ASSERT_FALSE(segment.has_value());
  EXPECT_EQ(segment.error().code, StatusCode::kInternal)
      << ToString(segment.error());
  EXPECT_THAT(segment.error().message,
              AllOf(HasSubstr(ctx.int_id), HasSubstr("SegmentHeader")))
      << ToString(segment.error());
}

TEST(SharedMemorySegmentFitsLowerSizeBound, FailsWhenSegmentTooSmall) {
  using custom_array = std::array<int32_t, 10>;
  Status s = SharedMemorySegmentFitsLowerSizeBound<custom_array>(
      "segment",
      /*segment_size=*/sizeof(SegmentHeader) + sizeof(custom_array) - 1);
  EXPECT_EQ(s.code, StatusCode::kInternal) << ToString(s);
  EXPECT_THAT(s.message, HasSubstr("size")) << ToString(s);
}

TEST(SharedMemorySegmentFitsLowerSizeBound, CorrectSize) {
  using custom_array = std::array<int32_t, 10>;
  INTR_EXPECT_OK(SharedMemorySegmentFitsLowerSizeBound<custom_array>(
      "segment",
      /*segment_size=*/sizeof(SegmentHeader) + sizeof(custom_array)));
}

TEST(SharedMemorySegmentFitsLowerSizeBound, WorksWhenSegmentTooBig) {
  using custom_array = std::array<int32_t, 10>;
  INTR_EXPECT_OK(SharedMemorySegmentFitsLowerSizeBound<custom_array>(
      "segment",
      /*segment_size=*/sizeof(SegmentHeader) + sizeof(custom_array) + 1));
}

TEST(TestMemorySegment, GetChecksSizeOfRawData) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  // Shrinks the segment to an invalid size, but larger than the size of the
  // SegmentHeader, so that MemorySegment::Get() succeeds, but
  // SharedMemoryManager::Get<T>() fails.
  ASSERT_EQ(ftruncate(ctx.shm_manager->SegmentNameToFileDescriptorMap().at(
                          ctx.int_id),
                      sizeof(SegmentHeader) + 1),
            0);
  auto segment = ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
      ctx.int_id, /*logger=*/nullptr);
  ASSERT_FALSE(segment.has_value());
  EXPECT_EQ(segment.error().code, StatusCode::kInternal)
      << ToString(segment.error());
  EXPECT_THAT(segment.error().message,
              AllOf(HasSubstr(ctx.int_id), HasSubstr("bytes")))
      << ToString(segment.error());
}

TEST(ReadOnlyMemorySegment, DefaultConstructedValueSizeIsZero) {
  ReadOnlyMemorySegment<int> segment;
  EXPECT_THAT(segment.ValueSize(), Eq(0));
}

TEST(ReadWriteMemorySegment, DefaultConstructedValueSizeIsZero) {
  ReadWriteMemorySegment<int> segment;
  EXPECT_THAT(segment.ValueSize(), Eq(0));
}

TEST(TestMemorySegment, ValueSizeWorks) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());
  INTR_ASSERT_OK_AND_ASSIGN(auto segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));

  EXPECT_EQ(segment.ValueSize(), sizeof(int));
}

TEST(TestMemorySegment, InitReadOnly) {
  ASSERT_NE(UniqueHardwareModuleName(), UniqueHardwareModuleName());
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(ro_segment.Name(), Eq(ctx.int_id));
  EXPECT_THAT(ro_segment.GetValue(), Eq(123));
  EXPECT_THAT(ro_segment.Header().ReaderRefCount(), Eq(1));
  EXPECT_THAT(ro_segment.Header().WriterRefCount(), Eq(0));
  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(123)));
}

TEST(TestMemorySegment, DefaultInitReadOnly) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  ReadOnlyMemorySegment<int> ro_segment;
  EXPECT_FALSE(ro_segment.IsValid());
  INTR_ASSERT_OK_AND_ASSIGN(ro_segment,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_TRUE(ro_segment.IsValid());
}

TEST(TestMemorySegment, InitConstReadOnly) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(ro_segment.Name(), Eq(ctx.int_id));
  EXPECT_THAT(ro_segment.GetValue(), Eq(123));
  EXPECT_THAT(ro_segment.Header().ReaderRefCount(), Eq(1));
  EXPECT_THAT(ro_segment.Header().WriterRefCount(), Eq(0));
  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(123)));
}

TEST(TestMemorySegment, InitReadWrite) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());
  INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));

  EXPECT_THAT(rw_segment.Name(), Eq(ctx.int_id));
  EXPECT_THAT(rw_segment.GetValue(), Eq(123));
  EXPECT_THAT(rw_segment.Header().WriterRefCount(), Eq(1));
  EXPECT_THAT(rw_segment.Header().ReaderRefCount(), Eq(0));

  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(123)));

  rw_segment.SetValue(789);
  EXPECT_THAT(rw_segment.GetValue(), Eq(789));
  EXPECT_THAT(int_val, Pointee(Eq(789)));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto rw_segment_large,
      ctx.shm_manager->Get<ReadWriteMemorySegment<LargeArrayT>>(
          ctx.array_id, /*logger=*/nullptr));
  const LargeArrayT* arr_val =
      ctx.shm_manager->GetSegmentValue<LargeArrayT>(ctx.array_id);
  EXPECT_THAT(arr_val, Pointee(Eq(rw_segment_large.GetValue())));
  LargeArrayT& val = rw_segment_large.GetValue();
  val.back() = 1;
  EXPECT_THAT(arr_val->back(), Eq(1));
}

TEST(TestMemorySegment, DefaultInitReadWrite) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  ReadWriteMemorySegment<int> rw_segment;
  EXPECT_FALSE(rw_segment.IsValid());
  INTR_ASSERT_OK_AND_ASSIGN(rw_segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_TRUE(rw_segment.IsValid());
}

TEST(TestMemorySegment, InitConstReadWrite) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(const auto rw_segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(rw_segment.Name(), Eq(ctx.int_id));
  EXPECT_THAT(rw_segment.GetValue(), Eq(123));
  EXPECT_THAT(rw_segment.Header().WriterRefCount(), Eq(1));
  EXPECT_THAT(rw_segment.Header().ReaderRefCount(), Eq(0));

  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(123)));
}

TEST(TestMemorySegment, InitMultipleReadOnly) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment1,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), 1);

  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment2,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(ro_segment2.Header().ReaderRefCount(), Eq(2));
  EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(2));

  {
    // Move assignment
    auto moved_ro_segment2 = std::move(ro_segment2);
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(ro_segment2.HeaderPointer(), IsNull());

    // Reference count is shared between the moved-to segment and
    // ro_segment1, since ro_segment1 points to the same data.
    //
    // The count remains the same because we only moved ro_segment2.
    EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(2));
    EXPECT_THAT(moved_ro_segment2.Header().ReaderRefCount(), Eq(2));

    // Get a new segment that uses the same memory.
    INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment3,
                              ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                  ctx.int_id, /*logger=*/nullptr));
    // Reference count, now shared across *three* segments, increases.
    EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(3));
    EXPECT_THAT(moved_ro_segment2.Header().ReaderRefCount(), Eq(3));
    EXPECT_THAT(ro_segment3.Header().ReaderRefCount(), Eq(3));

    // Move constructor (this is different from move assignment!)
    ReadOnlyMemorySegment<int> moved_ro_segment3{std::move(ro_segment3)};
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(ro_segment3.HeaderPointer(), IsNull());

    // The shared reference count stays the same
    EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(3));
    EXPECT_THAT(moved_ro_segment2.Header().ReaderRefCount(), Eq(3));
    EXPECT_THAT(moved_ro_segment3.Header().ReaderRefCount(), Eq(3));

    // Move one segment into another
    moved_ro_segment2 = std::move(moved_ro_segment3);
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(moved_ro_segment3.HeaderPointer(), IsNull());

    // This should *decrease* the reference count, because we've displaced the
    // old value of moved_ro_segment2.
    EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(2));
    EXPECT_THAT(moved_ro_segment2.Header().ReaderRefCount(), Eq(2));
  }

  // Ref count decrements for all when moved_ro_segment3 goes out of scope.
  EXPECT_THAT(ro_segment1.Header().ReaderRefCount(), Eq(1));
}

TEST(TestMemorySegment, InitMultipleReadWrite) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment1,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(1));

  INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment2,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(rw_segment2.Header().WriterRefCount(), Eq(2));
  EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(2));

  {
    // Move assignment, ref_count remains untouched.
    auto moved_rw_segment2 = std::move(rw_segment2);
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(rw_segment2.HeaderPointer(), IsNull());

    // Reference count is shared between the moved-to segment and
    // rw_segment1, since rw_segment1 points to the same data.
    //
    // The count remains the same because we only moved rw_segment2.
    EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(2));
    EXPECT_THAT(moved_rw_segment2.Header().WriterRefCount(), Eq(2));

    // Get a new segment that uses the same memory.
    INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment3,
                              ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                  ctx.int_id, /*logger=*/nullptr));
    // Reference count, now shared across *three* segments, increases.
    EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(3));
    EXPECT_THAT(moved_rw_segment2.Header().WriterRefCount(), Eq(3));
    EXPECT_THAT(rw_segment3.Header().WriterRefCount(), Eq(3));

    // Move constructor (this is different from move assignment!)
    ReadWriteMemorySegment<int> moved_rw_segment3{std::move(rw_segment3)};
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(rw_segment3.HeaderPointer(), IsNull());

    // The shared reference count stays the same
    EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(3));
    EXPECT_THAT(moved_rw_segment2.Header().WriterRefCount(), Eq(3));
    EXPECT_THAT(moved_rw_segment3.Header().WriterRefCount(), Eq(3));

    // Move one segment into another
    moved_rw_segment2 = std::move(moved_rw_segment3);
    // Moved-from segment has nullptr header now
    // NOLINTNEXTLINE(bugprone-use-after-move)
    EXPECT_THAT(moved_rw_segment3.HeaderPointer(), IsNull());

    // This should *decrease* the reference count, because we've displaced the
    // old value of moved_rw_segment2.
    EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(2));
    EXPECT_THAT(moved_rw_segment2.Header().WriterRefCount(), Eq(2));
  }

  // Ref count decrements for all when moved_rw_segment3 goes out of scope.
  EXPECT_THAT(rw_segment1.Header().WriterRefCount(), Eq(1));
}

TEST(TestMemorySegment, ReadWriteData) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(rw_segment.GetValue(), Eq(123));

  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  EXPECT_THAT(ro_segment.GetValue(), Eq(123));

  rw_segment.SetValue(456);
  EXPECT_THAT(ro_segment.GetValue(), Eq(rw_segment.GetValue()));
  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(456)));
}

TEST(TestMemorySegment, WriteDataInFork) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  auto pid = fork();
  if (pid == -1) {
    FAIL();
  }
  // Child process gets a read-write handle and increments the shared data.
  if (pid == 0) {
    {
      INTR_ASSERT_OK_AND_ASSIGN(
          auto rw_segment, ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                               ctx.int_id, /*logger=*/nullptr));
      EXPECT_THAT(rw_segment.GetValue(), Eq(123));
      rw_segment.SetValue(456);
    }
    _exit(EXIT_SUCCESS);
  } else {
    // Wait for the child process to exit.
    wait(nullptr);
    INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment,
                              ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                  ctx.int_id, /*logger=*/nullptr));
    EXPECT_THAT(ro_segment.GetValue(), Eq(456));
    EXPECT_THAT(ro_segment.Header().ReaderRefCount(), Eq(1));
  }

  const int* int_val = ctx.shm_manager->GetSegmentValue<int>(ctx.int_id);
  EXPECT_THAT(int_val, Pointee(Eq(456)));
}

TEST(TestMemorySegment, UpdatedAt) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx, SetupSharedManager());

  INTR_ASSERT_OK_AND_ASSIGN(auto rw_segment,
                            ctx.shm_manager->Get<ReadWriteMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));
  INTR_ASSERT_OK_AND_ASSIGN(auto ro_segment,
                            ctx.shm_manager->Get<ReadOnlyMemorySegment<int>>(
                                ctx.int_id, /*logger=*/nullptr));

  ASSERT_EQ(ro_segment.Header().LastUpdatedTime(), Time{});
  ASSERT_EQ(ro_segment.Header().NumUpdates(), 0);
  ASSERT_EQ(ro_segment.Header().LastUpdatedCycle(), 0);

  // Mark an update on the rw_segment, and check the read only segment can see
  // it.
  const auto now = Now();
  const uint64_t cycle = 42;
  rw_segment.UpdatedAt(now, cycle);
  EXPECT_EQ(ro_segment.Header().LastUpdatedTime(), now);
  EXPECT_EQ(ro_segment.Header().NumUpdates(), 1);
  EXPECT_EQ(ro_segment.Header().LastUpdatedCycle(), cycle);
}

TEST(TestMemorySegment, IsRequiredIsPopulatedTrue) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx,
                            SetupSharedManager(/**must_be_used=*/true));

  intrinsic_fbs::SegmentInfo segment_info = ctx.shm_manager->GetSegmentInfo();
  // Size of segment_info.names() is not adjusted it stays an array of size 100.
  EXPECT_THAT(segment_info.size(), Eq(2));
  EXPECT_THAT(segment_info.names()->Get(0)->must_be_used(), Eq(true));
  EXPECT_THAT(segment_info.names()->Get(1)->must_be_used(), Eq(true));
}

TEST(TestMemorySegment, IsRequiredIsPopulatedFalse) {
  INTR_ASSERT_OK_AND_ASSIGN(SharedMemoryTestContext ctx,
                            SetupSharedManager(/**must_be_used=*/false));

  intrinsic_fbs::SegmentInfo segment_info = ctx.shm_manager->GetSegmentInfo();
  // Size of segment_info.names() is not adjusted it stays an array of size 100.
  EXPECT_THAT(segment_info.size(), Eq(2));
  EXPECT_THAT(segment_info.names()->Get(0)->must_be_used(), Eq(false));
  EXPECT_THAT(segment_info.names()->Get(1)->must_be_used(), Eq(false));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
