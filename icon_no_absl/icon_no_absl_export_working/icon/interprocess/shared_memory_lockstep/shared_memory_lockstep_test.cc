#include "icon/interprocess/shared_memory_lockstep/shared_memory_lockstep.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <latch>
#include <string>
#include <utility>

#include "icon/interprocess/shared_memory_manager/memory_segment.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "icon/utils/time.h"
#include "util/thread/lockstep.h"
#include "util/thread/thread.h"

namespace intrinsic::icon {

static constexpr auto kLockstepTestTimeout = std::chrono::milliseconds(100);

TEST(SharedMemoryLockstepTest, SingleProcess) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  std::string segment_name = UniqueMemoryNamespace();

  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, segment_name,
                                                /*logger=*/nullptr));
  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep_twin,
      GetSharedMemoryLockstep(*shm_manager, segment_name, /*logger=*/nullptr));

  ASSERT_TRUE(lockstep.Connected());
  INTR_ASSERT_OK(lockstep->StartOperationAWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep->EndOperationA());
  INTR_ASSERT_OK(
      lockstep_twin->StartOperationBWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_twin->EndOperationB());
}

TEST(SharedMemoryLockstepTest, LockstepIsNotConnectedWithoutClient) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  std::string segment_name = UniqueMemoryNamespace();

  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, segment_name,
                                                /*logger=*/nullptr));
  // Don't attach a client to the lockstep.
  // ASSERT_OK_AND_ASSIGN(SharedMemoryLockstep lockstep_twin,
  //                      GetSharedMemoryLockstep(segment_name));
  EXPECT_FALSE(lockstep.Connected());
}

TEST(SharedMemoryLockstepTest, SingleProcessMoveWorks) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  std::string segment_name = UniqueMemoryNamespace();
  SharedMemoryLockstep lockstep_moved;
  SharedMemoryLockstep lockstep_twin_moved;
  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, segment_name,
                                                /*logger=*/nullptr));
  INTR_ASSERT_OK_AND_ASSIGN(auto lockstep_twin,
                            GetSharedMemoryLockstep(*shm_manager, segment_name,
                                                    /*logger=*/nullptr));

  INTR_ASSERT_OK(lockstep->StartOperationAWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep->EndOperationA());
  INTR_ASSERT_OK(
      lockstep_twin->StartOperationBWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_twin->EndOperationB());

  lockstep_moved = std::move(lockstep);
  INTR_ASSERT_OK(
      lockstep_moved->StartOperationAWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_moved->EndOperationA());
  INTR_ASSERT_OK(
      lockstep_twin->StartOperationBWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_twin->EndOperationB());

  lockstep_twin_moved = std::move(lockstep_twin);

  INTR_ASSERT_OK(
      lockstep_moved->StartOperationAWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_moved->EndOperationA());
  INTR_ASSERT_OK(
      lockstep_twin_moved->StartOperationBWithTimeout(kLockstepTestTimeout));
  INTR_ASSERT_OK(lockstep_twin_moved->EndOperationB());
}

TEST(SharedMemoryLockstepTest, MultithreadLockstepWorks) {
  std::string lockstep_segment = UniqueMemoryNamespace();
  std::string data_segment = UniqueMemoryNamespace();

  const int kRepetitions = 10000;

  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                                /*logger=*/nullptr));
  INTR_ASSERT_OK(shm_manager->AddSegment<int>(data_segment,
                                              /*must_be_used=*/false, 0));
  std::latch op_a_thread_ready(2);
  intrinsic::Thread op_a_thread(
      [&op_a_thread_ready, &shm_manager, lockstep_segment, data_segment]() {
        INTR_ASSERT_OK_AND_ASSIGN(
            auto lockstep_twin,
            GetSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                    /*logger=*/nullptr));

        INTR_ASSERT_OK_AND_ASSIGN(
            auto data_twin,
            ReadWriteMemorySegment<int>::Get(
                shm_manager->SegmentNameToFileDescriptorMap(), data_segment,
                /*logger=*/nullptr));
        op_a_thread_ready.count_down();
        for (int i = 0; i < kRepetitions; i++) {
          INTR_ASSERT_OK(
              lockstep_twin->StartOperationAWithTimeout(kLockstepTestTimeout));
          data_twin.SetValue(data_twin.GetValue() + 1);  // Increment data.
          // Child process should only see odd values.
          EXPECT_EQ(data_twin.GetValue(), 2 * i + 1);
          INTR_ASSERT_OK(lockstep_twin->EndOperationA());
        }
      });

  op_a_thread_ready.arrive_and_wait();

  INTR_ASSERT_OK_AND_ASSIGN(
      auto data,
      ReadWriteMemorySegment<int>::Get(
          shm_manager->SegmentNameToFileDescriptorMap(), data_segment,
          /*logger=*/nullptr));

  for (int i = 0; i < kRepetitions; i++) {
    // Fail early in case op_a_thread_started is not getting scheduled.
    INTR_ASSERT_OK(lockstep->StartOperationBWithTimeout(kLockstepTestTimeout));
    data.SetValue(data.GetValue() + 1);  // Increment data.
    // Parent process should only see even values.
    EXPECT_EQ(data.GetValue(), 2 * i + 2);
    INTR_ASSERT_OK(lockstep->EndOperationB());
  }
}

TEST(SharedMemoryLockstepTest, MultiprocessCancelWorks) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  std::string lockstep_segment = UniqueMemoryNamespace();
  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                                /*logger=*/nullptr));
  auto pid = fork();
  ASSERT_NE(pid, -1);
  if (pid == 0) {  // Child process:
    INTR_ASSERT_OK_AND_ASSIGN(
        auto lockstep_twin,
        GetSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                /*logger=*/nullptr));
    lockstep_twin->Cancel(/*logger=*/nullptr);  // Cancel outside of operation.
    _exit(EXIT_SUCCESS);
  } else {  // Parent process:
    RealtimeStatus result =
        lockstep->StartOperationBWithTimeout(kLockstepTestTimeout);
    EXPECT_EQ(result.code, StatusCode::kAborted) << ToString(result);
    // Wait for the child process to exit.
    wait(nullptr);
  }
}

TEST(SharedMemoryLockstepTest, MultiprocessCancelDuringOperationWorks) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module_name",
                                  /*logger=*/nullptr));
  std::string lockstep_segment = UniqueMemoryNamespace();
  INTR_ASSERT_OK_AND_ASSIGN(
      auto lockstep, CreateSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                                /*logger=*/nullptr));
  auto pid = fork();
  ASSERT_NE(pid, -1);
  if (pid == 0) {  // Child process:
    INTR_ASSERT_OK_AND_ASSIGN(
        auto lockstep_twin,
        GetSharedMemoryLockstep(*shm_manager, lockstep_segment,
                                /*logger=*/nullptr));
    INTR_ASSERT_OK(
        lockstep_twin->StartOperationAWithTimeout(kLockstepTestTimeout));
    lockstep_twin->Cancel(/*logger=*/nullptr);  // Cancel during operation.
    _exit(EXIT_SUCCESS);
  } else {  // Parent process:
    RealtimeStatus result =
        lockstep->StartOperationBWithTimeout(kLockstepTestTimeout);
    EXPECT_EQ(result.code, StatusCode::kAborted) << ToString(result);
    // Wait for the child process to exit.
    wait(nullptr);
  }
}

}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
