#include "util/thread/thread.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sched.h>
#include <unistd.h>  // For geteuid()

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tl/expected.hpp>
#include <utility>
#include <vector>

#include "icon/utils/log.h"
#include "icon/utils/realtime_guard.h"
#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/time.hpp"
#include "util/thread/stop_token.h"
#include "util/thread/thread.h"

namespace intrinsic {
namespace {

void Increment(int& x) { x += 1; }

TEST(ThreadTest, DefaultConstructDestruct) {
  Thread thread;
  EXPECT_EQ(thread.joinable(), false);
}

TEST(ThreadTest, MoveCtorDoesNotCancelThread) {
  std::atomic<int> execution_count = 0;
  Thread t1([&execution_count](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ++execution_count;
  });
  Thread t2(std::move(t1));
  // Nothing is running and t1 is not a valid thread of execution anymore.
  EXPECT_EQ(execution_count, 0);
  t2.request_stop();
  t2.join();
  EXPECT_EQ(execution_count, 1);
}

TEST(ThreadTest, MoveAssignmentDoesNotCancelThread) {
  std::atomic<int> execution_count = 0;
  Thread t1([&execution_count](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ++execution_count;
  });
  Thread t2;
  EXPECT_TRUE(t1.joinable());
  // Move assignment should not cancel the thread.
  t2 = std::move(t1);
  // The test below shows that the thread is still running.
  EXPECT_EQ(execution_count, 0);
  // Now we force the thread to stop and to increment the counter.
  t2.request_stop();
  t2.join();
  EXPECT_EQ(execution_count, 1);
}

TEST(ThreadTest, MoveAssignmentFromMovedFromExpectedWorks) {
  std::atomic<int> execution_count = 0;
  tl::expected<Thread, Status> t1([&execution_count](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ++execution_count;
  });
  ASSERT_TRUE(t1.has_value());
  Thread t2;
  EXPECT_TRUE(t1.value().joinable());
  // Move assignment should not cancel the thread.
  t2 = *std::move(t1);
  // The test below shows that the thread is still running.
  EXPECT_EQ(execution_count, 0);
  // Now we force the thread to stop and to increment the counter.
  t2.request_stop();
  t2.join();
  EXPECT_EQ(execution_count, 1);
}

TEST(ThreadTest, MoveAssignmentformLValueRefWorks) {
  std::atomic<int> execution_count = 0;
  tl::expected<Thread, Status> t1([&execution_count](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ++execution_count;
  });
  ASSERT_TRUE(t1.has_value());
  Thread t2;
  EXPECT_TRUE(t1.value().joinable());
  // Move assignment should not cancel the thread.
  // Here, we convert an l-value ref to an r-value ref before moving.
  // tl::expected::operator* returns a mutable reference to the thread.
  t2 = std::move(*t1);
  EXPECT_FALSE(t1.value().joinable());
  // The test below shows that the thread is still running.
  EXPECT_EQ(execution_count, 0);
  // Now we force the thread to stop and to increment the counter.
  t2.request_stop();
  t2.join();
  EXPECT_EQ(execution_count, 1);
}

TEST(ThreadTest, MoveConstruct) {
  Thread thread_a;
  Thread thread_b(std::move(thread_a));
  EXPECT_EQ(thread_b.joinable(), false);
}

TEST(ThreadTest, MoveAssign) {
  Thread thread_a;
  Thread thread_b;
  thread_b = std::move(thread_a);
  EXPECT_EQ(thread_b.joinable(), false);
}

TEST(ThreadTest, ConstructDestruct) {
  int count = 0;
  Thread thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 1);
}

TEST(ThreadTest, ConstructMoveDeleteOldJoin) {
  int count = 0;
  Thread thread_b;
  {
    Thread thread_a(Increment, std::ref(count));
    thread_b = std::move(thread_a);
  }
  ASSERT_TRUE(thread_b.joinable());
  thread_b.join();
  EXPECT_EQ(count, 1);
}

TEST(ThreadTest, ConstructMoveJoin) {
  int count = 0;
  Thread thread_a(Increment, std::ref(count));
  Thread thread_b = std::move(thread_a);
  ASSERT_TRUE(thread_b.joinable());
  thread_b.join();
  EXPECT_EQ(count, 1);
}

TEST(ThreadTest, MoveAfterJoin) {
  Thread thread([]() {});
  thread.join();
  thread = Thread([]() {});
  thread.join();
}

TEST(ThreadTest, MoveAssignWithoutJoin) {
  Thread thread([]() {});
  bool thread_executed = false;
  thread = Thread([&]() { thread_executed = true; });
  thread.join();
  EXPECT_TRUE(thread_executed);
}

TEST(ThreadTest, ThreadIdIsDefaultAfterJoin) {
  Thread t1([](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  EXPECT_NE(t1.get_id(), Thread::id());
  t1.request_stop();
  // Wait for the thread to stop w/o calling join()!
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // The thread body has finished here but we did not yet call join.
  EXPECT_NE(t1.get_id(), Thread::id());
  t1.join();
  EXPECT_EQ(t1.get_id(), Thread::id());
}

TEST(ThreadTest, ThreadIdIsDefaultForEmptyThread) {
  Thread t1;
  EXPECT_EQ(t1.get_id(), Thread::id());
}

TEST(ThreadTest, JoinableAfterStop) {
  Thread t1([](StopToken st) {
    while (!st.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  ASSERT_TRUE(t1.joinable());
  t1.request_stop();
  // Wait for the thread to stop w/o calling join()!
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(t1.joinable());
  // Only after an explicit join() call the thread becomes non-joinable.
  // I.e. joinable() != IsRunning()
  t1.join();
  EXPECT_FALSE(t1.joinable());
}

TEST(ThreadTest, FuncWithStopToken) {
  bool stopped = false;
  Thread thread([&](StopToken st) {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (st.stop_requested()) {
        stopped = true;
        return;
      }
    }
  });
  ASSERT_TRUE(thread.request_stop());
  thread.join();
  EXPECT_TRUE(stopped);
}

TEST(ThreadTest, StopViaStopSource) {
  bool stopped = false;
  Thread thread([&](StopToken st) {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (st.stop_requested()) {
        stopped = true;
        return;
      }
    }
  });
  StopSource stop_source = thread.get_stop_source();
  stop_source.request_stop();
  thread.join();
  EXPECT_TRUE(stopped);
}

TEST(ThreadTest, ConstructDestructWithLambda) {
  int num_calls = 0;
  Thread thread([&num_calls]() { num_calls++; });
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(num_calls, 1);
}

TEST(ThreadTest, ConstructDestructWithBoundFunction) {
  int count = 0;
  Thread thread(std::bind_front(Increment, std::ref(count)));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 1);
}

TEST(ThreadTest, StartJoinStartDefaultConstructor) {
  int count = 0;
  Thread thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 1);

  thread = Thread(Increment, std::ref(count));
  EXPECT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 2);
}

TEST(ThreadTest, StartStartJoinDefaultConstructor) {
  int count = 0;
  Thread thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());

  thread = Thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 2);
}

TEST(ThreadTest, StartStartJoinNonDefaultConstructor) {
  int count = 0;
  Thread thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());

  thread = Thread(Increment, std::ref(count));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 2);
}

// TODO(nilsb): reactivate if/when we export CreateRealtimeCapableThread()
#if 0

// TODO(b/422931054): Ensure
// `ThreadTest.ConstructDestructStartSchedulePermissionDenied`
// Deterministically Tests Real-Time Thread Creation Failure Due to
TEST(ThreadTest, ConstructDestructStartSchedulePermissionDenied) {
  int count = 0;
  Thread thread;
  constexpr int kPriority = 10;
  // Permissions We are not expecting a specific outcome for the realtime thread
  // creation since it is depended on the environment in which tests are
  // executed.
  if (auto status_or_thread = CreateRealtimeCapableThread(
          ThreadOptions().SetPriority(kPriority).SetSchedulePolicy(SCHED_FIFO),
          Increment, std::ref(count));
    status_or_thread.ok())
  {
    thread = std::move(*status_or_thread);
    ASSERT_TRUE(thread.joinable());
    thread.join();
    EXPECT_EQ(count, 1);
  } else {
    // We failed to create a realtime capable thread, so we don't expect the
    // thread checker to have been called.
    ASSERT_EQ(thread.joinable(), false);
    EXPECT_EQ(count, 0);
  }
}

TEST(ThreadTest, ThreadStartRetrySucceedsAfterError) {
  int count = 0;
  Thread thread;
  constexpr int kPriority = 10;
  // We are not expecting a specific outcome for the realtime thread creation
  // since it is depended on the environment in which tests are executed.
  if (auto status_or_thread = CreateRealtimeCapableThread(
          ThreadOptions().SetPriority(kPriority).SetSchedulePolicy(SCHED_FIFO),
          Increment, std::ref(count));
    status_or_thread.ok())
  {
    thread = std::move(*status_or_thread);
  } else {
    // We failed to create a realtime capable thread, so we don't expect the
    // thread checker to have been called.
    ASSERT_EQ(thread.joinable(), false);
    EXPECT_EQ(count, 0);
    thread =
      Thread(Increment, std::ref(count));
  }
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(count, 1);
}

TEST(ThreadTest, ConstructDestructStartPermittedSchedule) {
  int count = 0;
  Thread thread;
  // The schedule macro is defined by linux.
  constexpr int kPriority = 0;
  // Setting SCHED_OTHER as the policy is always permitted under linux:
  // https://linux.die.net/man/3/pthread_setschedparam
  constexpr int kPolicy = SCHED_OTHER;
  ASSERT_OK_AND_ASSIGN(
      thread,
      CreateRealtimeCapableThread(
          ThreadOptions().SetPriority(kPriority).SetSchedulePolicy(kPolicy),
          Increment, std::ref(count)));
  ASSERT_EQ(thread.joinable(), true);
  thread.join();
  EXPECT_EQ(count, 1);
  EXPECT_EQ(thread_checker.Options().GetPriority(), kPriority);
  EXPECT_EQ(thread_checker.Options().GetSchedulePolicy(), kPolicy);
}

TEST(ThreadTest, PermittedScheduleSameAfterMove) {
  int count = 0;
  Thread thread;
  // The schedule macro is defined by linux.
  constexpr int kPriority = 0;
  // Setting SCHED_OTHER as the policy is always permitted under linux:
  // https://linux.die.net/man/3/pthread_setschedparam
  constexpr int kPolicy = SCHED_OTHER;

  absl::Notification moved_notification;
  absl::Notification started_notification;

  ASSERT_OK_AND_ASSIGN(
      thread,
      CreateRealtimeCapableThread(
          ThreadOptions().SetPriority(kPriority).SetSchedulePolicy(kPolicy),
      [&moved_notification, &started_notification, &thread_checker]() {
        started_notification.Notify();      // set the options before the move.
        moved_notification.WaitForNotification();      // read after move.
        thread_checker.ReadThreadOptions();
          }));

  started_notification.WaitForNotification();
  Thread threadb(std::move(thread));
  moved_notification.Notify();
  threadb.join();
  EXPECT_EQ(count, 1);
  EXPECT_EQ(thread_checker.Options().GetPriority(), kPriority);
  EXPECT_EQ(thread_checker.Options().GetSchedulePolicy(), kPolicy);
}

TEST(ThreadTest, ConstructDestructStartInvalidScheduleParameters) {
  int count = 0;
  Thread thread;
  // The schedule macro is defined by linux.
  constexpr int kPriority = -10;  // SCHED_OTHER only allows 0 priority
  // Setting SCHED_OTHER as the policy is always permitted under linux:
  // https://linux.die.net/man/3/pthread_setschedparam
  constexpr int kPolicy = SCHED_OTHER;
  EXPECT_THAT(
      CreateRealtimeCapableThread(
          ThreadOptions().SetPriority(kPriority).SetSchedulePolicy(kPolicy),
          Increment, std::ref(count)),
      StatusIs(absl::StatusCode::kInvalidArgument));
  ASSERT_EQ(thread.joinable(), false);
  EXPECT_EQ(count, 0);
}

#endif

TEST(ThreadTest, DefaultConstructDestructThread) {
  int count = 0;
  Thread thread(Increment, std::ref(count));
  ASSERT_EQ(thread.joinable(), true);
  thread.join();
  EXPECT_EQ(count, 1);
}

// TODO(nilsb): reactive if/when we export ThreadOptions
#if 0

TEST(ThreadTest, SetsThreadName) {
  testing::ThreadChecker run_checker;
  Thread thread;
  constexpr char kName[] = "foo";
  ASSERT_OK_AND_ASSIGN(
      thread, CreateRealtimeCapableThread(
                  ThreadOptions().SetName(kName),
                  &testing::ThreadChecker::ReadThreadOptions, &run_checker));
  ASSERT_EQ(thread.joinable(), true);
  thread.join();
  EXPECT_EQ(*run_checker.Options().GetName(), kName);
}

TEST(ThreadTest, SetsTruncatedThreadName) {
  testing::ThreadChecker run_checker;
  Thread thread;
  const std::string name = "The_quick_brown_fox_jumps_over_the_lazy_dog";
  const std::string name_truncated(
    name, name.size() - GetMaxPosixThreadNameLength() + 1,
    GetMaxPosixThreadNameLength());
  ASSERT_OK_AND_ASSIGN(
      thread, CreateRealtimeCapableThread(
                  ThreadOptions().SetName(name),
                  &testing::ThreadChecker::ReadThreadOptions, &run_checker));
  ASSERT_EQ(thread.joinable(), true);
  thread.join();
  EXPECT_EQ(*run_checker.Options().GetName(), name_truncated);
}

TEST(ThreadTest, StartDefaultOptionsConstructorStartEquivalence) {
  testing::ThreadChecker run_checker1;
  Thread thread1;
  ASSERT_OK_AND_ASSIGN(
      thread1, CreateRealtimeCapableThread(
                   ThreadOptions(), &testing::ThreadChecker::ReadThreadOptions,
                   &run_checker1));
  testing::ThreadChecker run_checker2;
  Thread thread2(&testing::ThreadChecker::ReadThreadOptions, &run_checker2);

  ASSERT_EQ(thread1.joinable(), true);
  ASSERT_EQ(thread2.joinable(), true);

  thread1.join();
  thread2.join();
  EXPECT_EQ(run_checker1.NumCalls(), 1);
  EXPECT_EQ(run_checker2.NumCalls(), 1);
  EXPECT_EQ(run_checker1.Options().GetCpuSet(),
            run_checker2.Options().GetCpuSet());
  EXPECT_EQ(run_checker1.Options().GetPriority(),
            run_checker2.Options().GetPriority());
  EXPECT_EQ(run_checker1.Options().GetSchedulePolicy(),
            run_checker2.Options().GetSchedulePolicy());
}

TEST(ThreadTest, DefaultOptionsAreEqual) {
  ThreadOptions op1;
  ThreadOptions op2;
  EXPECT_EQ(op1, op2);
}

TEST(ThreadTest, NonDefaultOptionsAreEqual) {
  constexpr int kPrio = 1;
  constexpr int kPolicy = 2;
  const std::vector<int> affinity = {0};
  EXPECT_EQ(ThreadOptions()
    .SetAffinity(affinity)
    .SetPriority(kPrio)
    .SetSchedulePolicy(kPolicy),
            ThreadOptions()
    .SetAffinity(affinity)
    .SetPriority(kPrio)
    .SetSchedulePolicy(kPolicy));
}

TEST(ThreadTest, OptionsUnequalWhenPrioDifferent) {
  EXPECT_NE(ThreadOptions().SetPriority(1), ThreadOptions().SetPriority(2));
}

TEST(ThreadTest, OptionsUnequalWhenPolicyDifferent) {
  EXPECT_NE(ThreadOptions().SetSchedulePolicy(1),
            ThreadOptions().SetSchedulePolicy(2));
}

TEST(ThreadTest, OptionsUnequalWhenAffinityDifferent) {
  EXPECT_NE(ThreadOptions().SetAffinity({0}), ThreadOptions().SetAffinity({1}));
}

TEST(ThreadTest, OptionsUnequalWhenPolicyNotSetInOne) {
  EXPECT_NE(ThreadOptions().SetSchedulePolicy(1), ThreadOptions());
}

TEST(ThreadTest, OptionsUnequalWhenPrioNotSetInOne) {
  EXPECT_NE(ThreadOptions().SetPriority(1), ThreadOptions());
}

#endif

TEST(ThreadTest, AssertsNonRealtime) {
  // When there is no realtime guard enabled (per the default options), we
  // expect that a non-realtime call in the thread to succeed.
  Thread thread([]() { INTRINSIC_ASSERT_NON_REALTIME(); });
  thread.join();
}

// TODO(nilsb): reactivate when we have non-dummy realtime checking/malloc count
// in open-source
#if 0
TEST(ThreadTest, AssertsRealtime) {
  // We expect that enabling the realtime guard will cause a non-realtime call
  // in the thread to fail.
  Thread thread;
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        ASSERT_OK_AND_ASSIGN(thread,
                             CreateRealtimeCapableThread(
                                 ThreadOptions().SetRealtimeGuarded(),
          []() {INTRINSIC_ASSERT_NON_REALTIME();}));
        thread.join();
      },
      ::testing::HasSubstr("Unsafe code executed from realtime thread"));
}

TEST(ThreadTest, AssertsMallocGuard) {
  Thread thread;
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        ASSERT_OK_AND_ASSIGN(thread,
                             CreateRealtimeCapableThread(
                                 ThreadOptions().SetMallocGuarded(), []() {
            SetThreadLocalMallocGuardReaction(
                                       icon::MallocGuardReaction::kAbort);
            LOG(INFO) << "some malloc";
                                 }));
        thread.join();
      },
      ::testing::HasSubstr("malloc"));
}
#endif

TEST(ThreadTest, ConstructDestructWithMoveOnlyLambda) {
  int num_calls = 0;
  Thread thread(
      [&num_calls, p = std::make_unique<int>(1)]() { num_calls += *p; });
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(num_calls, 1);
}

#if 0
TEST(ThreadTest, StartWithMoveOnlyLambda) {
  int num_calls = 0;
  Thread thread;
  ASSERT_OK_AND_ASSIGN(
      thread,
      CreateRealtimeCapableThread(
          ThreadOptions(),
      [&num_calls, p = std::make_unique<int>(1)]() {num_calls += *p;}));
  ASSERT_TRUE(thread.joinable());
  thread.join();
  EXPECT_EQ(num_calls, 1);
}
#endif

// TODO(nilsb): reactivate if/when we export tracing
#if 0
TEST(ThreadTest, InitializesTracer) {
  // Isolate test to ClosureThread
  Thread test_thread([]() {
      tracing::Tracer::EnableTracing();
      Thread thread;
      ASSERT_OK_AND_ASSIGN(thread,
                         CreateRealtimeCapableThread(ThreadOptions(), []() {
        IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
                           // If a thread local tracer was initialized during
                           // thread creation, GetThreadLocalTracerOrNullptr
                           // will not malloc.
        tracing::TracerInterface * tracer =
        tracing::Tracer::GetThreadLocalTracerOrNullptr();
        IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
        ASSERT_NE(tracer, nullptr);
                         }));
      thread.join();
    });
}

TEST(ThreadTest, SkipsInitializingTracer) {
  // Isolate test to ClosureThread
  Thread test_thread([]() {
      tracing::Tracer::EnableTracing();
      Thread thread;
      ASSERT_OK_AND_ASSIGN(
        thread,
        CreateRealtimeCapableThread(
            ThreadOptions().SetSkipInitializingThreadLocalTracer(), []() {
            IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
              // If no thread local tracer was initialized during thread
              // creation, GetThreadLocalTracerOrNullptr will create one and
              // malloc.
              //
              // N.B. The malloc count of GetThreadLocalTracerOrNullptr depends
              // on Thread because it uses Thread in its implementation. If you
              // see this count change due to a seemingly unrelated change to
              // Thread, that is likely why.
            tracing::TracerInterface * tracer =
            tracing::Tracer::GetThreadLocalTracerOrNullptr();
            IF_INTRINSIC_MALLOC_TEST_EXPECT_ALLOCATIONS_EQ(2);
            ASSERT_NE(tracer, nullptr);
            }));
      thread.join();
    });
}

#endif

}  // namespace
}  // namespace intrinsic

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
