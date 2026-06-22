#include "icon/interprocess/remote_trigger/remote_trigger_server.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <latch>
#include <memory>
#include <string_view>
#include <thread>
#include <utility>

#include "icon/interprocess/remote_trigger/remote_trigger_test_common.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "util/thread/thread.h"

namespace intrinsic::icon {
namespace {

using remote_trigger_test_common::WaitForServer;

constexpr absl::string_view kModuleName = "my_test_hardware_module";

log::Logger::SinkCallback StdErrSink() {
  return [](const log::Logger::LogEntry& entry) {
    std::string severity_name = "";
    switch (entry.severity) {
      case log::Logger::Severity::kDebug:
        severity_name = "DEBUG";
        break;
      case log::Logger::Severity::kInfo:
        severity_name = " INFO";
        break;
      case log::Logger::Severity::kWarning:
        severity_name = " WARN";
        break;
      case log::Logger::Severity::kError:
        severity_name = "ERROR";
        break;
      case log::Logger::Severity::kFatal:
        severity_name = "FATAL";
        break;
      default:
        severity_name = "?????";
        break;
    }
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
    std::cerr << severity_name << "[" << seconds << "] "
              << entry.loc.file_name() << ":" << entry.loc.line() << " | "
              << entry.msg << std::endl;
  };
}

class TestRemoteTriggerWithEmptyCallback : public ::testing::Test {
 public:
  void SetUp() override {
    INTR_ASSERT_OK_AND_ASSIGN(
        shm_manager_, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                  kModuleName, nullptr));

    INTR_ASSERT_OK_AND_ASSIGN(
        auto server,
        RemoteTriggerServer::Create(
            *shm_manager_,
            /*server_memory_name=*/UniqueMemoryNamespace(), nullptr, []() {}));
    server_ = std::make_unique<RemoteTriggerServer>(std::move(server));
  }

  std::unique_ptr<RemoteTriggerServer> server_;
  std::unique_ptr<SharedMemoryManager> shm_manager_;
};

TEST_F(TestRemoteTriggerWithEmptyCallback, ServerStartsCorrectly) {
  // Initial state: server is not started but ready to start.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_TRUE(server_->IsReadyToStart());

  Thread server_thread(&RemoteTriggerServer::Start, server_.get(), nullptr);
  // Thread might take a bit to start, so let's wait until the server is
  // started.
  EXPECT_TRUE(WaitForServer(*server_)) << "failed to start server in time";
  // Server is started and thus not ready to start.
  EXPECT_TRUE(server_->IsStarted());
  EXPECT_FALSE(server_->IsReadyToStart());

  server_->RequestStop();
  // Server is not started but is ready to start again since there is no async
  // thread running.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_TRUE(server_->IsReadyToStart());

  server_thread.join();
  // Joining server_thread has no effect; the server is still not started and
  // ready to start again.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_TRUE(server_->IsReadyToStart());
}

TEST_F(TestRemoteTriggerWithEmptyCallback, ServerStartsCorrectlyInAsyncMode) {
  // Initial state: server is not started but ready to start.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_TRUE(server_->IsReadyToStart());

  INTR_EXPECT_OK(server_->StartAsync(nullptr));
  // Server is started and thus not ready to start.
  EXPECT_TRUE(server_->IsStarted());
  EXPECT_FALSE(server_->IsReadyToStart());

  server_->RequestStop();
  // Server is neither started nor ready to start again since the async thread
  // has not been joined.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_FALSE(server_->IsReadyToStart());

  server_->JoinAsyncThread();
  // Back to initial state: server is not started but ready to start.
  EXPECT_FALSE(server_->IsStarted());
  EXPECT_TRUE(server_->IsReadyToStart());
}

TEST_F(TestRemoteTriggerWithEmptyCallback,
       CannotStartServerAfterStopWithJoinFalse) {
  INTR_EXPECT_OK(server_->StartAsync(nullptr));

  server_->RequestStop();
  EXPECT_FALSE(server_->IsReadyToStart());
  // Fails because we stopped the server, but didn't join the thread.
  EXPECT_EQ(server_->StartAsync(nullptr).code, StatusCode::kFailedPrecondition);
  // False, even though the thread is not joined!
  EXPECT_FALSE(server_->IsStarted());

  // Now we stop the thread
  server_->JoinAsyncThread();
  EXPECT_FALSE(server_->IsStarted());

  // Now we can restart the server
  EXPECT_TRUE(server_->IsReadyToStart());
  INTR_EXPECT_OK(server_->StartAsync(nullptr));
  // ...and stop it again
  server_->RequestStop();
  server_->JoinAsyncThread();
  EXPECT_FALSE(server_->IsStarted());
}

TEST_F(TestRemoteTriggerWithEmptyCallback, MoveRunningServerWillBeStopped) {
  INTR_EXPECT_OK(server_->StartAsync(nullptr));
  EXPECT_TRUE(server_->IsStarted());

  // The move constructor stops the server instance.
  auto moved_server = std::move(*server_);
  EXPECT_FALSE(moved_server.IsStarted());
}

TEST_F(TestRemoteTriggerWithEmptyCallback, CantQueryServerWhenAlreadyStarted) {
  INTR_EXPECT_OK(server_->StartAsync(nullptr));
  EXPECT_TRUE(server_->IsStarted());

  EXPECT_FALSE(server_->Query(nullptr));
}

TEST(RemoteTriggerServerTest, ServerStopsWhenFutexIsClosed) {
  auto logger = log::Logger(log::Logger::Severity::kDebug, StdErrSink());

  std::string shared_memory_namespace = UniqueMemoryNamespace();
  std::string server_memory_name = "my_test_server";

  INTR_ASSERT_OK_AND_ASSIGN(auto shm_manager,
                            SharedMemoryManager::Create(shared_memory_namespace,
                                                        kModuleName, &logger));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto server, RemoteTriggerServer::Create(*shm_manager, server_memory_name,
                                               &logger, []() {}));

  INTR_EXPECT_OK(server.StartAsync(&logger));
  EXPECT_TRUE(server.IsStarted());

  std::string request_memory_name = absl::StrCat(server_memory_name, ".req");
  INTR_ASSERT_OK_AND_ASSIGN(
      auto request_futex, shm_manager->Get<ReadWriteMemorySegment<BinaryFutex>>(
                              request_memory_name, &logger));

  request_futex.GetValue().Close();

  std::latch done(2);
  std::thread join_thread([&]() {
    server.JoinAsyncThread();
    done.count_down();
  });

  done.arrive_and_wait();
  join_thread.join();
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
