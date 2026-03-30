#include "intrinsic/shared_memory_manager/remote_trigger_server.hpp"
#include "intrinsic/shared_memory_manager/remote_trigger_constants.hpp"

namespace intrinsic::hal {

tl::expected<RemoteTriggerServer, Status> RemoteTriggerServer::Create(
      intrinsic::hal::SharedMemoryManager& shm_manager,
      std::string_view server_memory_name,
      Callback&& callback) {
  
  std::string request_name = std::string(server_memory_name) + kSemRequestSuffix;
  std::string response_name = std::string(server_memory_name) + kSemResponseSuffix;

  if (auto status = shm_manager.AddSegment<BinaryFutex>(
          request_name,
          /*must_be_used=*/false,
          BinaryFutex());
      !status.ok()) {
    return tl::make_unexpected(status);
  }
  
  if (auto status = shm_manager.AddSegment<BinaryFutex>(
          response_name,
          /*must_be_used=*/false,
          BinaryFutex());
      !status.ok()) {
    return tl::make_unexpected(status);
  }
  
  auto request_futex = shm_manager.Get<ReadOnlyMemorySegment<BinaryFutex>>(request_name);
  if (!request_futex.has_value()) return tl::make_unexpected(request_futex.error());
  
  auto response_futex = shm_manager.Get<ReadWriteMemorySegment<BinaryFutex>>(response_name);
  if (!response_futex.has_value()) return tl::make_unexpected(response_futex.error());
  return RemoteTriggerServer(
    server_memory_name,
      std::move(request_futex.value()),
      std::move(response_futex.value()),
      std::move(callback));
}

RemoteTriggerServer::RemoteTriggerServer(
    std::string_view server_memory_name,
    ReadOnlyMemorySegment<BinaryFutex>&& request_futex,
    ReadWriteMemorySegment<BinaryFutex>&& response_futex,
    Callback&& callback)
    : server_memory_name_(std::string(server_memory_name)),
      callback_(std::forward<Callback>(callback)),
      request_futex_(std::forward<decltype(request_futex)>(request_futex)),
      response_futex_(std::forward<decltype(response_futex)>(response_futex)) {}

RemoteTriggerServer::RemoteTriggerServer(RemoteTriggerServer&& other) noexcept
    : server_memory_name_("") {
  // Make sure that moved server is no longer running.
  other.RequestStop();
  other.JoinAsyncThread();
  server_memory_name_ = std::exchange(other.server_memory_name_, "");
  callback_ = std::exchange(other.callback_, nullptr);
  is_running_.store(false);
  request_futex_ =
      std::exchange(other.request_futex_, ReadOnlyMemorySegment<BinaryFutex>());
  response_futex_ = std::exchange(other.response_futex_,
                                  ReadWriteMemorySegment<BinaryFutex>());
}

RemoteTriggerServer& RemoteTriggerServer::operator=(
    RemoteTriggerServer&& other) noexcept {
  if (this != &other) {
    // Make sure that moved server is no longer running.
    other.RequestStop();
    other.JoinAsyncThread();

    server_memory_name_ = std::exchange(other.server_memory_name_, "");
    callback_ = std::exchange(other.callback_, nullptr);
    is_running_.store(false);
    request_futex_ = std::exchange(other.request_futex_,
                                   ReadOnlyMemorySegment<BinaryFutex>());
    response_futex_ = std::exchange(other.response_futex_,
                                    ReadWriteMemorySegment<BinaryFutex>());
  }

  return *this;
}

RemoteTriggerServer::~RemoteTriggerServer() {
  RequestStop();
  JoinAsyncThread();

  // Close `response_futex_`, but only if it's still there (if we're currently
  // destroying a moved-out-of RemoteTriggerServer, `response_futex_` is
  // nullptr)
  if (auto* response_futex_ptr = reinterpret_cast<BinaryFutex*>(response_futex_.GetRawValue());
      response_futex_ptr != nullptr) {
    response_futex_ptr->Close();
  }
}

void RemoteTriggerServer::Start() {
  // System is already running.
  if (is_running_.load()) {
    return;
  }
  is_running_.store(true);

  Run();
}

Status RemoteTriggerServer::StartAsync(RemoteTriggerServer::Prelude prelude) {
  // Report incomplete shutdown
  if (!is_running_ && async_thread_.joinable()) {
    return Status{
      .code=StatusCode::kFailedPrecondition,
      .message=  "RemoteTriggerServer is not running, but its async thread is active. "
        "Did you call `RequestStop()` and *not* call `JoinAsyncThread()` "
        "afterwards?",
      };
  }
  // System is already running.
  if (bool expected = false;
      !is_running_.compare_exchange_strong(expected, true)) {
    return OkStatus();
  }

  async_thread_ = Thread(&RemoteTriggerServer::Run, this, std::move(prelude));
  if (async_thread_.joinable()) {
    return OkStatus();
  } else {
    RequestStop();
    JoinAsyncThread();
    return Status{
      .code=StatusCode::kInternal,
      .message=  "RemoteTriggerServer failed to start async thread.",
    };
  }
}

bool RemoteTriggerServer::IsStarted() const { return is_running_.load(); }

bool RemoteTriggerServer::IsReadyToStart() const {
  return !IsStarted() && !async_thread_.joinable();
}

void RemoteTriggerServer::RequestStop() { is_running_.store(false); }

void RemoteTriggerServer::JoinAsyncThread() {
  if (async_thread_.joinable()) {
    async_thread_.join();
  }
}

bool RemoteTriggerServer::Query() {
  // The server was started, we don't allow single queries concurrently.
  if (is_running_.load()) {
    return false;
  }

  auto wait_status = request_futex_.GetValue().WaitFor(std::chrono::milliseconds(100));
  // If we woke up because of timeout, don't execute the callback.
  if (wait_status.code == StatusCode::kDeadlineExceeded) {
    return false;
  }
  // Some error occurred, we stop the server.
  if (!wait_status.ok()) {
    // TODO(nilsb): Add realtime logging.
    #if 0
    INTRINSIC_RT_LOG(ERROR)
        << "unable to receive client request: " << wait_status.message();
    #endif
    return false;
  }

  // We woke up because we got a request.
  // If the object was moved or went out of scope, the callback_ might be
  // invalid.
  if (callback_ == nullptr) {
    return false;
  }
  // Call the passed in user callback.
  callback_();

  // Notify the caller.
  auto post_status = response_futex_.GetValue().Post();
  // If we're unable to send the response, that usually indicates a corrupt
  // system state in which the semaphores got destroyed. This further results
  // in a timeout on client side, which has potentially better means to react
  // to wrong behavior.
  if (!post_status.ok()) {
    // TODO(nilsb): Add realtime logging.
    #if 0
    INTRINSIC_RT_LOG(ERROR)
        << "unable to send response to client: " << post_status.message();
    #endif
  }
  return true;
}

void RemoteTriggerServer::Run(RemoteTriggerServer::Prelude prelude) {
  if (prelude) {
    if (auto status = prelude(); !status.ok()) {
      // TODO(nilsb): Proper logging
      std::cerr << "RemoteTriggerServer Prelude failed: " << status << std::endl;
      is_running_.store(false);
      return;
    }
  }
  // Given its asynchronous nature, we have to make sure that the server
  // instance is valid at every point. That is, while the server is running, the
  // object may have been moved and destroyed.
  while (is_running_.load()) {
    auto wait_status =
        request_futex_.GetValue().WaitFor(std::chrono::milliseconds(100));
    // If we woke up because of timeout, don't execute the callback.
    if (wait_status.code == StatusCode::kDeadlineExceeded) {
      continue;
    }
    // Some error occurred, we stop the server.
    if (!wait_status.ok()) {
      // TODO(nilsb): Add realtime logging.
      #if 0
      INTRINSIC_RT_LOG(ERROR)
          << "unable to receive client request: " << wait_status.message();
      #endif
      RequestStop();
      JoinAsyncThread();
      return;
    }

    if (!is_running_.load()) {
      return;
    }

    // We woke up because we got a request.
    // If the object was moved or went out of scope, the callback_ might be
    // invalid.
    if (callback_ == nullptr) {
      RequestStop();
      JoinAsyncThread();
      return;
    }
    // Call the passed in user callback.
    callback_();

    // Notify the caller.
    auto post_status = response_futex_.GetValue().Post();
    // If we're unable to send the response, that usually indicates a corrupt
    // system state in which the semaphores got destroyed. This further results
    // in a timeout on client side, which has potentially better means to react
    // to wrong behavior.
    if (!post_status.ok()) {
      // TODO(nilsb): Add realtime logging.
      #if 0
      INTRINSIC_RT_LOG(ERROR)
          << "unable to send response to client: " << post_status.message();
      #endif
      RequestStop();
      JoinAsyncThread();
      return;
    }
  }
}

}  // namespace intrinsic::hal
