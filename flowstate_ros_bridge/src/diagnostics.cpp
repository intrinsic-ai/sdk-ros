// Copyright 2025 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flowstate_ros_bridge/diagnostics.hpp"

#include <utility>

#include "absl/log/log.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace flowstate_ros_bridge {

Diagnostics::Diagnostics(const std::string& diagnostics_service_address,
                         std::size_t deadline_seconds)
    : diagnostics_service_address_(diagnostics_service_address),
      deadline_seconds_(deadline_seconds) {
  // Do nothing. Connections will happen later.
}

absl::Status Diagnostics::connect() {
  if (connected_) {
    return absl::OkStatus();
  }

  grpc::ChannelArguments channel_args =
      intrinsic::connect::DefaultGrpcChannelArgs();
  channel_args.SetMaxReceiveMessageSize(-1);     // no limit
  channel_args.SetMaxSendMessageSize(10000000);  // 10 MB

  LOG(INFO) << "Connecting to diagnostics service at "
            << diagnostics_service_address_;
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> diagnostics_channel,
                        intrinsic::connect::CreateClientChannel(
                            diagnostics_service_address_,
                            absl::Now() + absl::Seconds(deadline_seconds_),
                            channel_args));
  INTR_RETURN_IF_ERROR(intrinsic::connect::WaitForChannelConnected(
      diagnostics_service_address_, diagnostics_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  diagnostics_stub_ = SystemServiceState::NewStub(std::move(diagnostics_channel));
  LOG(INFO) << "Successfully connected to diagnostics service!";

  connected_ = true;
  return absl::OkStatus();
}

absl::StatusOr<std::vector<intrinsic_proto::services::v1::InstanceState>>
Diagnostics::get_diagnostics() {
  if (!connected_ || !diagnostics_stub_) {
    return absl::FailedPreconditionError(
        "Not connected to the diagnostics service.");
  }

  auto client_context = std::make_unique<grpc::ClientContext>();
  client_context->set_deadline(std::chrono::system_clock::now() +
                               std::chrono::seconds(deadline_seconds_));

  intrinsic_proto::services::v1::ListInstanceStatesRequest request;
  intrinsic_proto::services::v1::ListInstanceStatesResponse response;

  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(
      diagnostics_stub_->ListInstanceStates(client_context.get(), request,
                                            &response)));

  return std::vector<intrinsic_proto::services::v1::InstanceState>(
      response.states().begin(), response.states().end());
}


}  // namespace flowstate_ros_bridge
