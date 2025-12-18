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

#ifndef FLOWSTATE_ROS_BRIDGE__DIAGNOSTICS_HPP_
#define FLOWSTATE_ROS_BRIDGE__DIAGNOSTICS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/assets/services/proto/v1/system_service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/system_service_state.pb.h"

namespace flowstate_ros_bridge {

/// @brief A client for the diagnostics service.
class Diagnostics : public std::enable_shared_from_this<Diagnostics> {
  using SystemServiceState =
      ::intrinsic_proto::services::v1::SystemServiceState;

 public:
  /**
   * @brief Construct a new Diagnostics object
   *
   * @param diagnostics_service_address The network address of the diagnostics
   * service.
   * @param deadline_seconds The timeout for gRPC calls.
   */
  Diagnostics(const std::string& diagnostics_service_address,
              std::size_t deadline_seconds = 5);

  /// @brief Establish connection with the diagnostics service.
  absl::Status connect();

  /**
   * @brief Retrieve diagnostics from Flowstate
   *
   * @return absl::StatusOr<std::vector<intrinsic_proto::services::v1::InstanceState>>
   */
  absl::StatusOr<std::vector<intrinsic_proto::services::v1::InstanceState>>
  get_diagnostics();

 private:
  std::string diagnostics_service_address_;
  std::size_t deadline_seconds_;
  bool connected_{false};
  std::unique_ptr<SystemServiceState::Stub> diagnostics_stub_;
};
}  // namespace flowstate_ros_bridge
#endif  // FLOWSTATE_ROS_BRIDGE__DIAGNOSTICS_HPP_
