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
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace flowstate_ros_bridge {

Diagnostics::Diagnostics(const std::string& diagnostics_service_address,
                         std::size_t deadline_seconds)
    : diagnostics_service_address_(diagnostics_service_address),
      deadline_seconds_(deadline_seconds) {
  // Do nothing. Connections will happen later.
}

absl::Status Diagnostics::connect() {
  // TODO(user): Implement connection to diagnostics service.
  return absl::OkStatus();
}

absl::StatusOr<std::vector<intrinsic_proto::services::v1::InstanceState>>
Diagnostics::get_diagnostics() {
  // TODO(user): Implement retrieval of diagnostics.
  return absl::UnimplementedError("get_diagnostics not implemented");
}


}  // namespace flowstate_ros_bridge