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

#ifndef FLOWSTATE_ROS_BRIDGE__WORLD_HPP_
#define FLOWSTATE_ROS_BRIDGE__WORLD_HPP_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/proto/geometry_service.pb.h"
#include "intrinsic/math/proto/tf_message.pb.h"
// Placeholder for new proto messages
// #include "intrinsic/robot/proto/joint_state_message.pb.h"
// #include "intrinsic/robot/proto/gripper_state_message.pb.h"
// #include "intrinsic/sensor/proto/force_torque_message.pb.h"
// #include "intrinsic/sensor/proto/camera_message.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/zenoh_publisher_data.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace flowstate_ros_bridge {

///=============================================================================
/// A class that establishes bi-directional communication with a World service
class World : public std::enable_shared_from_this<World> {
  using ObjectWorldService = ::intrinsic_proto::world::ObjectWorldService;
  using GeometryService = ::intrinsic_proto::geometry::GeometryService;

 public:
  World(std::shared_ptr<intrinsic::PubSub> pubsub,
        const std::string& world_service_address,
        const std::string& geometry_service_address,
        std::size_t deadline_seconds = 10);

  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>> CreateTfSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::TFMessage> callback);

  // Skeleton for Robot Joint States subscription
  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
  CreateJointStateSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::JointStateMessage> callback);

  // Skeleton for Gripper States subscription
  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
  CreateGripperStateSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::GripperStateMessage> callback);

  // Skeleton for Force Torque Sensor Values subscription
  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>> CreateForceTorqueSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::ForceTorqueMessage> callback);

  // Skeleton for Camera Stream subscription
  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>> CreateCameraSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::CameraMessage> callback);

  // Establish connections with various services.
  absl::Status connect();

  /**
   * @brief Retrieve sceneObjects from Flowstate belief world
   *
   * @param object_names An optional vector of the names of SceneObjects to be
   * retrieved from the World service. If unspecified, all
   * SceneObjects in the Belief World will be retrieved.
   * @return absl::StatusOr<std::vector<intrinsic::world::WorldObject>>
   */
  absl::StatusOr<std::vector<intrinsic::world::WorldObject>> GetObjects(
      std::optional<std::vector<std::string>> object_names = std::nullopt);

  absl::StatusOr<std::string> GetGltf(const std::string& geometry_ref,
                                      const std::string& renderable_ref);

 private:
  std::shared_ptr<intrinsic::PubSub> pubsub_;
  std::string world_service_address_;
  std::string geometry_service_address_;
  size_t deadline_seconds_ = 10;
  bool connected_ = false;
  std::shared_ptr<intrinsic::world::ObjectWorldClient> object_world_client_;
  std::shared_ptr<GeometryService::Stub> geometry_stub_;
};

}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE__WORLD_HPP_
