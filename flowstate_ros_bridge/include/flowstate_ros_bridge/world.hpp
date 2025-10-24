// Copyright 2025 Intrinsic Innovation LLC
//
// You are hereby granted a non-exclusive, worldwide, royalty-free license to
// use, copy, modify, and distribute this Intrinsic SDK in source code or binary
// form for use in connection with the services and APIs provided by Intrinsic
// Innovation LLC (“Intrinsic”).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// If you use this Intrinsic SDK with any Intrinsic services, your use is
// subject to the Intrinsi Platform Terms of Service
// [https://intrinsic.ai/platform-terms].  If you create works that call
// Intrinsic APIs, you must agree to the terms of service for those APIs
// separately. This license does not grant you any special rights to use the
// services.
//
// This copyright notice shall be included in all copies or substantial portions
// of the software.

#ifndef FLOWSTATE_ROS_BRIDGE__WORLD_HPP_
#define FLOWSTATE_ROS_BRIDGE__WORLD_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/math/proto/tf_message.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
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
        std::shared_ptr<grpc::Channel> world_channel,
        std::shared_ptr<grpc::Channel> geometry_channel,
        std::size_t deadline_seconds = 10);

  absl::StatusOr<std::shared_ptr<intrinsic::Subscription>> CreateTfSubscription(
      intrinsic::SubscriptionOkCallback<intrinsic_proto::TFMessage> callback);

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
  std::shared_ptr<grpc::Channel> world_channel_;
  std::shared_ptr<grpc::Channel> geometry_channel_;
  size_t deadline_seconds_ = 10;
  bool connected_ = false;
  std::shared_ptr<intrinsic::world::ObjectWorldClient> object_world_client_;
  std::shared_ptr<GeometryService::Stub> geometry_stub_;
};

}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE__WORLD_HPP_
