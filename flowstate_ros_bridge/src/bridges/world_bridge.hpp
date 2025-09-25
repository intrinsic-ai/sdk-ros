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

#ifndef BRIDGES__WORLD_BRIDGE_HPP_
#define BRIDGES__WORLD_BRIDGE_HPP_

#include <memory>
#include <set>
#include <thread>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_split.h"
#include "absl/synchronization/mutex.h"
#include "flowstate_interfaces/srv/get_resource.hpp"
#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace flowstate_ros_bridge {

///=============================================================================
// The Node will retrieve information required for configuration from
// ROS Parameters.
class WorldBridge : public BridgeInterface {
 public:
  ~WorldBridge();

  using GetResource = flowstate_interfaces::srv::GetResource;

  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client) final;

 private:
  void TfCallback(const intrinsic_proto::TFMessage&);

  struct Data : public std::enable_shared_from_this<Data> {
    /**
     * @brief Send visualization messages for Flowstate sceneObjects
     *
     * @param object_names An optional vector of the names of SceneObjects to be
     * retrieved and published as Marker messages. If unspecified, all
     * SceneObjects in the Belief World will be retrieved and published.
     * @return absl::Status
     */
    absl::Status SendObjectVisualizationMessages(
        std::optional<std::vector<std::string>> object_names = std::nullopt);

    ROSNodeInterfaces node_interfaces_;
    std::shared_ptr<World> world_;
    std::shared_ptr<intrinsic::Subscription> tf_sub_;
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>
        workcell_markers_pub_;
    std::string tf_prefix_;
    std::shared_ptr<rclcpp::Service<GetResource>> get_resource_srv_;
    absl::flat_hash_map<std::string, std::vector<uint8_t>> renderables_;
    absl::flat_hash_set<std::string> tf_frame_names_;
    std::optional<std::vector<std::string>> send_object_names_
        ABSL_GUARDED_BY(mutex_) = std::nullopt;
    bool send_new_objects_ ABSL_GUARDED_BY(mutex_) = true;
    std::shared_ptr<std::thread> viz_thread_;
    absl::Mutex mutex_;  // protects send_object_names_, send_new_objects_
    std::string mesh_url_prefix_;
    ~Data();
  };
  std::shared_ptr<Data> data_;
};

}  // namespace flowstate_ros_bridge.

#endif  // BRIDGES__WORLD_BRIDGE_HPP_
