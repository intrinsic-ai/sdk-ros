// Copyright 2026 Intrinsic Innovation LLC
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

#ifndef FLOWSTATE_ROS_BRIDGE__BRIDGES__IMAGE_BRIDGE_HPP_
#define FLOWSTATE_ROS_BRIDGE__BRIDGES__IMAGE_BRIDGE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "flowstate_ros_bridge/bridge_interface.hpp"
#include "intrinsic/perception/proto/v1/image_buffer.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flowstate_ros_bridge {

class ImageBridge : public BridgeInterface {
 public:
  ~ImageBridge() override;

  /// Documentation inherited.
  void declare_ros_parameters(ROSNodeInterfaces ros_node_interfaces) final;

  /// Documentation inherited.
  bool initialize(ROSNodeInterfaces ros_node_interfaces,
                  std::shared_ptr<Executive> executive_client,
                  std::shared_ptr<World> world_client,
                  std::shared_ptr<intrinsic::PubSub> pubsub) final;

  /// Helper to convert an Intrinsic v1 ImageBuffer proto to a ROS 2
  /// sensor_msgs/Image.
  static bool ConvertImageBufferToRosImage(
      const intrinsic_proto::perception::v1::ImageBuffer& proto_img,
      const std::string& encoding_override, sensor_msgs::msg::Image& ros_img);

  /// Helper to determine the ROS encoding string from the v1 ImageBuffer proto.
  static std::string DetermineRosEncoding(
      const intrinsic_proto::perception::v1::ImageBuffer& proto_img);

  /// Helper to get the number of bytes per channel for a given v1 DataType.
  static size_t BytesPerChannel(intrinsic_proto::perception::v1::DataType type);

 private:
  struct TopicBridge {
    std::string pubsub_topic;
    std::string ros_topic;
    std::string frame_id;
    std::string encoding_override;
    std::shared_ptr<intrinsic::Subscription> subscription;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher;
  };

  struct Data : public std::enable_shared_from_this<Data> {
    ROSNodeInterfaces node_interfaces_;
    std::shared_ptr<intrinsic::PubSub> pubsub_;
    std::vector<TopicBridge> topic_bridges_;

    void HandleImageBuffer(
        size_t bridge_index,
        const intrinsic_proto::perception::v1::ImageBuffer& msg);

    ~Data();
  };

  std::shared_ptr<Data> data_;
};

}  // namespace flowstate_ros_bridge

#endif  // FLOWSTATE_ROS_BRIDGE__BRIDGES__IMAGE_BRIDGE_HPP_
