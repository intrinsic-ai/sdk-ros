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

#include "image_bridge.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/strings/ascii.h"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flowstate_ros_bridge {

constexpr const char* kImageTopicsParamName = "image_topics";
constexpr const char* kImagePubSubTopicsParamName = "image_pubsub_topics";
constexpr const char* kImageRosTopicsParamName = "image_ros_topics";
constexpr const char* kImageFrameIdsParamName = "image_frame_ids";
constexpr const char* kImageEncodingsParamName = "image_encodings";
constexpr const char* kImageDefaultFrameIdParamName = "image_default_frame_id";

///=============================================================================
size_t ImageBridge::BytesPerChannel(
    intrinsic_proto::perception::v1::DataType type) {
  switch (type) {
    case intrinsic_proto::perception::v1::TYPE_UINT8:
    case intrinsic_proto::perception::v1::TYPE_INT8:
      return 1;
    case intrinsic_proto::perception::v1::TYPE_UINT16:
    case intrinsic_proto::perception::v1::TYPE_INT16:
      return 2;
    case intrinsic_proto::perception::v1::TYPE_UINT32:
    case intrinsic_proto::perception::v1::TYPE_INT32:
    case intrinsic_proto::perception::v1::TYPE_FLOAT32:
      return 4;
    case intrinsic_proto::perception::v1::TYPE_FLOAT64:
      return 8;
    default:
      return 1;
  }
}

///=============================================================================
std::string ImageBridge::DetermineRosEncoding(
    const intrinsic_proto::perception::v1::ImageBuffer& proto_img) {
  const auto pixel_type = proto_img.pixel_type();
  const auto data_type = proto_img.type();
  const int32_t channels =
      (proto_img.num_channels() > 0) ? proto_img.num_channels() : 1;

  if (pixel_type == intrinsic_proto::perception::v1::PIXEL_INTENSITY) {
    if (channels == 1) {
      switch (data_type) {
        case intrinsic_proto::perception::v1::TYPE_UINT8:
          return sensor_msgs::image_encodings::MONO8;
        case intrinsic_proto::perception::v1::TYPE_UINT16:
          return sensor_msgs::image_encodings::MONO16;
        case intrinsic_proto::perception::v1::TYPE_FLOAT32:
          return sensor_msgs::image_encodings::TYPE_32FC1;
        case intrinsic_proto::perception::v1::TYPE_FLOAT64:
          return sensor_msgs::image_encodings::TYPE_64FC1;
        case intrinsic_proto::perception::v1::TYPE_INT8:
          return sensor_msgs::image_encodings::TYPE_8SC1;
        case intrinsic_proto::perception::v1::TYPE_INT16:
          return sensor_msgs::image_encodings::TYPE_16SC1;
        case intrinsic_proto::perception::v1::TYPE_INT32:
          return sensor_msgs::image_encodings::TYPE_32SC1;
        default:
          break;
      }
    } else if (channels == 3) {
      switch (data_type) {
        case intrinsic_proto::perception::v1::TYPE_UINT8:
          return sensor_msgs::image_encodings::RGB8;
        case intrinsic_proto::perception::v1::TYPE_UINT16:
          return sensor_msgs::image_encodings::RGB16;
        case intrinsic_proto::perception::v1::TYPE_FLOAT32:
          return sensor_msgs::image_encodings::TYPE_32FC3;
        case intrinsic_proto::perception::v1::TYPE_FLOAT64:
          return sensor_msgs::image_encodings::TYPE_64FC3;
        case intrinsic_proto::perception::v1::TYPE_INT8:
          return sensor_msgs::image_encodings::TYPE_8SC3;
        case intrinsic_proto::perception::v1::TYPE_INT16:
          return sensor_msgs::image_encodings::TYPE_16SC3;
        case intrinsic_proto::perception::v1::TYPE_INT32:
          return sensor_msgs::image_encodings::TYPE_32SC3;
        default:
          break;
      }
    } else if (channels == 4) {
      switch (data_type) {
        case intrinsic_proto::perception::v1::TYPE_UINT8:
          return sensor_msgs::image_encodings::RGBA8;
        case intrinsic_proto::perception::v1::TYPE_UINT16:
          return sensor_msgs::image_encodings::RGBA16;
        case intrinsic_proto::perception::v1::TYPE_FLOAT32:
          return sensor_msgs::image_encodings::TYPE_32FC4;
        case intrinsic_proto::perception::v1::TYPE_FLOAT64:
          return sensor_msgs::image_encodings::TYPE_64FC4;
        case intrinsic_proto::perception::v1::TYPE_INT8:
          return sensor_msgs::image_encodings::TYPE_8SC4;
        case intrinsic_proto::perception::v1::TYPE_INT16:
          return sensor_msgs::image_encodings::TYPE_16SC4;
        case intrinsic_proto::perception::v1::TYPE_INT32:
          return sensor_msgs::image_encodings::TYPE_32SC4;
        default:
          break;
      }
    }
  } else if (pixel_type == intrinsic_proto::perception::v1::PIXEL_DEPTH) {
    if (channels == 1) {
      switch (data_type) {
        case intrinsic_proto::perception::v1::TYPE_UINT16:
          return sensor_msgs::image_encodings::TYPE_16UC1;
        case intrinsic_proto::perception::v1::TYPE_FLOAT32:
          return sensor_msgs::image_encodings::TYPE_32FC1;
        case intrinsic_proto::perception::v1::TYPE_FLOAT64:
          return sensor_msgs::image_encodings::TYPE_64FC1;
        case intrinsic_proto::perception::v1::TYPE_UINT32:
          return "32UC1";
        default:
          break;
      }
    }
  }

  // Fallback to standard OpenCV-like types
  switch (data_type) {
    case intrinsic_proto::perception::v1::TYPE_UINT8:
      if (channels == 1) return sensor_msgs::image_encodings::MONO8;
      if (channels == 3) return sensor_msgs::image_encodings::RGB8;
      if (channels == 4) return sensor_msgs::image_encodings::RGBA8;
      return "8UC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_INT8:
      return "8SC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_UINT16:
      if (channels == 1) return sensor_msgs::image_encodings::MONO16;
      if (channels == 3) return sensor_msgs::image_encodings::RGB16;
      if (channels == 4) return sensor_msgs::image_encodings::RGBA16;
      return "16UC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_INT16:
      return "16SC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_UINT32:
      return "32UC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_INT32:
      return "32SC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_FLOAT32:
      return "32FC" + std::to_string(channels);
    case intrinsic_proto::perception::v1::TYPE_FLOAT64:
      return "64FC" + std::to_string(channels);
    default:
      if (channels == 1) return sensor_msgs::image_encodings::MONO8;
      if (channels == 3) return sensor_msgs::image_encodings::RGB8;
      if (channels == 4) return sensor_msgs::image_encodings::RGBA8;
      return "8UC" + std::to_string(channels);
  }
}

///=============================================================================
bool ImageBridge::ConvertImageBufferToRosImage(
    const intrinsic_proto::perception::v1::ImageBuffer& proto_img,
    const std::string& encoding_override, sensor_msgs::msg::Image& ros_img) {
  if (proto_img.encoding() !=
          intrinsic_proto::perception::v1::ENCODING_UNSPECIFIED &&
      proto_img.encoding() !=
          intrinsic_proto::perception::v1::ENCODING_YUV420P) {
    LOG(ERROR) << "Compressed ImageBuffer encoding (" << proto_img.encoding()
               << ") cannot be converted directly to uncompressed "
                  "sensor_msgs/Image";
    return false;
  }

  if (!proto_img.has_dimensions()) {
    LOG(ERROR) << "ImageBuffer is missing dimensions";
    return false;
  }

  const uint32_t width = proto_img.dimensions().cols();
  const uint32_t height = proto_img.dimensions().rows();
  if (width == 0 || height == 0) {
    ros_img.width = width;
    ros_img.height = height;
    ros_img.step = 0;
    ros_img.data.clear();
    return true;
  }

  const int32_t num_channels =
      (proto_img.num_channels() > 0) ? proto_img.num_channels() : 1;
  const size_t bytes_per_channel = BytesPerChannel(proto_img.type());
  const size_t expected_size =
      static_cast<size_t>(width) * height * num_channels * bytes_per_channel;

  if (proto_img.data().size() < expected_size) {
    LOG(ERROR) << "ImageBuffer data size (" << proto_img.data().size()
               << ") is smaller than expected (" << expected_size << ")";
    return false;
  }

  if (!encoding_override.empty()) {
    ros_img.encoding = encoding_override;
  } else if (proto_img.encoding() ==
             intrinsic_proto::perception::v1::ENCODING_YUV420P) {
    ros_img.encoding = "yuv420";
  } else {
    ros_img.encoding = DetermineRosEncoding(proto_img);
  }

  if (ros_img.encoding.empty()) {
    LOG(ERROR) << "Failed to determine ROS image encoding for ImageBuffer";
    return false;
  }

  ros_img.width = width;
  ros_img.height = height;
  ros_img.step = width * num_channels * bytes_per_channel;
  ros_img.is_bigendian = 0;

  if (proto_img.has_packing_type() &&
      proto_img.packing_type() ==
          intrinsic_proto::perception::v1::PACKING_TYPE_PLANAR &&
      num_channels > 1) {
    ros_img.data.resize(expected_size);
    const auto* src_ptr =
        reinterpret_cast<const uint8_t*>(proto_img.data().data());
    uint8_t* dst_ptr = ros_img.data.data();
    const size_t total_pixels = static_cast<size_t>(width) * height;
    for (size_t p = 0; p < total_pixels; ++p) {
      for (int32_t c = 0; c < num_channels; ++c) {
        const size_t src_idx = (c * total_pixels + p) * bytes_per_channel;
        const size_t dst_idx = (p * num_channels + c) * bytes_per_channel;
        std::memcpy(dst_ptr + dst_idx, src_ptr + src_idx, bytes_per_channel);
      }
    }
  } else {
    ros_img.data.assign(
        reinterpret_cast<const uint8_t*>(proto_img.data().data()),
        reinterpret_cast<const uint8_t*>(proto_img.data().data()) +
            expected_size);
  }

  return true;
}

///=============================================================================
void ImageBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(
      kImageTopicsParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kImagePubSubTopicsParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kImageRosTopicsParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kImageFrameIdsParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(
      kImageEncodingsParamName,
      rclcpp::ParameterValue(std::vector<std::string>{}));
  param_interface->declare_parameter(kImageDefaultFrameIdParamName,
                                     rclcpp::ParameterValue(std::string{""}));
}

///=============================================================================
bool ImageBridge::initialize(ROSNodeInterfaces ros_node_interfaces,
                             std::shared_ptr<Executive> /*executive_client*/,
                             std::shared_ptr<World> world_client,
                             std::shared_ptr<intrinsic::PubSub> pubsub) {
  if (!pubsub && world_client) {
    pubsub = world_client->pubsub();
  }
  if (!pubsub) {
    LOG(ERROR) << "ImageBridge requires a valid PubSub instance";
    return false;
  }

  data_ = std::make_shared<Data>();
  data_->node_interfaces_ = std::move(ros_node_interfaces);
  data_->pubsub_ = std::move(pubsub);

  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
      param_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeParametersInterface>();
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();

  const std::string default_frame_id =
      param_interface->get_parameter(kImageDefaultFrameIdParamName).as_string();

  // 1. Process structured entries from "image_topics"
  const std::vector<std::string> topic_mappings =
      param_interface->get_parameter(kImageTopicsParamName).as_string_array();
  for (const auto& entry : topic_mappings) {
    std::string trimmed = entry;
    absl::StripAsciiWhitespace(&trimmed);
    if (trimmed.empty()) {
      continue;
    }

    std::string pubsub_topic;
    std::string ros_topic;
    std::string frame_id = default_frame_id;
    std::string encoding;

    if (absl::StrContains(trimmed, "->")) {
      std::pair<std::string_view, std::string_view> arrow_parts =
          absl::StrSplit(trimmed, absl::MaxSplits("->", 1));
      pubsub_topic = std::string(arrow_parts.first);
      std::string_view rhs = arrow_parts.second;
      std::vector<std::string_view> rhs_parts = absl::StrSplit(rhs, ':');
      if (!rhs_parts.empty()) {
        ros_topic = std::string(rhs_parts[0]);
      }
      if (rhs_parts.size() > 1) {
        frame_id = std::string(rhs_parts[1]);
      }
      if (rhs_parts.size() > 2) {
        encoding = std::string(rhs_parts[2]);
      }
    } else if (absl::StrContains(trimmed, ",")) {
      std::vector<std::string_view> parts = absl::StrSplit(trimmed, ',');
      if (!parts.empty()) {
        pubsub_topic = std::string(parts[0]);
      }
      if (parts.size() > 1) {
        ros_topic = std::string(parts[1]);
      }
      if (parts.size() > 2) {
        frame_id = std::string(parts[2]);
      }
      if (parts.size() > 3) {
        encoding = std::string(parts[3]);
      }
    } else if (absl::StrContains(trimmed, ":")) {
      std::vector<std::string_view> parts = absl::StrSplit(trimmed, ':');
      if (!parts.empty()) {
        pubsub_topic = std::string(parts[0]);
      }
      if (parts.size() > 1) {
        ros_topic = std::string(parts[1]);
      }
      if (parts.size() > 2) {
        frame_id = std::string(parts[2]);
      }
      if (parts.size() > 3) {
        encoding = std::string(parts[3]);
      }
    } else {
      std::vector<std::string_view> parts = absl::StrSplit(
          trimmed, absl::ByAnyChar(" \t"), absl::SkipWhitespace());
      if (!parts.empty()) {
        pubsub_topic = std::string(parts[0]);
      }
      if (parts.size() > 1) {
        ros_topic = std::string(parts[1]);
      }
      if (parts.size() > 2) {
        frame_id = std::string(parts[2]);
      }
      if (parts.size() > 3) {
        encoding = std::string(parts[3]);
      }
    }

    absl::StripAsciiWhitespace(&pubsub_topic);
    absl::StripAsciiWhitespace(&ros_topic);
    absl::StripAsciiWhitespace(&frame_id);
    absl::StripAsciiWhitespace(&encoding);

    if (pubsub_topic.empty() || ros_topic.empty()) {
      LOG(ERROR) << "Invalid image topic mapping entry [" << entry
                 << "]: pubsub_topic or ros_topic is empty";
      return false;
    }

    TopicBridge tb;
    tb.pubsub_topic = pubsub_topic;
    tb.ros_topic = ros_topic;
    tb.frame_id = frame_id;
    tb.encoding_override = encoding;
    data_->topic_bridges_.push_back(std::move(tb));
  }

  // 2. Process parallel arrays "image_pubsub_topics" and "image_ros_topics"
  const std::vector<std::string> pubsub_topics =
      param_interface->get_parameter(kImagePubSubTopicsParamName)
          .as_string_array();
  const std::vector<std::string> ros_topics =
      param_interface->get_parameter(kImageRosTopicsParamName)
          .as_string_array();
  const std::vector<std::string> frame_ids =
      param_interface->get_parameter(kImageFrameIdsParamName).as_string_array();
  const std::vector<std::string> encodings =
      param_interface->get_parameter(kImageEncodingsParamName)
          .as_string_array();

  if (pubsub_topics.size() != ros_topics.size()) {
    LOG(ERROR) << "Size mismatch: " << kImagePubSubTopicsParamName << " ("
               << pubsub_topics.size() << ") and " << kImageRosTopicsParamName
               << " (" << ros_topics.size() << ") must have same length";
    return false;
  }

  for (size_t i = 0; i < pubsub_topics.size(); ++i) {
    std::string pubsub_topic = pubsub_topics[i];
    std::string ros_topic = ros_topics[i];
    absl::StripAsciiWhitespace(&pubsub_topic);
    absl::StripAsciiWhitespace(&ros_topic);
    if (pubsub_topic.empty() || ros_topic.empty()) {
      LOG(ERROR) << "Invalid empty topic at index " << i;
      return false;
    }
    TopicBridge tb;
    tb.pubsub_topic = pubsub_topic;
    tb.ros_topic = ros_topic;
    tb.frame_id = (i < frame_ids.size() && !frame_ids[i].empty())
                      ? frame_ids[i]
                      : default_frame_id;
    tb.encoding_override = (i < encodings.size()) ? encodings[i] : "";
    data_->topic_bridges_.push_back(std::move(tb));
  }

  if (data_->topic_bridges_.empty()) {
    LOG(WARNING) << "No image topics configured for ImageBridge";
    return true;
  }

  for (size_t i = 0; i < data_->topic_bridges_.size(); ++i) {
    auto& tb = data_->topic_bridges_[i];
    tb.publisher = rclcpp::create_publisher<sensor_msgs::msg::Image>(
        param_interface, topics_interface, tb.ros_topic,
        rclcpp::SystemDefaultsQoS());

    auto sub =
        data_->pubsub_
            ->CreateSubscription<intrinsic_proto::perception::v1::ImageBuffer>(
                tb.pubsub_topic, intrinsic::TopicConfig(),
                [data = data_,
                 i](const intrinsic_proto::perception::v1::ImageBuffer& msg) {
                  data->HandleImageBuffer(i, msg);
                });
    if (!sub.ok()) {
      LOG(ERROR) << "Failed to subscribe to Intrinsic PubSub topic ["
                 << tb.pubsub_topic << "]: " << sub.status();
      return false;
    }
    tb.subscription =
        std::make_shared<intrinsic::Subscription>(std::move(*sub));
    LOG(INFO) << "ImageBridge: Subscribed Intrinsic PubSub [" << tb.pubsub_topic
              << "] -> Publishing ROS Image [" << tb.ros_topic
              << "] (frame_id: \"" << tb.frame_id << "\")";
  }

  return true;
}

///=============================================================================
void ImageBridge::Data::HandleImageBuffer(
    size_t bridge_index,
    const intrinsic_proto::perception::v1::ImageBuffer& msg) {
  if (bridge_index >= topic_bridges_.size()) {
    return;
  }
  const auto& bridge = topic_bridges_[bridge_index];
  if (!bridge.publisher) {
    return;
  }

  sensor_msgs::msg::Image ros_img;
  auto clock_interface =
      node_interfaces_.get<rclcpp::node_interfaces::NodeClockInterface>();
  ros_img.header.stamp = clock_interface->get_clock()->now();
  ros_img.header.frame_id = bridge.frame_id;

  if (!ConvertImageBufferToRosImage(msg, bridge.encoding_override, ros_img)) {
    LOG(ERROR) << "Failed to convert ImageBuffer from topic ["
               << bridge.pubsub_topic << "]";
    return;
  }

  bridge.publisher->publish(std::move(ros_img));
}

///=============================================================================
ImageBridge::Data::~Data() {
  for (auto& bridge : topic_bridges_) {
    bridge.subscription.reset();
    bridge.publisher.reset();
  }
}

///=============================================================================
ImageBridge::~ImageBridge() = default;

}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::ImageBridge,
                       flowstate_ros_bridge::BridgeInterface)
