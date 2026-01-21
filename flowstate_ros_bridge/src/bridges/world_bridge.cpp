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

#include "world_bridge.hpp"

#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/eigen.h"
#include "tf2_ros/qos.hpp"

namespace flowstate_ros_bridge {

constexpr const char* kTfPrefixParamName = "world_tf_prefix";
constexpr const char* kResourceServiceName = "flowstate_get_resource";
constexpr const char* kMeshUrlPrefixParamName = "mesh_url_prefix";
constexpr const char* kEnableRobotStateBridgeParamName = "enable_robot_state_topic";
constexpr const char* kEnableGripperStateBridgeParamName = "enable_gripper_state_topic";
constexpr const char* kEnableForceTorqueBridgeParamName = "enable_force_torque_topic";
constexpr const char* kEnableCameraStreamBridgeParamName = "enable_camera_stream_topic";
constexpr const char* kRobotStatusTopicName = "robot_status_topic_name";

///=============================================================================
void WorldBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kTfPrefixParamName,
                                     rclcpp::ParameterValue{""});
  param_interface->declare_parameter(
      kMeshUrlPrefixParamName,
      rclcpp::ParameterValue{"http://localhost:8123/"});
  param_interface->declare_parameter(kEnableRobotStateBridgeParamName, rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kEnableGripperStateBridgeParamName, rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kEnableForceTorqueBridgeParamName, rclcpp::ParameterValue(true));
  param_interface->declare_parameter(kEnableCameraStreamBridgeParamName, rclcpp::ParameterValue(false));
  param_interface->declare_parameter(kRobotStatusTopicName, rclcpp::ParameterValue("/icon/robot_controller/"
                                                                                   "robot_status_throttle"));
}

///=============================================================================
bool WorldBridge::initialize(ROSNodeInterfaces ros_node_interfaces, std::shared_ptr<Executive> /*executive_client*/,
                             std::shared_ptr<World> world_client)
{
  data_ = std::make_shared<Data>();
  data_->node_interfaces_ = std::move(ros_node_interfaces);
  data_->world_ = std::move(world_client);

  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>
      param_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeParametersInterface>();

  data_->get_resource_srv_ = rclcpp::create_service<GetResource>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeServicesInterface>(),
      kResourceServiceName,
      [data_ = this->data_](const std::shared_ptr<GetResource::Request> request,
                            std::shared_ptr<GetResource::Response> response) {
        const std::string gltf_id = request->path;
        LOG(INFO) << "request resource path: " << gltf_id;
        if (!data_->renderables_.contains(gltf_id)) {
          response->status_code = GetResource::Response::ERROR;
          return;
        }
        response->status_code = GetResource::Response::OK;
        response->body = data_->renderables_[gltf_id];
      },
      rclcpp::ServicesQoS(), nullptr);

  data_->tf_prefix_ = param_interface->get_parameter(kTfPrefixParamName)
                          .get_value<std::string>();

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();

  data_->tf_pub_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
      param_interface, topics_interface, "tf",
      tf2_ros::DynamicBroadcasterQoS());

  const rclcpp::QoS markers_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  data_->workcell_markers_pub_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
          param_interface, topics_interface, "workcell_markers", markers_qos);

  data_->mesh_url_prefix_ =
      param_interface->get_parameter(kMeshUrlPrefixParamName)
          .get_value<std::string>();

  auto tf_sub_ = data_->world_->CreateTfSubscription(
      [this](const intrinsic_proto::TFMessage& msg) { this->TfCallback(msg); });
  if (!tf_sub_.ok()) {
    LOG(ERROR) << "Unable to create TF Subscription: " << tf_sub_.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate TF topic";
  data_->tf_sub_ = std::move(*tf_sub_);

  // Robot States Bridge
  data_->robot_state_topic_enabled_ = param_interface->get_parameter(kEnableRobotStateBridgeParamName).as_bool();
  data_->gripper_states_topic_enabled_ = param_interface->get_parameter(kEnableGripperStateBridgeParamName).as_bool();
  data_->force_torque_topic_enabled_ = param_interface->get_parameter(kEnableForceTorqueBridgeParamName).as_bool();
  data_->camera_stream_topic_enabled_ = param_interface->get_parameter(kEnableCameraStreamBridgeParamName).as_bool();
  LOG(INFO) << "Robot State Bridge Enabled: " << data_->robot_state_topic_enabled_;
  LOG(INFO) << "Gripper States Bridge Enabled: " << data_->gripper_states_topic_enabled_;
  LOG(INFO) << "Force Torque Bridge Enabled: " << data_->force_torque_topic_enabled_;
  LOG(INFO) << "Camera Stream Bridge Enabled: " << data_->camera_stream_topic_enabled_;

  // Create ROS publisheres
  data_->robot_state_pub_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(
      param_interface, topics_interface, "robot_state", rclcpp::SystemDefaultsQoS());
  data_->gripper_state_pub_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(
      param_interface, topics_interface, "gripper_states", rclcpp::SystemDefaultsQoS());
  data_->force_torque_pub_ = rclcpp::create_publisher<geometry_msgs::msg::WrenchStamped>(
      param_interface, topics_interface, "force_torque_sensors", rclcpp::SystemDefaultsQoS());
  data_->camera_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(param_interface, topics_interface,
                                                                         "camera_stream", rclcpp::SystemDefaultsQoS());

  // Create Flowstate subscriptions
  std::string robot_state_topic_name = param_interface->get_parameter(kRobotStatusTopicName).get_value<std::string>();
  LOG(INFO) << "Robot Status Topic Name: " << robot_state_topic_name;
  auto robot_state_sub = data_->world_->CreateRobotStateSubscription(
      robot_state_topic_name,
      [this](const intrinsic_proto::data_logger::LogItem& msg) { this->RobotStateCallback(msg); });
  if (!robot_state_sub.ok())
  {
    LOG(ERROR) << "Unable to create Robot State Subscription: " << robot_state_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate Robot State topic";
  data_->robot_state_sub_ = std::move(*robot_state_sub);

  // Camera Stream Bridge
  auto camera_sub = data_->world_->CreateCameraSubscription(
      [this](const intrinsic_proto::perception::SensorImage& msg) { this->CameraCallback(msg); });
  if (!camera_sub.ok())
  {
    LOG(ERROR) << "Unable to create Camera Stream Subscription: " << camera_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate Camera Stream topic";
  data_->camera_sub_ = std::move(*camera_sub);

  // Start a thread to publish sceneObject visualization messages whenever a new
  // object arrives
  std::weak_ptr<Data> data_wp = data_;
  data_->viz_thread_ = std::make_shared<std::thread>([data_wp]() {
    while (rclcpp::ok()) {
      if (auto data = data_wp.lock()) {
        data->mutex_.LockWhen(absl::Condition(
            +[](bool* condn) { return *condn; }, &data->send_new_objects_));

        const absl::Status status =
            data->SendObjectVisualizationMessages(data->send_object_names_);
        if (!status.ok()) {
          LOG(ERROR) << "Unable to send object visualization messages: "
                     << status.message();
          continue;
        }
        if (data->send_object_names_.has_value()) {
          // Clear object names if object retrival and publishing is successful
          data->send_object_names_.value().clear();
        }
        data->send_new_objects_ = false;

        data->mutex_.Unlock();
      } else {
        LOG(ERROR) << "data has expired! Terminating thread sending "
                      "visualization objects";
        return;
      }
    }
  });

  return true;
}

absl::Status WorldBridge::Data::SendObjectVisualizationMessages(
    std::optional<std::vector<std::string>> object_names) {
  absl::StatusOr<std::vector<intrinsic::world::WorldObject>> objects =
      world_->GetObjects(std::move(object_names));
  if (!objects.ok()) {
    return objects.status();
  }

  LOG(INFO) << "Retrieved " << objects->size() << " world objects:";

  size_t total_gltf_size = 0;
  visualization_msgs::msg::MarkerArray array_msg;
  rclcpp::Clock clock;
  const rclcpp::Time t = clock.now();

  for (const intrinsic::world::WorldObject& object : *objects) {
    int object_id = 0;
    const intrinsic_proto::world::Object& proto = object.Proto();
    for (const auto& entity : proto.entities()) {
      if (!entity.second.has_geometry_component()) {
        continue;
      }
      for (const auto& named_geometry :
           entity.second.geometry_component().named_geometries()) {
        // It seems the canonical constant for Intrinsic_Visual is not
        // available externally, so it needs to be hard-coded here.
        if (named_geometry.first != "Intrinsic_Visual") continue;
        for (const intrinsic_proto::world::GeometryComponent::Geometry&
                 geometry : named_geometry.second.geometries()) {
          const std::string renderable =
              geometry.geometry_storage_refs().renderable_ref();
          const std::string tf_frame_name =
              absl::StrFormat("%s%s/%s", tf_prefix_.c_str(),
                              object.Name().value(), entity.second.name());
          // Let's be smarter in the future. For now, just skip over
          // the intcas:// prefix
          const std::string gltf_path = absl::StrFormat(
              "gltf/%s_%s.glb",
              geometry.geometry_storage_refs().geometry_ref().substr(9),
              geometry.geometry_storage_refs().renderable_ref().substr(9));

          const auto renderable_name = std::string("/") + gltf_path;
          auto renderables_it = renderables_.find(renderable_name);

          if (renderables_it == renderables_.end()) {
            const absl::StatusOr<std::string> gltf = world_->GetGltf(
                geometry.geometry_storage_refs().geometry_ref(),
                geometry.geometry_storage_refs().renderable_ref());
            if (!gltf.ok()) {
              LOG(ERROR) << "Unable to fetch renderable for " << tf_frame_name
                         << ": " << gltf.status();
              continue;
            }
            total_gltf_size += gltf->size();
            std::vector<uint8_t> gltf_data;
            gltf_data.resize(gltf->size());
            memcpy(&gltf_data[0], gltf->data(), gltf->size());

            LOG(INFO) << "Fetched " << gltf->size() << " bytes for "
                      << tf_frame_name;
            renderables_it =
                renderables_.emplace(renderable_name, std::move(gltf_data))
                    .first;
          }

          const absl::StatusOr<intrinsic::eigenmath::MatrixXd> transform_xd =
              intrinsic_proto::FromProto(geometry.ref_t_shape_aff());
          if (!transform_xd.ok()) continue;
          // An intermediate 4d matrix seems necessary for template inference to
          // work as expected when converting to an AffineTransform3d later.
          const intrinsic::eigenmath::Matrix4d transform_4d = *transform_xd;
          const intrinsic::eigenmath::AffineTransform3d affine(transform_4d);

          visualization_msgs::msg::Marker marker_msg;
          marker_msg.header.frame_id = tf_frame_name;
          marker_msg.header.stamp = t;
          marker_msg.ns = tf_frame_name;
          marker_msg.id = object_id++;
          marker_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
          marker_msg.action = visualization_msgs::msg::Marker::ADD;
          marker_msg.pose.position.x = affine.translation().x();
          marker_msg.pose.position.y = affine.translation().y();
          marker_msg.pose.position.z = affine.translation().z();
          const intrinsic::eigenmath::Quaterniond quat(affine.rotation());
          marker_msg.pose.orientation.x = quat.x();
          marker_msg.pose.orientation.y = quat.y();
          marker_msg.pose.orientation.z = quat.z();
          marker_msg.pose.orientation.w = quat.w();
          // Currently all of our meshes have unit scaling. If this changes,
          // we could use Transform::computeRotationScaling() but that costs
          // a SVD, and it doesn't seem worth it when _all_ of our meshes are
          // already at unit scale.
          marker_msg.scale.x = 1.0;
          marker_msg.scale.y = 1.0;
          marker_msg.scale.z = 1.0;
          // Leave color as (1, 1, 1, 1) so that the mesh color is as expected
          // from the embedded glTF textures.
          marker_msg.color.r = 1.0;
          marker_msg.color.g = 1.0;
          marker_msg.color.b = 1.0;
          marker_msg.color.a = 1.0;
          // Set lifetime to (0, 0) to indicate these meshes never expire.
          marker_msg.lifetime.sec = 0;
          marker_msg.lifetime.nanosec = 0;
          // Lock the mesh to its TF frame so that motion is handled correctly
          marker_msg.frame_locked = true;
          // Set the mesh resource path to the HTTP/rmw_zenoh proxy
          marker_msg.mesh_resource = mesh_url_prefix_ + gltf_path;
          marker_msg.mesh_use_embedded_materials = true;

          array_msg.markers.push_back(std::move(marker_msg));
        }
      }
      // LOG(INFO) << entity.second;
    }
  }
  LOG(INFO) << "Total gltf size: " << total_gltf_size << " bytes";

  if (!array_msg.markers.empty()) {
    workcell_markers_pub_->publish(array_msg);
  }

  return absl::OkStatus();
}

///=============================================================================
WorldBridge::Data::~Data() {}

void WorldBridge::TfCallback(const intrinsic_proto::TFMessage& tf_proto) {
  rclcpp::Clock clock;
  const rclcpp::Time t_start = clock.now();

  absl::flat_hash_set<std::string> new_tf_frame_names;
  absl::flat_hash_set<std::string> new_object_names;

  tf2_msgs::msg::TFMessage tf_ros;
  tf_ros.transforms = std::vector<geometry_msgs::msg::TransformStamped>(
      tf_proto.transforms_size());
  int tf_idx = 0;
  for (const auto& ts_proto : tf_proto.transforms()) {
    geometry_msgs::msg::TransformStamped* ts_ros = &tf_ros.transforms[tf_idx++];
    ts_ros->header.stamp.sec = ts_proto.header().stamp().seconds();
    ts_ros->header.stamp.nanosec = ts_proto.header().stamp().nanos();
    ts_ros->header.frame_id = data_->tf_prefix_ + ts_proto.header().frame_id();
    ts_ros->child_frame_id = data_->tf_prefix_ + ts_proto.child_frame_id();

    new_tf_frame_names.insert(ts_ros->child_frame_id);
    if (!data_->tf_frame_names_.contains(ts_ros->child_frame_id)) {
      // We parse the "OBJECT_NAME/ENTITY_NAME" string to get the OBJECT_NAME
      LOG(INFO) << "new child_frame_id: " << ts_ros->child_frame_id;
      const std::size_t str_end = ts_proto.child_frame_id().find('/');
      new_object_names.insert(ts_proto.child_frame_id().substr(0, str_end));
    }
    // The auto-generated CDR types do not currently have assignment operators
    // or helper conversion functions from the corresponding protos, so we need
    // to explicitly copy all the fields.
    const auto& t = ts_proto.transform();  // just to save some typing
    ts_ros->transform.translation.x = t.translation().x();
    ts_ros->transform.translation.y = t.translation().y();
    ts_ros->transform.translation.z = t.translation().z();
    ts_ros->transform.rotation.x = t.rotation().x();
    ts_ros->transform.rotation.y = t.rotation().y();
    ts_ros->transform.rotation.z = t.rotation().z();
    ts_ros->transform.rotation.w = t.rotation().w();
  }
  data_->tf_pub_->publish(tf_ros);

  // print a timing snapshot every 10 seconds
  static int count = 0;
  if (count++ % 500 == 0) {
    const rclcpp::Duration elapsed = clock.now() - t_start;
    LOG(INFO) << "tf translation time: " << (1000.0 * elapsed.seconds())
              << " ms";
  }

  std::vector<std::string> deleted_tf_frames;
  for (const auto& tf_frame : data_->tf_frame_names_) {
    if (!new_tf_frame_names.contains(tf_frame)) {
      deleted_tf_frames.push_back(tf_frame);
    }
  }
  if (!deleted_tf_frames.empty()) {
    visualization_msgs::msg::MarkerArray array_msg;
    for (const auto& tf_frame : deleted_tf_frames) {
      LOG(INFO) << "Removed sceneObject with frame_id " << tf_frame
                << " in the world, updating visualization markers.";

      visualization_msgs::msg::Marker marker_msg;
      marker_msg.ns = tf_frame;
      marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;

      array_msg.markers.push_back(std::move(marker_msg));
    }
    data_->workcell_markers_pub_->publish(array_msg);
  }

  if (!new_object_names.empty()) {
    data_->mutex_.Lock();
    if (data_->send_object_names_.has_value()) {
      data_->send_object_names_.value().insert(
          data_->send_object_names_.value().end(), new_object_names.begin(),
          new_object_names.end());
    } else {
      data_->send_object_names_ = std::vector<std::string>(
          new_object_names.begin(), new_object_names.end());
    }
    // Signal background thread to send object visualization messages
    data_->send_new_objects_ = true;
    data_->mutex_.Unlock();
  }

  data_->tf_frame_names_ = std::move(new_tf_frame_names);
}

///=============================================================================
void WorldBridge::RobotStateCallback(const intrinsic_proto::data_logger::LogItem& log_item)
{
  rclcpp::Clock clock;
  const rclcpp::Time t_start = clock.now();

  const auto& payload = log_item.payload();

  switch (payload.data_case())
  {
    case intrinsic_proto::data_logger::LogItem::Payload::kIconRobotStatus: {
      LOG(INFO) << "Received RobotStatus:\n" << payload.icon_robot_status().DebugString();
      break;
    }

    case intrinsic_proto::data_logger::LogItem::Payload::kIconL1JointState: {
      LOG(INFO) << "Received JointState:\n" << payload.icon_l1_joint_state().DebugString();
      break;
    }

    case intrinsic_proto::data_logger::LogItem::Payload::kIconFtWrench: {
      LOG(INFO) << "Received FT Wrench:\n" << payload.icon_ft_wrench().DebugString();
      break;
    }

    default: {
      break;
    }
  }

  // for (const auto& entry : robot_state_proto.status_map()) {
  //   const std::string& part_name = entry.first;
  //   const auto& part_status = entry.second;
  //   LOG(INFO) << "Part: " << part_name << ", Joint States: " << part_status.joint_states_size();
  //   for (int i = 0; i < part_status.joint_states_size(); ++i) {
  //     const auto& joint_state = part_status.joint_states(i);
  //     if (joint_state.has_position_sensed()) {
  //       LOG(INFO) << "  Joint " << i << " position: " << joint_state.position_sensed();
  //     }
  //     if (joint_state.has_velocity_sensed()) {
  //       LOG(INFO) << "  Joint " << i << " velocity: " << joint_state.velocity_sensed();
  //     }
  //     if (joint_state.has_acceleration_sensed()) {
  //       LOG(INFO) << "  Joint " << i << " acceleration: " << joint_state.acceleration_sensed();
  //     }
  //     if (joint_state.has_torque_sensed()) {
  //       LOG(INFO) << "  Joint " << i << " torque: " << joint_state.torque_sensed();
  //     }
  //   }

  //   if (part_status.has_gripper_state()) {
  //     LOG(INFO) << "  Gripper State: " << part_status.gripper_state().sensed_state();
  //   }
  // }

  // TODO: Translate
  // sensor_msgs::msg::JointState robot_state_ros;
  // data_->robot_state_pub_->publish(robot_state_ros);

  // TODO: Use enable flags, extract gripper and force torque sensors and republish in ROS
  const rclcpp::Duration elapsed = clock.now() - t_start;
  LOG(INFO) << "Robot state translation time: " << (1000.0 * elapsed.seconds()) << " ms";
  LOG(INFO) << "=======================================================" << std::endl;
}

///=============================================================================
void WorldBridge::CameraCallback(const intrinsic_proto::perception::SensorImage& camera_proto)
{
  if (data_->camera_stream_topic_enabled_)
  {
    rclcpp::Clock clock;
    const rclcpp::Time t_start = clock.now();

    sensor_msgs::msg::Image camera_ros;
    // camera_ros.header.stamp.sec = camera_proto.header().stamp().seconds();
    // camera_ros.header.stamp.nanosec = camera_proto.header().stamp().nanos();
    // camera_ros.header.frame_id = data_->tf_prefix_ + camera_proto.header().frame_id();
    // Populate camera_ros from camera_proto
    // This is a skeleton, actual conversion logic would go here.
    // For example, if camera_proto contains raw image data:
    // camera_ros.height = camera_proto.height();
    // camera_ros.width = camera_proto.width();
    // camera_ros.encoding = camera_proto.encoding(); // e.g., "rgb8", "mono8"
    // camera_ros.is_bigendian = 0; // or 1 if applicable
    // camera_ros.step = camera_proto.step(); // row length in bytes
    // camera_ros.data.assign(camera_proto.data().begin(), camera_proto.data().end());

    data_->camera_pub_->publish(camera_ros);

    const rclcpp::Duration elapsed = clock.now() - t_start;
    LOG(INFO) << "Camera stream translation time: " << (1000.0 * elapsed.seconds()) << " ms";
  }
}

WorldBridge::~WorldBridge() {
  if (data_->viz_thread_ && data_->viz_thread_->joinable()) {
    data_->viz_thread_->join();
  }
  data_->viz_thread_.reset();
}

}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::WorldBridge,
                       flowstate_ros_bridge::BridgeInterface)
