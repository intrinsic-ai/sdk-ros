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

#include <gtest/gtest.h>
#include <rmw/rmw.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "aquarium_gateway/serialization.hpp"

// Generated Proto Headers (local)
#include "test_perception_messages.pb.h"

// Official ROS 2 Headers (C++)
#include "aquarium/srv/capture.hpp"

// Official ROS 2 Headers (C)
#include "aquarium/srv/detail/capture__functions.h"
#include "aquarium/srv/detail/capture__struct.h"
#include "aquarium/srv/detail/capture__type_support.h"

using namespace aquarium_gateway;

// Generic helper to convert C type description to C++
type_description_interfaces::msg::TypeDescription to_cpp_type_desc(
    const rosidl_runtime_c__type_description__TypeDescription* c_desc) {
  type_description_interfaces::msg::TypeDescription cpp_desc;

  // 1. Map individual type description
  cpp_desc.type_description.type_name = c_desc->type_description.type_name.data;
  for (size_t i = 0; i < c_desc->type_description.fields.size; ++i) {
    const auto& c_field = c_desc->type_description.fields.data[i];
    type_description_interfaces::msg::Field cpp_field;
    cpp_field.name = c_field.name.data;
    cpp_field.type.type_id = c_field.type.type_id;
    cpp_field.type.capacity = c_field.type.capacity;
    cpp_field.type.string_capacity = c_field.type.string_capacity;
    if (c_field.type.nested_type_name.data) {
      cpp_field.type.nested_type_name = c_field.type.nested_type_name.data;
    }
    cpp_desc.type_description.fields.push_back(cpp_field);
  }

  // 2. Map referenced types
  for (size_t i = 0; i < c_desc->referenced_type_descriptions.size; ++i) {
    const auto& c_ref = c_desc->referenced_type_descriptions.data[i];
    type_description_interfaces::msg::IndividualTypeDescription cpp_ref;
    cpp_ref.type_name = c_ref.type_name.data;
    for (size_t j = 0; j < c_ref.fields.size; ++j) {
      const auto& c_field = c_ref.fields.data[j];
      type_description_interfaces::msg::Field cpp_field;
      cpp_field.name = c_field.name.data;
      cpp_field.type.type_id = c_field.type.type_id;
      cpp_field.type.capacity = c_field.type.capacity;
      cpp_field.type.string_capacity = c_field.type.string_capacity;
      if (c_field.type.nested_type_name.data) {
        cpp_field.type.nested_type_name = c_field.type.nested_type_name.data;
      }
      cpp_ref.fields.push_back(cpp_field);
    }
    cpp_desc.referenced_type_descriptions.push_back(cpp_ref);
  }

  return cpp_desc;
}

template <typename ProtoMsg, typename ROSCppMsg, typename ROSCStruct>
void VerifySerialization(
    const ProtoMsg& proto_msg, const ROSCppMsg& cpp_msg,
    const rosidl_runtime_c__type_description__TypeDescription* c_type_desc,
    const rosidl_message_type_support_t* ts, bool (*c_init_func)(ROSCStruct*),
    void (*c_fini_func)(ROSCStruct*)) {
  auto type_desc = to_cpp_type_desc(c_type_desc);
  auto layout = ComputeDynamicLayout(type_desc.type_description, type_desc);

  // =========================================================================
  // PATH 1: Proto -> Dynamic C-struct -> Dynamic CDR -> C-struct -> Proto
  // =========================================================================

  // 1. Map Protobuf -> C-struct
  std::vector<std::unique_ptr<uint8_t[]>> allocations;
  auto c_struct_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct = reinterpret_cast<ROSCStruct*>(c_struct_buf.get());
  ASSERT_TRUE(c_init_func(c_struct));

  MapProtobufToCStruct(proto_msg, layout, c_struct_buf.get(), allocations,
                       type_desc);

  // 2. Serialize C-struct to CDR using RMW
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(rmw_serialized_message_init(&serialized_msg, 0, &allocator),
            RMW_RET_OK);
  ASSERT_EQ(rmw_serialize(c_struct, ts, &serialized_msg), RMW_RET_OK);

  // 3. Deserialize Dynamic CDR -> C-struct
  auto c_struct_dest_dyn_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_dest_dyn_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct_dest_dyn =
      reinterpret_cast<ROSCStruct*>(c_struct_dest_dyn_buf.get());
  ASSERT_TRUE(c_init_func(c_struct_dest_dyn));

  ASSERT_EQ(rmw_deserialize(&serialized_msg, ts, c_struct_dest_dyn),
            RMW_RET_OK);

  // 4. Map C-struct -> Protobuf
  ProtoMsg proto_msg_dest_dyn;
  MapCStructToProtobuf(layout, c_struct_dest_dyn_buf.get(), proto_msg_dest_dyn,
                       type_desc);

  // 5. Compare Protobuf
  // Normalize original proto by mapping it to C-struct and back to Proto
  // to account for one-way lossy mappings (like optional nested messages).
  ProtoMsg proto_msg_norm;
  {
    std::vector<std::unique_ptr<uint8_t[]>> allocs;
    auto buf = std::make_unique<uint8_t[]>(layout.total_size);
    std::memset(buf.get(), 0, layout.total_size);
    ROSCStruct* s = reinterpret_cast<ROSCStruct*>(buf.get());
    c_init_func(s);
    MapProtobufToCStruct(proto_msg, layout, buf.get(), allocs, type_desc);
    MapCStructToProtobuf(layout, buf.get(), proto_msg_norm, type_desc);
  }

  std::string proto_bytes_norm, proto_bytes_dest_dyn;
  ASSERT_TRUE(proto_msg_norm.SerializeToString(&proto_bytes_norm));
  ASSERT_TRUE(proto_msg_dest_dyn.SerializeToString(&proto_bytes_dest_dyn));
  EXPECT_EQ(proto_bytes_norm, proto_bytes_dest_dyn);

  // =========================================================================
  // PATH 2: Official C++ -> Official CDR -> C-struct -> Proto
  // =========================================================================

  // 1. Serialize official C++ message to CDR
  rclcpp::Serialization<ROSCppMsg> serializer;
  rclcpp::SerializedMessage official_serialized_msg;
  serializer.serialize_message(&cpp_msg, &official_serialized_msg);
  const auto& rcl_msg = official_serialized_msg.get_rcl_serialized_message();

  // 2. Deserialize official CDR -> C-struct
  auto c_struct_dest_off_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_dest_off_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct_dest_off =
      reinterpret_cast<ROSCStruct*>(c_struct_dest_off_buf.get());
  ASSERT_TRUE(c_init_func(c_struct_dest_off));

  ASSERT_EQ(rmw_deserialize(&rcl_msg, ts, c_struct_dest_off), RMW_RET_OK);

  // 3. Map C-struct -> Protobuf
  ProtoMsg proto_msg_dest_off;
  MapCStructToProtobuf(layout, c_struct_dest_off_buf.get(), proto_msg_dest_off,
                       type_desc);

  // 4. Compare Protobuf
  std::string proto_bytes_dest_off;
  ASSERT_TRUE(proto_msg_dest_off.SerializeToString(&proto_bytes_dest_off));
  EXPECT_EQ(proto_bytes_norm, proto_bytes_dest_off);

  // =========================================================================
  // Cleanup
  // =========================================================================
  c_fini_func(c_struct_dest_dyn);
  c_fini_func(c_struct_dest_off);
  ASSERT_EQ(rmw_serialized_message_fini(&serialized_msg), RMW_RET_OK);
}

// Declare extern C typesupport getters for Capture
extern "C" {
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Request();

const rosidl_runtime_c__type_description__TypeDescription*
aquarium__srv__Capture_Request__get_type_description(
    const rosidl_message_type_support_t*);

bool aquarium__srv__Capture_Request__init(aquarium__srv__Capture_Request*);
void aquarium__srv__Capture_Request__fini(aquarium__srv__Capture_Request*);

const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Response();

const rosidl_runtime_c__type_description__TypeDescription*
aquarium__srv__Capture_Response__get_type_description(
    const rosidl_message_type_support_t*);

bool aquarium__srv__Capture_Response__init(aquarium__srv__Capture_Response*);
void aquarium__srv__Capture_Response__fini(aquarium__srv__Capture_Response*);
}

class PerceptionSerializationTest : public ::testing::Test {};

TEST_F(PerceptionSerializationTest, CaptureRequest_Empty) {
  test_perception_messages::CaptureRequest proto_msg;
  // Leave everything empty

  aquarium::srv::Capture_Request cpp_msg;
  // Leave everything empty

  auto c_type_desc = aquarium__srv__Capture_Request__get_type_description(NULL);
  auto ts =
      rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Request();

  VerifySerialization<test_perception_messages::CaptureRequest,
                      aquarium::srv::Capture_Request,
                      aquarium__srv__Capture_Request>(
      proto_msg, cpp_msg, c_type_desc, ts, aquarium__srv__Capture_Request__init,
      aquarium__srv__Capture_Request__fini);
}

TEST_F(PerceptionSerializationTest, CaptureRequest_Full) {
  test_perception_messages::CaptureRequest proto_msg;

  // 1. CameraConfig
  auto* config = proto_msg.mutable_camera_config();
  config->mutable_identifier()->mutable_genicam()->set_device_id("camera_left");

  auto* setting = config->add_camera_settings();
  setting->set_name("ExposureAuto");
  setting->set_enumeration_value("Off");

  auto* setting2 = config->add_camera_settings();
  setting2->set_name("ExposureTime");
  setting2->set_float_value(10000.0);

  auto* setting3 = config->add_camera_settings();
  setting3->set_name("Gain");
  setting3->set_integer_value(12);

  auto* setting4 = config->add_camera_settings();
  setting4->set_name("ReverseX");
  setting4->set_bool_value(true);

  auto* sensor_config = config->add_sensor_configs();
  sensor_config->set_id(0);

  auto* pose = sensor_config->mutable_camera_t_sensor();
  pose->mutable_position()->set_x(0.1);
  pose->mutable_position()->set_y(0.2);
  pose->mutable_position()->set_z(0.3);
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);

  auto* params = sensor_config->mutable_camera_params();
  params->mutable_intrinsic_params()->set_focal_length_x(500.0);
  params->mutable_intrinsic_params()->set_focal_length_y(500.0);
  params->mutable_intrinsic_params()->set_principal_point_x(320.0);
  params->mutable_intrinsic_params()->set_principal_point_y(240.0);
  params->mutable_intrinsic_params()->mutable_dimensions()->set_cols(640);
  params->mutable_intrinsic_params()->mutable_dimensions()->set_rows(480);

  // 2. Timeout (Duration)
  proto_msg.mutable_timeout()->set_sec(5);
  proto_msg.mutable_timeout()->set_nanosec(123456789);

  // 3. Sensor IDs
  proto_msg.add_sensor_ids(0);
  proto_msg.add_sensor_ids(1);

  // 4. Post processing map
  auto& map = *proto_msg.mutable_post_processing_by_sensor_id();
  test_perception_messages::SensorImagePostProcessing proc;
  proc.mutable_crop()->set_origin_x(10);
  proc.mutable_crop()->set_origin_y(20);
  proc.mutable_crop()->mutable_dimensions()->set_cols(100);
  proc.mutable_crop()->mutable_dimensions()->set_rows(200);
  map[0] = proc;

  // 5. Capture result location (optional)
  proto_msg.mutable_capture_result_location()->set_store("my_store");
  proto_msg.mutable_capture_result_location()->set_key("my_key");

  // 6. Context (optional)
  proto_msg.mutable_context()->set_skill_id(123);
  proto_msg.mutable_context()->set_executive_plan_id(456);

  // Populate corresponding C++ message
  aquarium::srv::Capture_Request cpp_msg;

  // 1. CameraConfig
  cpp_msg.camera_config.identifier.genicam.device_id = "camera_left";

  aquarium::msg::CameraSetting cpp_setting;
  cpp_setting.name = "ExposureAuto";
  cpp_setting.enumeration_value = "Off";
  cpp_msg.camera_config.camera_settings.push_back(cpp_setting);

  aquarium::msg::CameraSetting cpp_setting2;
  cpp_setting2.name = "ExposureTime";
  cpp_setting2.float_value = 10000.0;
  cpp_msg.camera_config.camera_settings.push_back(cpp_setting2);

  aquarium::msg::CameraSetting cpp_setting3;
  cpp_setting3.name = "Gain";
  cpp_setting3.integer_value = 12;
  cpp_msg.camera_config.camera_settings.push_back(cpp_setting3);

  aquarium::msg::CameraSetting cpp_setting4;
  cpp_setting4.name = "ReverseX";
  cpp_setting4.bool_value = true;
  cpp_msg.camera_config.camera_settings.push_back(cpp_setting4);

  aquarium::msg::SensorConfig cpp_sensor_config;
  cpp_sensor_config.id = 0;

  geometry_msgs::msg::Pose cpp_pose;
  cpp_pose.position.x = 0.1;
  cpp_pose.position.y = 0.2;
  cpp_pose.position.z = 0.3;
  cpp_pose.orientation.x = 0.0;
  cpp_pose.orientation.y = 0.0;
  cpp_pose.orientation.z = 0.0;
  cpp_pose.orientation.w = 1.0;
  cpp_sensor_config.camera_t_sensor.push_back(cpp_pose);

  aquarium::msg::CameraParams cpp_params;
  cpp_params.intrinsic_params.focal_length_x = 500.0;
  cpp_params.intrinsic_params.focal_length_y = 500.0;
  cpp_params.intrinsic_params.principal_point_x = 320.0;
  cpp_params.intrinsic_params.principal_point_y = 240.0;
  cpp_params.intrinsic_params.dimensions.cols = 640;
  cpp_params.intrinsic_params.dimensions.rows = 480;
  cpp_sensor_config.camera_params.push_back(cpp_params);

  cpp_msg.camera_config.sensor_configs.push_back(cpp_sensor_config);

  // 2. Timeout
  cpp_msg.timeout.sec = 5;
  cpp_msg.timeout.nanosec = 123456789;

  // 3. Sensor IDs
  cpp_msg.sensor_ids.push_back(0);
  cpp_msg.sensor_ids.push_back(1);

  // 4. Post processing map
  aquarium::msg::SensorImagePostProcessingMap cpp_map_entry;
  cpp_map_entry.first = 0;

  aquarium::msg::CropOptions cpp_crop;
  cpp_crop.origin_x = 10;
  cpp_crop.origin_y = 20;
  cpp_crop.dimensions.cols = 100;
  cpp_crop.dimensions.rows = 200;
  cpp_map_entry.second.crop.push_back(cpp_crop);

  cpp_msg.post_processing_by_sensor_id.push_back(cpp_map_entry);

  // 5. Capture result location
  aquarium::msg::StorageLocation cpp_loc;
  cpp_loc.store = "my_store";
  cpp_loc.key = "my_key";
  cpp_msg.capture_result_location.push_back(cpp_loc);

  // 6. Context
  aquarium::msg::Context cpp_ctx;
  cpp_ctx.skill_id = 123;
  cpp_ctx.executive_plan_id = 456;
  cpp_msg.context.push_back(cpp_ctx);

  auto c_type_desc = aquarium__srv__Capture_Request__get_type_description(NULL);
  auto ts =
      rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Request();

  VerifySerialization<test_perception_messages::CaptureRequest,
                      aquarium::srv::Capture_Request,
                      aquarium__srv__Capture_Request>(
      proto_msg, cpp_msg, c_type_desc, ts, aquarium__srv__Capture_Request__init,
      aquarium__srv__Capture_Request__fini);
}

TEST_F(PerceptionSerializationTest, CaptureResponse_Empty) {
  test_perception_messages::CaptureResponse proto_msg;
  // Leave empty (oneof is not set)

  aquarium::srv::Capture_Response cpp_msg;

  auto c_type_desc =
      aquarium__srv__Capture_Response__get_type_description(NULL);
  auto ts =
      rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Response();

  VerifySerialization<test_perception_messages::CaptureResponse,
                      aquarium::srv::Capture_Response,
                      aquarium__srv__Capture_Response>(
      proto_msg, cpp_msg, c_type_desc, ts,
      aquarium__srv__Capture_Response__init,
      aquarium__srv__Capture_Response__fini);
}

TEST_F(PerceptionSerializationTest, CaptureResponse_Result) {
  test_perception_messages::CaptureResponse proto_msg;
  auto* result = proto_msg.mutable_capture_result();
  result->mutable_capture_at()->set_sec(1717171717);
  result->mutable_capture_at()->set_nanosec(123456);

  auto* img = result->add_sensor_images();
  img->mutable_acquisition_time()->set_sec(1717171717);
  img->mutable_acquisition_time()->set_nanosec(123456);
  img->mutable_buffer()->mutable_dimensions()->set_cols(2);
  img->mutable_buffer()->mutable_dimensions()->set_rows(2);
  img->mutable_buffer()->set_pixel_type(
      test_perception_messages::PixelType::PIXEL_INTENSITY);
  img->mutable_buffer()->set_type(
      test_perception_messages::DataType::TYPE_UINT8);
  uint8_t data[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  img->mutable_buffer()->set_data(data, sizeof(data));

  // Populate corresponding C++ message
  aquarium::srv::Capture_Response cpp_msg;
  cpp_msg.capture_result.capture_at.sec = 1717171717;
  cpp_msg.capture_result.capture_at.nanosec = 123456;

  aquarium::msg::SensorImage cpp_img;
  cpp_img.acquisition_time.sec = 1717171717;
  cpp_img.acquisition_time.nanosec = 123456;

  aquarium::msg::Dimensions cpp_dims;
  cpp_dims.cols = 2;
  cpp_dims.rows = 2;
  cpp_img.buffer.dimensions.push_back(cpp_dims);

  cpp_img.buffer.pixel_type = 1;  // PIXEL_INTENSITY
  cpp_img.buffer.type = 1;        // TYPE_UINT8
  cpp_img.buffer.data.assign(data, data + sizeof(data));
  cpp_msg.capture_result.sensor_images.push_back(cpp_img);

  auto c_type_desc =
      aquarium__srv__Capture_Response__get_type_description(NULL);
  auto ts =
      rosidl_typesupport_c__get_message_type_support_handle__aquarium__srv__Capture_Response();

  VerifySerialization<test_perception_messages::CaptureResponse,
                      aquarium::srv::Capture_Response,
                      aquarium__srv__Capture_Response>(
      proto_msg, cpp_msg, c_type_desc, ts,
      aquarium__srv__Capture_Response__init,
      aquarium__srv__Capture_Response__fini);
}
