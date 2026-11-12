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

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "aquarium_gateway/serialization.hpp"
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/detail/path__functions.h"
#include "nav_msgs/msg/detail/path__struct.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/allocator.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"
#include "sensor_msgs/msg/detail/image__functions.h"
#include "sensor_msgs/msg/detail/image__struct.h"
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "test_messages.pb.h"
#include "tf2_msgs/msg/detail/tf_message__functions.h"
#include "tf2_msgs/msg/detail/tf_message__struct.h"
#include "tf2_msgs/msg/tf_message.hpp"

extern "C" {
const rosidl_runtime_c__type_description__TypeDescription*
sensor_msgs__msg__Image__get_type_description(
    const rosidl_message_type_support_t*);
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image();

const rosidl_runtime_c__type_description__TypeDescription*
tf2_msgs__msg__TFMessage__get_type_description(
    const rosidl_message_type_support_t*);
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__msg__TFMessage();

const rosidl_runtime_c__type_description__TypeDescription*
sensor_msgs__msg__PointCloud2__get_type_description(
    const rosidl_message_type_support_t*);
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud2();

const rosidl_runtime_c__type_description__TypeDescription*
nav_msgs__msg__Path__get_type_description(const rosidl_message_type_support_t*);
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path();

const rosidl_runtime_c__type_description__TypeDescription*
geometry_msgs__msg__PoseWithCovariance__get_type_description(
    const rosidl_message_type_support_t*);
const rosidl_message_type_support_t*
rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseWithCovariance();
}

namespace aquarium_gateway {

std::string to_std_string(const rosidl_runtime_c__String& c_str) {
  if (!c_str.data) return "";
  return std::string(c_str.data, c_str.size);
}

type_description_interfaces::msg::IndividualTypeDescription to_cpp_individual(
    const rosidl_runtime_c__type_description__IndividualTypeDescription&
        c_ind) {
  type_description_interfaces::msg::IndividualTypeDescription cpp_ind;
  cpp_ind.type_name = to_std_string(c_ind.type_name);
  for (size_t i = 0; i < c_ind.fields.size; ++i) {
    const auto& c_field = c_ind.fields.data[i];
    type_description_interfaces::msg::Field cpp_field;
    cpp_field.name = to_std_string(c_field.name);
    cpp_field.type.type_id = c_field.type.type_id;
    cpp_field.type.capacity = c_field.type.capacity;
    cpp_field.type.string_capacity = c_field.type.string_capacity;
    cpp_field.type.nested_type_name =
        to_std_string(c_field.type.nested_type_name);
    cpp_field.default_value = to_std_string(c_field.default_value);
    cpp_ind.fields.push_back(cpp_field);
  }
  return cpp_ind;
}

type_description_interfaces::msg::TypeDescription to_cpp_type_desc(
    const rosidl_runtime_c__type_description__TypeDescription* c_desc) {
  type_description_interfaces::msg::TypeDescription cpp_desc;
  cpp_desc.type_description = to_cpp_individual(c_desc->type_description);
  for (size_t i = 0; i < c_desc->referenced_type_descriptions.size; ++i) {
    cpp_desc.referenced_type_descriptions.push_back(
        to_cpp_individual(c_desc->referenced_type_descriptions.data[i]));
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
  ASSERT_EQ(layout.total_size, sizeof(ROSCStruct));

  // Path 1: Proto -> Dynamic C-struct -> Dynamic CDR -> C-struct -> Proto
  std::vector<std::unique_ptr<uint8_t[]>> allocations;
  auto c_struct_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct = reinterpret_cast<ROSCStruct*>(c_struct_buf.get());

  MapProtobufToCStruct(proto_msg, layout, c_struct_buf.get(), allocations,
                       type_desc);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(rmw_serialized_message_init(&serialized_msg, 0, &allocator),
            RMW_RET_OK);
  ASSERT_EQ(rmw_serialize(c_struct, ts, &serialized_msg), RMW_RET_OK);

  auto c_struct_dest_dyn_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_dest_dyn_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct_dest_dyn =
      reinterpret_cast<ROSCStruct*>(c_struct_dest_dyn_buf.get());
  ASSERT_TRUE(c_init_func(c_struct_dest_dyn));
  ASSERT_EQ(rmw_deserialize(&serialized_msg, ts, c_struct_dest_dyn),
            RMW_RET_OK);

  ProtoMsg proto_msg_dest_dyn;
  MapCStructToProtobuf(layout, c_struct_dest_dyn_buf.get(), proto_msg_dest_dyn,
                       type_desc);

  std::string proto_bytes_orig;
  std::string proto_bytes_dest_dyn;
  ASSERT_TRUE(proto_msg.SerializeToString(&proto_bytes_orig));
  ASSERT_TRUE(proto_msg_dest_dyn.SerializeToString(&proto_bytes_dest_dyn));
  EXPECT_EQ(proto_bytes_orig, proto_bytes_dest_dyn);

  // Path 2: Official C++ -> Official CDR -> C-struct -> Proto
  rclcpp::Serialization<ROSCppMsg> serializer;
  rclcpp::SerializedMessage official_serialized_msg;
  serializer.serialize_message(&cpp_msg, &official_serialized_msg);
  const auto& rcl_msg = official_serialized_msg.get_rcl_serialized_message();

  auto c_struct_dest_off_buf = std::make_unique<uint8_t[]>(layout.total_size);
  std::memset(c_struct_dest_off_buf.get(), 0, layout.total_size);
  ROSCStruct* c_struct_dest_off =
      reinterpret_cast<ROSCStruct*>(c_struct_dest_off_buf.get());
  ASSERT_TRUE(c_init_func(c_struct_dest_off));
  ASSERT_EQ(rmw_deserialize(&rcl_msg, ts, c_struct_dest_off), RMW_RET_OK);

  ProtoMsg proto_msg_dest_off;
  MapCStructToProtobuf(layout, c_struct_dest_off_buf.get(), proto_msg_dest_off,
                       type_desc);

  std::string proto_bytes_dest_off;
  ASSERT_TRUE(proto_msg_dest_off.SerializeToString(&proto_bytes_dest_off));
  EXPECT_EQ(proto_bytes_orig, proto_bytes_dest_off);

  c_fini_func(c_struct_dest_dyn);
  c_fini_func(c_struct_dest_off);
  ASSERT_EQ(rmw_serialized_message_fini(&serialized_msg), RMW_RET_OK);
}

class SerializationTest : public ::testing::Test {};

TEST_F(SerializationTest, EmptyImage) {
  test_messages::Image proto_msg;
  proto_msg.mutable_header()->mutable_stamp()->set_sec(123);
  proto_msg.mutable_header()->mutable_stamp()->set_nanosec(456);
  proto_msg.mutable_header()->set_frame_id("camera");
  proto_msg.set_height(0);
  proto_msg.set_width(0);
  proto_msg.set_encoding("rgb8");
  proto_msg.set_is_bigendian(0);
  proto_msg.set_step(0);
  proto_msg.set_data("");

  sensor_msgs::msg::Image cpp_msg;
  cpp_msg.header.stamp.sec = 123;
  cpp_msg.header.stamp.nanosec = 456;
  cpp_msg.header.frame_id = "camera";
  cpp_msg.height = 0;
  cpp_msg.width = 0;
  cpp_msg.encoding = "rgb8";
  cpp_msg.is_bigendian = 0;
  cpp_msg.step = 0;

  auto* c_type_desc = sensor_msgs__msg__Image__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image();

  VerifySerialization<test_messages::Image, sensor_msgs::msg::Image,
                      sensor_msgs__msg__Image>(
      proto_msg, cpp_msg, c_type_desc, ts, sensor_msgs__msg__Image__init,
      sensor_msgs__msg__Image__fini);
}

TEST_F(SerializationTest, SmallImage) {
  test_messages::Image proto_msg;
  proto_msg.mutable_header()->mutable_stamp()->set_sec(999);
  proto_msg.mutable_header()->mutable_stamp()->set_nanosec(888);
  proto_msg.mutable_header()->set_frame_id("opaque_frame");
  proto_msg.set_height(2);
  proto_msg.set_width(2);
  proto_msg.set_encoding("rgba8");
  proto_msg.set_is_bigendian(1);
  proto_msg.set_step(8);
  uint8_t data[] = {10, 20,  30,  40,  50,  60,  70,  80,
                    90, 100, 110, 120, 130, 140, 150, 160};
  proto_msg.set_data(data, sizeof(data));

  sensor_msgs::msg::Image cpp_msg;
  cpp_msg.header.stamp.sec = 999;
  cpp_msg.header.stamp.nanosec = 888;
  cpp_msg.header.frame_id = "opaque_frame";
  cpp_msg.height = 2;
  cpp_msg.width = 2;
  cpp_msg.encoding = "rgba8";
  cpp_msg.is_bigendian = 1;
  cpp_msg.step = 8;
  cpp_msg.data.assign(data, data + sizeof(data));

  auto* c_type_desc = sensor_msgs__msg__Image__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image();

  VerifySerialization<test_messages::Image, sensor_msgs::msg::Image,
                      sensor_msgs__msg__Image>(
      proto_msg, cpp_msg, c_type_desc, ts, sensor_msgs__msg__Image__init,
      sensor_msgs__msg__Image__fini);
}

TEST_F(SerializationTest, EmptyTFMessage) {
  test_messages::TFMessage proto_msg;
  tf2_msgs::msg::TFMessage cpp_msg;

  auto* c_type_desc = tf2_msgs__msg__TFMessage__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__msg__TFMessage();

  VerifySerialization<test_messages::TFMessage, tf2_msgs::msg::TFMessage,
                      tf2_msgs__msg__TFMessage>(
      proto_msg, cpp_msg, c_type_desc, ts, tf2_msgs__msg__TFMessage__init,
      tf2_msgs__msg__TFMessage__fini);
}

TEST_F(SerializationTest, ComplexTFMessage) {
  test_messages::TFMessage proto_msg;
  auto* t1 = proto_msg.add_transforms();
  t1->mutable_header()->mutable_stamp()->set_sec(1);
  t1->mutable_header()->mutable_stamp()->set_nanosec(2);
  t1->mutable_header()->set_frame_id("world");
  t1->set_child_frame_id("robot");
  t1->mutable_transform()->mutable_translation()->set_x(1.5);
  t1->mutable_transform()->mutable_translation()->set_y(-2.0);
  t1->mutable_transform()->mutable_translation()->set_z(0.5);
  t1->mutable_transform()->mutable_rotation()->set_x(0.0);
  t1->mutable_transform()->mutable_rotation()->set_y(0.0);
  t1->mutable_transform()->mutable_rotation()->set_z(0.707);
  t1->mutable_transform()->mutable_rotation()->set_w(0.707);

  auto* t2 = proto_msg.add_transforms();
  t2->mutable_header()->mutable_stamp()->set_sec(3);
  t2->mutable_header()->mutable_stamp()->set_nanosec(4);
  t2->mutable_header()->set_frame_id("robot");
  t2->set_child_frame_id("camera");
  t2->mutable_transform()->mutable_translation()->set_x(0.1);
  t2->mutable_transform()->mutable_translation()->set_y(0.0);
  t2->mutable_transform()->mutable_translation()->set_z(0.8);
  t2->mutable_transform()->mutable_rotation()->set_x(0.5);
  t2->mutable_transform()->mutable_rotation()->set_y(-0.5);
  t2->mutable_transform()->mutable_rotation()->set_z(0.5);
  t2->mutable_transform()->mutable_rotation()->set_w(-0.5);

  tf2_msgs::msg::TFMessage cpp_msg;
  geometry_msgs::msg::TransformStamped cpp_t1;
  cpp_t1.header.stamp.sec = 1;
  cpp_t1.header.stamp.nanosec = 2;
  cpp_t1.header.frame_id = "world";
  cpp_t1.child_frame_id = "robot";
  cpp_t1.transform.translation.x = 1.5;
  cpp_t1.transform.translation.y = -2.0;
  cpp_t1.transform.translation.z = 0.5;
  cpp_t1.transform.rotation.x = 0.0;
  cpp_t1.transform.rotation.y = 0.0;
  cpp_t1.transform.rotation.z = 0.707;
  cpp_t1.transform.rotation.w = 0.707;
  cpp_msg.transforms.push_back(cpp_t1);

  geometry_msgs::msg::TransformStamped cpp_t2;
  cpp_t2.header.stamp.sec = 3;
  cpp_t2.header.stamp.nanosec = 4;
  cpp_t2.header.frame_id = "robot";
  cpp_t2.child_frame_id = "camera";
  cpp_t2.transform.translation.x = 0.1;
  cpp_t2.transform.translation.y = 0.0;
  cpp_t2.transform.translation.z = 0.8;
  cpp_t2.transform.rotation.x = 0.5;
  cpp_t2.transform.rotation.y = -0.5;
  cpp_t2.transform.rotation.z = 0.5;
  cpp_t2.transform.rotation.w = -0.5;
  cpp_msg.transforms.push_back(cpp_t2);

  auto* c_type_desc = tf2_msgs__msg__TFMessage__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__msg__TFMessage();

  VerifySerialization<test_messages::TFMessage, tf2_msgs::msg::TFMessage,
                      tf2_msgs__msg__TFMessage>(
      proto_msg, cpp_msg, c_type_desc, ts, tf2_msgs__msg__TFMessage__init,
      tf2_msgs__msg__TFMessage__fini);
}

TEST_F(SerializationTest, PointCloud2) {
  test_messages::PointCloud2 proto_msg;
  proto_msg.mutable_header()->mutable_stamp()->set_sec(42);
  proto_msg.mutable_header()->mutable_stamp()->set_nanosec(43);
  proto_msg.mutable_header()->set_frame_id("lidar");
  proto_msg.set_height(1);
  proto_msg.set_width(3);
  proto_msg.set_is_bigendian(false);
  proto_msg.set_point_step(16);
  proto_msg.set_row_step(48);
  proto_msg.set_is_dense(true);

  const char* names[] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    auto* f = proto_msg.add_fields();
    f->set_name(names[i]);
    f->set_offset(i * 4);
    f->set_datatype(7);
    f->set_count(1);
  }

  float points_data[] = {1.0f, 2.0f, 3.0f, 0.5f, 4.0f, 5.0f,
                         6.0f, 1.0f, 7.0f, 8.0f, 9.0f, 0.0f};
  proto_msg.set_data(reinterpret_cast<const char*>(points_data),
                     sizeof(points_data));

  sensor_msgs::msg::PointCloud2 cpp_msg;
  cpp_msg.header.stamp.sec = 42;
  cpp_msg.header.stamp.nanosec = 43;
  cpp_msg.header.frame_id = "lidar";
  cpp_msg.height = 1;
  cpp_msg.width = 3;
  cpp_msg.is_bigendian = false;
  cpp_msg.point_step = 16;
  cpp_msg.row_step = 48;
  cpp_msg.is_dense = true;

  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField pf;
    pf.name = names[i];
    pf.offset = i * 4;
    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pf.count = 1;
    cpp_msg.fields.push_back(pf);
  }

  cpp_msg.data.resize(sizeof(points_data));
  std::memcpy(cpp_msg.data.data(), points_data, sizeof(points_data));

  auto* c_type_desc =
      sensor_msgs__msg__PointCloud2__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud2();

  VerifySerialization<test_messages::PointCloud2, sensor_msgs::msg::PointCloud2,
                      sensor_msgs__msg__PointCloud2>(
      proto_msg, cpp_msg, c_type_desc, ts, sensor_msgs__msg__PointCloud2__init,
      sensor_msgs__msg__PointCloud2__fini);
}

TEST_F(SerializationTest, Path) {
  test_messages::Path proto_msg;
  proto_msg.mutable_header()->mutable_stamp()->set_sec(10);
  proto_msg.mutable_header()->mutable_stamp()->set_nanosec(20);
  proto_msg.mutable_header()->set_frame_id("map");

  auto* p1 = proto_msg.add_poses();
  p1->mutable_header()->mutable_stamp()->set_sec(11);
  p1->mutable_header()->mutable_stamp()->set_nanosec(21);
  p1->mutable_header()->set_frame_id("map");
  p1->mutable_pose()->mutable_position()->set_x(1.0);
  p1->mutable_pose()->mutable_position()->set_y(2.0);
  p1->mutable_pose()->mutable_position()->set_z(0.0);
  p1->mutable_pose()->mutable_orientation()->set_w(1.0);

  auto* p2 = proto_msg.add_poses();
  p2->mutable_header()->mutable_stamp()->set_sec(12);
  p2->mutable_header()->mutable_stamp()->set_nanosec(22);
  p2->mutable_header()->set_frame_id("map");
  p2->mutable_pose()->mutable_position()->set_x(2.0);
  p2->mutable_pose()->mutable_position()->set_y(3.0);
  p2->mutable_pose()->mutable_position()->set_z(0.0);
  p2->mutable_pose()->mutable_orientation()->set_z(0.707);
  p2->mutable_pose()->mutable_orientation()->set_w(0.707);

  nav_msgs::msg::Path cpp_msg;
  cpp_msg.header.stamp.sec = 10;
  cpp_msg.header.stamp.nanosec = 20;
  cpp_msg.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped cpp_p1;
  cpp_p1.header.stamp.sec = 11;
  cpp_p1.header.stamp.nanosec = 21;
  cpp_p1.header.frame_id = "map";
  cpp_p1.pose.position.x = 1.0;
  cpp_p1.pose.position.y = 2.0;
  cpp_p1.pose.orientation.w = 1.0;
  cpp_msg.poses.push_back(cpp_p1);

  geometry_msgs::msg::PoseStamped cpp_p2;
  cpp_p2.header.stamp.sec = 12;
  cpp_p2.header.stamp.nanosec = 22;
  cpp_p2.header.frame_id = "map";
  cpp_p2.pose.position.x = 2.0;
  cpp_p2.pose.position.y = 3.0;
  cpp_p2.pose.orientation.z = 0.707;
  cpp_p2.pose.orientation.w = 0.707;
  cpp_msg.poses.push_back(cpp_p2);

  auto* c_type_desc = nav_msgs__msg__Path__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path();

  VerifySerialization<test_messages::Path, nav_msgs::msg::Path,
                      nav_msgs__msg__Path>(proto_msg, cpp_msg, c_type_desc, ts,
                                           nav_msgs__msg__Path__init,
                                           nav_msgs__msg__Path__fini);
}

TEST_F(SerializationTest, PoseWithCovarianceFixedArray) {
  test_messages::PoseWithCovariance proto_msg;
  proto_msg.mutable_pose()->mutable_position()->set_x(1.25);
  proto_msg.mutable_pose()->mutable_position()->set_y(-2.5);
  proto_msg.mutable_pose()->mutable_position()->set_z(0.75);
  proto_msg.mutable_pose()->mutable_orientation()->set_x(0.0);
  proto_msg.mutable_pose()->mutable_orientation()->set_y(0.0);
  proto_msg.mutable_pose()->mutable_orientation()->set_z(0.0);
  proto_msg.mutable_pose()->mutable_orientation()->set_w(1.0);

  geometry_msgs::msg::PoseWithCovariance cpp_msg;
  cpp_msg.pose.position.x = 1.25;
  cpp_msg.pose.position.y = -2.5;
  cpp_msg.pose.position.z = 0.75;
  cpp_msg.pose.orientation.x = 0.0;
  cpp_msg.pose.orientation.y = 0.0;
  cpp_msg.pose.orientation.z = 0.0;
  cpp_msg.pose.orientation.w = 1.0;

  for (size_t i = 0; i < 36; ++i) {
    const double v = static_cast<double>(i) * 0.125 + 1.0;
    proto_msg.add_covariance(v);
    cpp_msg.covariance[i] = v;
  }

  auto* c_type_desc =
      geometry_msgs__msg__PoseWithCovariance__get_type_description(nullptr);
  auto* ts =
      rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseWithCovariance();

  VerifySerialization<test_messages::PoseWithCovariance,
                      geometry_msgs::msg::PoseWithCovariance,
                      geometry_msgs__msg__PoseWithCovariance>(
      proto_msg, cpp_msg, c_type_desc, ts,
      geometry_msgs__msg__PoseWithCovariance__init,
      geometry_msgs__msg__PoseWithCovariance__fini);
}

TEST_F(SerializationTest, StringToFieldTypeIdAliases) {
  using FieldType = type_description_interfaces::msg::FieldType;
  EXPECT_EQ(string_to_field_type_id("float"), FieldType::FIELD_TYPE_FLOAT);
  EXPECT_EQ(string_to_field_type_id("float32"), FieldType::FIELD_TYPE_FLOAT);
  EXPECT_EQ(string_to_field_type_id("double"), FieldType::FIELD_TYPE_DOUBLE);
  EXPECT_EQ(string_to_field_type_id("float64"), FieldType::FIELD_TYPE_DOUBLE);
  EXPECT_EQ(string_to_field_type_id("bool"), FieldType::FIELD_TYPE_BOOLEAN);
  EXPECT_EQ(string_to_field_type_id("boolean"), FieldType::FIELD_TYPE_BOOLEAN);
}

TEST_F(SerializationTest, ParseAndFormatFixedArrayFieldTypes) {
  using FieldType = type_description_interfaces::msg::FieldType;

  std::string nested;
  uint64_t capacity = 0;

  uint8_t tid = parse_field_type("double[36]", nested, capacity);
  EXPECT_EQ(tid, FieldType::FIELD_TYPE_DOUBLE_ARRAY);
  EXPECT_EQ(capacity, 36u);
  EXPECT_TRUE(nested.empty());

  FieldType ft;
  ft.type_id = tid;
  ft.capacity = capacity;
  EXPECT_EQ(FormatFieldType(ft), "double[36]");

  tid = parse_field_type("geometry_msgs/msg/Point[4]", nested, capacity);
  EXPECT_EQ(tid, FieldType::FIELD_TYPE_NESTED_TYPE_ARRAY);
  EXPECT_EQ(capacity, 4u);
  EXPECT_EQ(nested, "geometry_msgs/msg/Point");

  ft.type_id = tid;
  ft.capacity = capacity;
  ft.nested_type_name = nested;
  EXPECT_EQ(FormatFieldType(ft), "geometry_msgs/msg/Point[4]");

  tid = parse_field_type("int32[<=5]", nested, capacity);
  EXPECT_EQ(tid, FieldType::FIELD_TYPE_INT32_BOUNDED_SEQUENCE);
  EXPECT_EQ(capacity, 5u);
}

}  // namespace aquarium_gateway

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
