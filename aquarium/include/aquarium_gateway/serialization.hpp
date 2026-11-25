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

#ifndef AQUARIUM_GATEWAY__SERIALIZATION_HPP_
#define AQUARIUM_GATEWAY__SERIALIZATION_HPP_

#include <google/protobuf/message.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/string.h"
#include "type_description_interfaces/msg/field_type.hpp"
#include "type_description_interfaces/msg/type_description.hpp"

namespace aquarium_gateway {

struct GenericPrimitiveSequence {
  void* data;
  size_t size;
  size_t capacity;
  bool is_rosidl_buffer;
  bool owns_rosidl_buffer;
};

struct GenericMessageSequence {
  void* data;
  size_t size;
  size_t capacity;
};

static_assert(sizeof(GenericPrimitiveSequence) ==
                  sizeof(rosidl_runtime_c__double__Sequence),
              "GenericPrimitiveSequence struct layout must match "
              "rosidl_runtime_c primitive sequences");

enum class ContainerKind {
  kSingular,
  kFixedArray,
  kBoundedSequence,
  kUnboundedSequence,
};

struct DecodedFieldType {
  uint8_t base_type_id = 0;
  ContainerKind container = ContainerKind::kSingular;

  bool is_collection() const { return container != ContainerKind::kSingular; }

  bool is_sequence() const {
    return container == ContainerKind::kBoundedSequence ||
           container == ContainerKind::kUnboundedSequence;
  }
};

struct DynamicField {
  std::string name;
  uint8_t type_id = 0;
  size_t offset = 0;
  size_t array_capacity = 0;
  std::string nested_type_name;
};

struct DynamicLayout {
  size_t total_size = 0;
  size_t max_alignment = 1;
  std::vector<DynamicField> fields;
};

DecodedFieldType DecodeFieldTypeId(uint8_t type_id);

std::string FormatFieldType(
    const type_description_interfaces::msg::FieldType& type);

uint8_t string_to_field_type_id(const std::string& type_str);

uint8_t parse_field_type(const std::string& type_str,
                         std::string& nested_type_name, uint64_t& capacity);

uint8_t parse_field_type(const std::string& type_str,
                         std::string& nested_type_name);

bool parse_ros_type(const std::string& ros_type, std::string& package,
                    std::string& base_name);

const type_description_interfaces::msg::IndividualTypeDescription*
FindIndividualType(
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const std::string& suffix);

DynamicLayout ComputeDynamicLayout(
    const type_description_interfaces::msg::IndividualTypeDescription&
        type_desc,
    const type_description_interfaces::msg::TypeDescription& parent_desc);

void MapCStructToProtobuf(
    const DynamicLayout& layout, const uint8_t* source_buf,
    google::protobuf::Message& dest_proto,
    const type_description_interfaces::msg::TypeDescription& parent_desc);

void MapProtobufToCStruct(
    const google::protobuf::Message& proto_msg, const DynamicLayout& layout,
    uint8_t* dest_buf, std::vector<std::unique_ptr<uint8_t[]>>& allocations,
    const type_description_interfaces::msg::TypeDescription& parent_desc);

}  // namespace aquarium_gateway

#endif  // AQUARIUM_GATEWAY__SERIALIZATION_HPP_
