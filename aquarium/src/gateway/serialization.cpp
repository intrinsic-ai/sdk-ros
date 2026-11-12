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

#include "aquarium_gateway/serialization.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <type_traits>

namespace aquarium_gateway {
namespace {

using FieldType = type_description_interfaces::msg::FieldType;

bool IsStringTypeId(uint8_t base_type_id) {
  return base_type_id == FieldType::FIELD_TYPE_STRING ||
         base_type_id == FieldType::FIELD_TYPE_BOUNDED_STRING;
}

int64_t GetProtoInt64(const google::protobuf::Message& msg,
                      const google::protobuf::Reflection* reflection,
                      const google::protobuf::FieldDescriptor* pf) {
  switch (pf->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return reflection->GetInt32(msg, pf);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return reflection->GetInt64(msg, pf);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return reflection->GetUInt32(msg, pf);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return static_cast<int64_t>(reflection->GetUInt64(msg, pf));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      return reflection->GetEnum(msg, pf)->number();
    default:
      return 0;
  }
}

uint64_t GetProtoUInt64(const google::protobuf::Message& msg,
                        const google::protobuf::Reflection* reflection,
                        const google::protobuf::FieldDescriptor* pf) {
  if (pf->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT64) {
    return reflection->GetUInt64(msg, pf);
  }
  return static_cast<uint64_t>(GetProtoInt64(msg, reflection, pf));
}

int64_t GetProtoRepeatedInt64(const google::protobuf::Message& msg,
                              const google::protobuf::Reflection* reflection,
                              const google::protobuf::FieldDescriptor* pf,
                              int index) {
  switch (pf->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return reflection->GetRepeatedInt32(msg, pf, index);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return reflection->GetRepeatedInt64(msg, pf, index);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return reflection->GetRepeatedUInt32(msg, pf, index);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return static_cast<int64_t>(
          reflection->GetRepeatedUInt64(msg, pf, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      return reflection->GetRepeatedEnum(msg, pf, index)->number();
    default:
      return 0;
  }
}

uint64_t GetProtoRepeatedUInt64(const google::protobuf::Message& msg,
                                const google::protobuf::Reflection* reflection,
                                const google::protobuf::FieldDescriptor* pf,
                                int index) {
  if (pf->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT64) {
    return reflection->GetRepeatedUInt64(msg, pf, index);
  }
  return static_cast<uint64_t>(
      GetProtoRepeatedInt64(msg, reflection, pf, index));
}

void SetProtoInt64(google::protobuf::Message* msg,
                   const google::protobuf::Reflection* reflection,
                   const google::protobuf::FieldDescriptor* pf, int64_t val) {
  switch (pf->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->SetInt32(msg, pf, static_cast<int32_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->SetInt64(msg, pf, val);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->SetUInt32(msg, pf, static_cast<uint32_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->SetUInt64(msg, pf, static_cast<uint64_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const auto* enum_val =
          pf->enum_type()->FindValueByNumber(static_cast<int>(val));
      if (enum_val != nullptr) {
        reflection->SetEnum(msg, pf, enum_val);
      }
      break;
    }
    default:
      break;
  }
}

void AddProtoInt64(google::protobuf::Message* msg,
                   const google::protobuf::Reflection* reflection,
                   const google::protobuf::FieldDescriptor* pf, int64_t val) {
  switch (pf->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      reflection->AddInt32(msg, pf, static_cast<int32_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      reflection->AddInt64(msg, pf, val);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      reflection->AddUInt32(msg, pf, static_cast<uint32_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      reflection->AddUInt64(msg, pf, static_cast<uint64_t>(val));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const auto* enum_val =
          pf->enum_type()->FindValueByNumber(static_cast<int>(val));
      if (enum_val != nullptr) {
        reflection->AddEnum(msg, pf, enum_val);
      }
      break;
    }
    default:
      break;
  }
}

void SetProtoUInt64(google::protobuf::Message* msg,
                    const google::protobuf::Reflection* reflection,
                    const google::protobuf::FieldDescriptor* pf, uint64_t val) {
  if (pf->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT64) {
    reflection->SetUInt64(msg, pf, val);
  } else {
    SetProtoInt64(msg, reflection, pf, static_cast<int64_t>(val));
  }
}

void AddProtoUInt64(google::protobuf::Message* msg,
                    const google::protobuf::Reflection* reflection,
                    const google::protobuf::FieldDescriptor* pf, uint64_t val) {
  if (pf->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT64) {
    reflection->AddUInt64(msg, pf, val);
  } else {
    AddProtoInt64(msg, reflection, pf, static_cast<int64_t>(val));
  }
}

const google::protobuf::FieldDescriptor* ResolveProtoField(
    const google::protobuf::Descriptor* desc, const std::string& ros_name) {
  if (desc->options().map_entry()) {
    if (ros_name == "first") {
      return desc->FindFieldByName("key");
    }
    if (ros_name == "second") {
      return desc->FindFieldByName("value");
    }
    return nullptr;
  }
  if (desc->full_name() == "google.protobuf.Duration" ||
      desc->full_name() == "google.protobuf.Timestamp") {
    if (ros_name == "sec") {
      return desc->FindFieldByName("seconds");
    }
    if (ros_name == "nanosec") {
      return desc->FindFieldByName("nanos");
    }
    return nullptr;
  }
  return desc->FindFieldByName(ros_name);
}

bool GetBaseElementSizeAndAlignment(
    uint8_t base_type_id, const std::string& nested_type_name,
    const type_description_interfaces::msg::TypeDescription& parent_desc,
    size_t& elem_size, size_t& elem_align) {
  switch (base_type_id) {
    case FieldType::FIELD_TYPE_INT8:
    case FieldType::FIELD_TYPE_UINT8:
    case FieldType::FIELD_TYPE_BOOLEAN:
    case FieldType::FIELD_TYPE_BYTE:
    case FieldType::FIELD_TYPE_CHAR:
      elem_size = 1;
      elem_align = 1;
      return true;
    case FieldType::FIELD_TYPE_INT16:
    case FieldType::FIELD_TYPE_UINT16:
      elem_size = 2;
      elem_align = 2;
      return true;
    case FieldType::FIELD_TYPE_INT32:
    case FieldType::FIELD_TYPE_UINT32:
    case FieldType::FIELD_TYPE_FLOAT:
      elem_size = 4;
      elem_align = 4;
      return true;
    case FieldType::FIELD_TYPE_INT64:
    case FieldType::FIELD_TYPE_UINT64:
    case FieldType::FIELD_TYPE_DOUBLE:
      elem_size = 8;
      elem_align = 8;
      return true;
    case FieldType::FIELD_TYPE_STRING:
    case FieldType::FIELD_TYPE_BOUNDED_STRING:
      elem_size = sizeof(rosidl_runtime_c__String);
      elem_align = alignof(rosidl_runtime_c__String);
      return true;
    case FieldType::FIELD_TYPE_NESTED_TYPE: {
      const auto* nested_desc =
          FindIndividualType(parent_desc, nested_type_name);
      if (nested_desc != nullptr) {
        auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
        elem_size = nested_layout.total_size;
        elem_align = nested_layout.max_alignment;
        return true;
      }
      std::cerr << "GATEWAY ERROR: Nested type not found in parent descriptor: "
                << nested_type_name << std::endl;
      elem_size = 8;
      elem_align = 8;
      return false;
    }
    default:
      elem_size = 8;
      elem_align = 8;
      return false;
  }
}

void WriteRosStringAt(uint8_t* dst, const std::string& val,
                      std::vector<std::unique_ptr<uint8_t[]>>& allocations) {
  rosidl_runtime_c__String ros_str;
  ros_str.size = val.size();
  ros_str.capacity = val.size() + 1;
  auto str_data = std::make_unique<uint8_t[]>(ros_str.capacity);
  std::memcpy(str_data.get(), val.c_str(), ros_str.capacity);
  ros_str.data = reinterpret_cast<char*>(str_data.get());
  std::memcpy(dst, &ros_str, sizeof(rosidl_runtime_c__String));
  allocations.push_back(std::move(str_data));
}

void InitEmptyCStructField(
    const DynamicField& f, uint8_t* dest_buf,
    std::vector<std::unique_ptr<uint8_t[]>>& allocations,
    const type_description_interfaces::msg::TypeDescription& parent_desc) {
  const DecodedFieldType decoded = DecodeFieldTypeId(f.type_id);
  if (decoded.container == ContainerKind::kSingular) {
    if (IsStringTypeId(decoded.base_type_id)) {
      WriteRosStringAt(dest_buf + f.offset, "", allocations);
    } else if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
      const auto* nested_desc =
          FindIndividualType(parent_desc, f.nested_type_name);
      if (nested_desc != nullptr) {
        auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
        for (const auto& nf : nested_layout.fields) {
          InitEmptyCStructField(nf, dest_buf + f.offset, allocations,
                                parent_desc);
        }
      }
    }
  } else if (decoded.container == ContainerKind::kFixedArray &&
             f.array_capacity > 0) {
    if (IsStringTypeId(decoded.base_type_id)) {
      for (size_t i = 0; i < f.array_capacity; ++i) {
        WriteRosStringAt(
            dest_buf + f.offset + i * sizeof(rosidl_runtime_c__String), "",
            allocations);
      }
    } else if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
      const auto* nested_desc =
          FindIndividualType(parent_desc, f.nested_type_name);
      if (nested_desc != nullptr) {
        auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
        for (size_t i = 0; i < f.array_capacity; ++i) {
          uint8_t* elem_ptr =
              dest_buf + f.offset + i * nested_layout.total_size;
          for (const auto& nf : nested_layout.fields) {
            InitEmptyCStructField(nf, elem_ptr, allocations, parent_desc);
          }
        }
      }
    }
  }
}

template <typename T>
void CopyPrimitivesToProto(const void* raw_data, size_t count,
                           google::protobuf::Message* dest_proto,
                           const google::protobuf::Reflection* reflection,
                           const google::protobuf::FieldDescriptor* pf) {
  if (raw_data == nullptr || count == 0) {
    return;
  }
  const size_t limit = pf->is_repeated() ? count : 1;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(raw_data);
  for (size_t i = 0; i < limit; ++i) {
    T v{};
    std::memcpy(&v, bytes + i * sizeof(T), sizeof(T));
    if constexpr (std::is_same_v<T, float>) {
      if (pf->is_repeated()) {
        reflection->AddFloat(dest_proto, pf, v);
      } else {
        reflection->SetFloat(dest_proto, pf, v);
      }
    } else if constexpr (std::is_same_v<T, double>) {
      if (pf->is_repeated()) {
        reflection->AddDouble(dest_proto, pf, v);
      } else {
        reflection->SetDouble(dest_proto, pf, v);
      }
    } else if constexpr (std::is_same_v<T, bool>) {
      if (pf->is_repeated()) {
        reflection->AddBool(dest_proto, pf, v);
      } else {
        reflection->SetBool(dest_proto, pf, v);
      }
    } else if constexpr (std::is_same_v<T, uint64_t>) {
      if (pf->is_repeated()) {
        AddProtoUInt64(dest_proto, reflection, pf, v);
      } else {
        SetProtoUInt64(dest_proto, reflection, pf, v);
      }
    } else {
      if (pf->is_repeated()) {
        AddProtoInt64(dest_proto, reflection, pf, static_cast<int64_t>(v));
      } else {
        SetProtoInt64(dest_proto, reflection, pf, static_cast<int64_t>(v));
      }
    }
  }
}

void DispatchPrimitivesToProto(uint8_t base_type_id, const void* raw_data,
                               size_t count,
                               google::protobuf::Message* dest_proto,
                               const google::protobuf::Reflection* reflection,
                               const google::protobuf::FieldDescriptor* pf) {
  switch (base_type_id) {
    case FieldType::FIELD_TYPE_INT8:
      CopyPrimitivesToProto<int8_t>(raw_data, count, dest_proto, reflection,
                                    pf);
      break;
    case FieldType::FIELD_TYPE_UINT8:
    case FieldType::FIELD_TYPE_BYTE:
    case FieldType::FIELD_TYPE_CHAR:
      CopyPrimitivesToProto<uint8_t>(raw_data, count, dest_proto, reflection,
                                     pf);
      break;
    case FieldType::FIELD_TYPE_INT16:
      CopyPrimitivesToProto<int16_t>(raw_data, count, dest_proto, reflection,
                                     pf);
      break;
    case FieldType::FIELD_TYPE_UINT16:
      CopyPrimitivesToProto<uint16_t>(raw_data, count, dest_proto, reflection,
                                      pf);
      break;
    case FieldType::FIELD_TYPE_INT32:
      CopyPrimitivesToProto<int32_t>(raw_data, count, dest_proto, reflection,
                                     pf);
      break;
    case FieldType::FIELD_TYPE_UINT32:
      CopyPrimitivesToProto<uint32_t>(raw_data, count, dest_proto, reflection,
                                      pf);
      break;
    case FieldType::FIELD_TYPE_INT64:
      CopyPrimitivesToProto<int64_t>(raw_data, count, dest_proto, reflection,
                                     pf);
      break;
    case FieldType::FIELD_TYPE_UINT64:
      CopyPrimitivesToProto<uint64_t>(raw_data, count, dest_proto, reflection,
                                      pf);
      break;
    case FieldType::FIELD_TYPE_FLOAT:
      CopyPrimitivesToProto<float>(raw_data, count, dest_proto, reflection, pf);
      break;
    case FieldType::FIELD_TYPE_DOUBLE:
      CopyPrimitivesToProto<double>(raw_data, count, dest_proto, reflection,
                                    pf);
      break;
    case FieldType::FIELD_TYPE_BOOLEAN:
      CopyPrimitivesToProto<bool>(raw_data, count, dest_proto, reflection, pf);
      break;
    default:
      break;
  }
}

template <typename T>
void CopyProtoToPrimitives(const google::protobuf::Message& proto_msg,
                           const google::protobuf::Reflection* reflection,
                           const google::protobuf::FieldDescriptor* pf,
                           size_t count, uint8_t* dst) {
  for (size_t i = 0; i < count; ++i) {
    T v{};
    const int idx = static_cast<int>(i);
    if constexpr (std::is_same_v<T, float>) {
      v = pf->is_repeated() ? reflection->GetRepeatedFloat(proto_msg, pf, idx)
                            : reflection->GetFloat(proto_msg, pf);
    } else if constexpr (std::is_same_v<T, double>) {
      v = pf->is_repeated() ? reflection->GetRepeatedDouble(proto_msg, pf, idx)
                            : reflection->GetDouble(proto_msg, pf);
    } else if constexpr (std::is_same_v<T, bool>) {
      v = pf->is_repeated() ? reflection->GetRepeatedBool(proto_msg, pf, idx)
                            : reflection->GetBool(proto_msg, pf);
    } else if constexpr (std::is_same_v<T, uint64_t>) {
      v = pf->is_repeated()
              ? GetProtoRepeatedUInt64(proto_msg, reflection, pf, idx)
              : GetProtoUInt64(proto_msg, reflection, pf);
    } else {
      v = static_cast<T>(
          pf->is_repeated()
              ? GetProtoRepeatedInt64(proto_msg, reflection, pf, idx)
              : GetProtoInt64(proto_msg, reflection, pf));
    }
    std::memcpy(dst + i * sizeof(T), &v, sizeof(T));
  }
}

void DispatchProtoToPrimitives(uint8_t base_type_id,
                               const google::protobuf::Message& proto_msg,
                               const google::protobuf::Reflection* reflection,
                               const google::protobuf::FieldDescriptor* pf,
                               size_t count, uint8_t* dst) {
  switch (base_type_id) {
    case FieldType::FIELD_TYPE_INT8:
      CopyProtoToPrimitives<int8_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_UINT8:
    case FieldType::FIELD_TYPE_BYTE:
    case FieldType::FIELD_TYPE_CHAR:
      CopyProtoToPrimitives<uint8_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_INT16:
      CopyProtoToPrimitives<int16_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_UINT16:
      CopyProtoToPrimitives<uint16_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_INT32:
      CopyProtoToPrimitives<int32_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_UINT32:
      CopyProtoToPrimitives<uint32_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_INT64:
      CopyProtoToPrimitives<int64_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_UINT64:
      CopyProtoToPrimitives<uint64_t>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_FLOAT:
      CopyProtoToPrimitives<float>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_DOUBLE:
      CopyProtoToPrimitives<double>(proto_msg, reflection, pf, count, dst);
      break;
    case FieldType::FIELD_TYPE_BOOLEAN:
      CopyProtoToPrimitives<bool>(proto_msg, reflection, pf, count, dst);
      break;
    default:
      break;
  }
}

}  // namespace

DecodedFieldType DecodeFieldTypeId(uint8_t type_id) {
  DecodedFieldType out;
  if (type_id >= 49 && type_id <= 70) {
    out.base_type_id = static_cast<uint8_t>(type_id - 48);
    out.container = ContainerKind::kFixedArray;
  } else if (type_id >= 97 && type_id <= 118) {
    out.base_type_id = static_cast<uint8_t>(type_id - 96);
    out.container = ContainerKind::kBoundedSequence;
  } else if (type_id >= 145 && type_id <= 166) {
    out.base_type_id = static_cast<uint8_t>(type_id - 144);
    out.container = ContainerKind::kUnboundedSequence;
  } else {
    out.base_type_id = type_id;
    out.container = ContainerKind::kSingular;
  }
  return out;
}

std::string FormatFieldType(
    const type_description_interfaces::msg::FieldType& type) {
  const DecodedFieldType decoded = DecodeFieldTypeId(type.type_id);

  std::string suffix;
  switch (decoded.container) {
    case ContainerKind::kFixedArray:
      if (type.capacity > 0) {
        suffix = "[" + std::to_string(type.capacity) + "]";
      } else {
        suffix = "[]";
      }
      break;
    case ContainerKind::kBoundedSequence:
      suffix = "[bounded]";
      break;
    case ContainerKind::kUnboundedSequence:
      suffix = "[]";
      break;
    case ContainerKind::kSingular:
      break;
  }

  std::string type_name = "unknown";
  switch (decoded.base_type_id) {
    case FieldType::FIELD_TYPE_NESTED_TYPE:
      type_name = type.nested_type_name;
      break;
    case FieldType::FIELD_TYPE_INT8:
      type_name = "int8";
      break;
    case FieldType::FIELD_TYPE_UINT8:
      type_name = "uint8";
      break;
    case FieldType::FIELD_TYPE_INT16:
      type_name = "int16";
      break;
    case FieldType::FIELD_TYPE_UINT16:
      type_name = "uint16";
      break;
    case FieldType::FIELD_TYPE_INT32:
      type_name = "int32";
      break;
    case FieldType::FIELD_TYPE_UINT32:
      type_name = "uint32";
      break;
    case FieldType::FIELD_TYPE_INT64:
      type_name = "int64";
      break;
    case FieldType::FIELD_TYPE_UINT64:
      type_name = "uint64";
      break;
    case FieldType::FIELD_TYPE_FLOAT:
      type_name = "float";
      break;
    case FieldType::FIELD_TYPE_DOUBLE:
      type_name = "double";
      break;
    case FieldType::FIELD_TYPE_BOOLEAN:
      type_name = "bool";
      break;
    case FieldType::FIELD_TYPE_BYTE:
      type_name = "byte";
      break;
    case FieldType::FIELD_TYPE_CHAR:
      type_name = "char";
      break;
    case FieldType::FIELD_TYPE_STRING:
    case FieldType::FIELD_TYPE_BOUNDED_STRING:
      type_name = "string";
      break;
    default:
      type_name = "primitive_" + std::to_string(decoded.base_type_id);
      break;
  }
  return type_name + suffix;
}

uint8_t string_to_field_type_id(const std::string& type_str) {
  if (type_str == "int8") return FieldType::FIELD_TYPE_INT8;
  if (type_str == "uint8") return FieldType::FIELD_TYPE_UINT8;
  if (type_str == "int16") return FieldType::FIELD_TYPE_INT16;
  if (type_str == "uint16") return FieldType::FIELD_TYPE_UINT16;
  if (type_str == "int32") return FieldType::FIELD_TYPE_INT32;
  if (type_str == "uint32") return FieldType::FIELD_TYPE_UINT32;
  if (type_str == "int64") return FieldType::FIELD_TYPE_INT64;
  if (type_str == "uint64") return FieldType::FIELD_TYPE_UINT64;
  if (type_str == "float" || type_str == "float32") {
    return FieldType::FIELD_TYPE_FLOAT;
  }
  if (type_str == "double" || type_str == "float64") {
    return FieldType::FIELD_TYPE_DOUBLE;
  }
  if (type_str == "bool" || type_str == "boolean") {
    return FieldType::FIELD_TYPE_BOOLEAN;
  }
  if (type_str == "byte") return FieldType::FIELD_TYPE_BYTE;
  if (type_str == "char") return FieldType::FIELD_TYPE_CHAR;
  if (type_str == "string") return FieldType::FIELD_TYPE_STRING;
  return FieldType::FIELD_TYPE_NOT_SET;
}

uint8_t parse_field_type(const std::string& type_str,
                         std::string& nested_type_name, uint64_t& capacity) {
  nested_type_name.clear();
  capacity = 0;

  std::string base_type = type_str;
  ContainerKind container = ContainerKind::kSingular;

  const size_t open_bracket = type_str.find('[');
  if (open_bracket != std::string::npos && type_str.back() == ']') {
    base_type = type_str.substr(0, open_bracket);
    const std::string bracket_inner =
        type_str.substr(open_bracket + 1, type_str.size() - open_bracket - 2);
    if (bracket_inner.empty()) {
      container = ContainerKind::kUnboundedSequence;
    } else if (bracket_inner == "bounded") {
      container = ContainerKind::kBoundedSequence;
    } else if (bracket_inner.rfind("<=", 0) == 0) {
      container = ContainerKind::kBoundedSequence;
      const std::string bound_digits = bracket_inner.substr(2);
      if (!bound_digits.empty() &&
          std::all_of(bound_digits.begin(), bound_digits.end(),
                      [](unsigned char c) { return std::isdigit(c) != 0; })) {
        capacity = std::stoull(bound_digits);
      }
    } else if (std::all_of(
                   bracket_inner.begin(), bracket_inner.end(),
                   [](unsigned char c) { return std::isdigit(c) != 0; })) {
      capacity = std::stoull(bracket_inner);
      container = (capacity > 0) ? ContainerKind::kFixedArray
                                 : ContainerKind::kUnboundedSequence;
    } else {
      container = ContainerKind::kUnboundedSequence;
    }
  }

  const uint8_t primitive_id = string_to_field_type_id(base_type);
  if (primitive_id != FieldType::FIELD_TYPE_NOT_SET) {
    switch (container) {
      case ContainerKind::kFixedArray:
        return static_cast<uint8_t>(primitive_id + 48);
      case ContainerKind::kBoundedSequence:
        return static_cast<uint8_t>(primitive_id + 96);
      case ContainerKind::kUnboundedSequence:
        return static_cast<uint8_t>(primitive_id + 144);
      case ContainerKind::kSingular:
        return primitive_id;
    }
  }

  nested_type_name = base_type;
  switch (container) {
    case ContainerKind::kFixedArray:
      return FieldType::FIELD_TYPE_NESTED_TYPE_ARRAY;
    case ContainerKind::kBoundedSequence:
      return FieldType::FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE;
    case ContainerKind::kUnboundedSequence:
      return FieldType::FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE;
    case ContainerKind::kSingular:
      return FieldType::FIELD_TYPE_NESTED_TYPE;
  }
  return FieldType::FIELD_TYPE_NESTED_TYPE;
}

uint8_t parse_field_type(const std::string& type_str,
                         std::string& nested_type_name) {
  uint64_t ignored_capacity = 0;
  return parse_field_type(type_str, nested_type_name, ignored_capacity);
}

bool parse_ros_type(const std::string& ros_type, std::string& package,
                    std::string& base_name) {
  const size_t first_slash = ros_type.find('/');
  const size_t last_slash = ros_type.rfind('/');
  if (first_slash == std::string::npos || last_slash == std::string::npos) {
    return false;
  }
  package = ros_type.substr(0, first_slash);
  base_name = ros_type.substr(last_slash + 1);
  return true;
}

const type_description_interfaces::msg::IndividualTypeDescription*
FindIndividualType(
    const type_description_interfaces::msg::TypeDescription& type_desc,
    const std::string& suffix) {
  if (type_desc.type_description.type_name.size() >= suffix.size() &&
      type_desc.type_description.type_name.rfind(suffix) ==
          type_desc.type_description.type_name.size() - suffix.size()) {
    return &type_desc.type_description;
  }
  for (const auto& ref : type_desc.referenced_type_descriptions) {
    if (ref.type_name.size() >= suffix.size() &&
        ref.type_name.rfind(suffix) == ref.type_name.size() - suffix.size()) {
      return &ref;
    }
  }
  return nullptr;
}

DynamicLayout ComputeDynamicLayout(
    const type_description_interfaces::msg::IndividualTypeDescription&
        type_desc,
    const type_description_interfaces::msg::TypeDescription& parent_desc) {
  DynamicLayout layout;
  size_t current_offset = 0;
  size_t max_alignment = 1;

  for (const auto& field : type_desc.fields) {
    const DecodedFieldType decoded = DecodeFieldTypeId(field.type.type_id);
    size_t field_size = 0;
    size_t alignment = 1;

    if (decoded.container == ContainerKind::kSingular) {
      GetBaseElementSizeAndAlignment(decoded.base_type_id,
                                     field.type.nested_type_name, parent_desc,
                                     field_size, alignment);
    } else if (decoded.container == ContainerKind::kFixedArray) {
      size_t elem_size = 0;
      GetBaseElementSizeAndAlignment(decoded.base_type_id,
                                     field.type.nested_type_name, parent_desc,
                                     elem_size, alignment);
      field_size = elem_size * static_cast<size_t>(field.type.capacity);
    } else if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
      field_size = sizeof(GenericMessageSequence);
      alignment = alignof(GenericMessageSequence);
    } else {
      field_size = sizeof(GenericPrimitiveSequence);
      alignment = alignof(GenericPrimitiveSequence);
    }

    if (alignment > max_alignment) {
      max_alignment = alignment;
    }
    if (current_offset % alignment != 0) {
      current_offset += (alignment - (current_offset % alignment));
    }

    DynamicField df;
    df.name = field.name;
    df.type_id = field.type.type_id;
    df.offset = current_offset;
    df.array_capacity = static_cast<size_t>(field.type.capacity);
    df.nested_type_name = field.type.nested_type_name;

    layout.fields.push_back(df);
    current_offset += field_size;
  }

  if (current_offset % max_alignment != 0) {
    current_offset += (max_alignment - (current_offset % max_alignment));
  }

  layout.total_size = current_offset;
  layout.max_alignment = max_alignment;
  return layout;
}

void MapCStructToProtobuf(
    const DynamicLayout& layout, const uint8_t* source_buf,
    google::protobuf::Message& dest_proto,
    const type_description_interfaces::msg::TypeDescription& parent_desc) {
  const google::protobuf::Descriptor* desc = dest_proto.GetDescriptor();
  const google::protobuf::Reflection* reflection = dest_proto.GetReflection();

  for (const auto& f : layout.fields) {
    const google::protobuf::FieldDescriptor* pf =
        ResolveProtoField(desc, f.name);
    if (pf == nullptr) {
      continue;
    }

    const DecodedFieldType decoded = DecodeFieldTypeId(f.type_id);

    if (decoded.is_collection()) {
      reflection->ClearField(&dest_proto, pf);

      const void* elem_data = nullptr;
      size_t elem_count = 0;

      if (decoded.container == ContainerKind::kFixedArray) {
        elem_data = source_buf + f.offset;
        elem_count = f.array_capacity;
      } else if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
        GenericMessageSequence msg_seq{};
        std::memcpy(&msg_seq, source_buf + f.offset,
                    sizeof(GenericMessageSequence));
        elem_data = msg_seq.data;
        elem_count = msg_seq.size;
      } else {
        GenericPrimitiveSequence prim_seq{};
        std::memcpy(&prim_seq, source_buf + f.offset,
                    sizeof(GenericPrimitiveSequence));
        elem_data = prim_seq.data;
        elem_count = prim_seq.size;
      }

      if ((decoded.base_type_id == FieldType::FIELD_TYPE_UINT8 ||
           decoded.base_type_id == FieldType::FIELD_TYPE_BYTE) &&
          pf->type() == google::protobuf::FieldDescriptor::TYPE_BYTES) {
        if (elem_data != nullptr && elem_count > 0) {
          reflection->SetString(
              &dest_proto, pf,
              std::string(reinterpret_cast<const char*>(elem_data),
                          elem_count));
        } else {
          reflection->SetString(&dest_proto, pf, "");
        }
        continue;
      }

      if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
        const auto* nested_desc =
            FindIndividualType(parent_desc, f.nested_type_name);
        if (nested_desc != nullptr && elem_data != nullptr) {
          auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
          const size_t element_size = nested_layout.total_size;
          const uint8_t* raw = reinterpret_cast<const uint8_t*>(elem_data);
          if (pf->is_repeated()) {
            for (size_t i = 0; i < elem_count; ++i) {
              google::protobuf::Message* nested_proto =
                  reflection->AddMessage(&dest_proto, pf);
              MapCStructToProtobuf(nested_layout, raw + i * element_size,
                                   *nested_proto, parent_desc);
            }
          } else if (elem_count > 0) {
            google::protobuf::Message* nested_proto =
                reflection->MutableMessage(&dest_proto, pf);
            MapCStructToProtobuf(nested_layout, raw, *nested_proto,
                                 parent_desc);
          }
        }
      } else if (IsStringTypeId(decoded.base_type_id)) {
        const auto* data =
            reinterpret_cast<const rosidl_runtime_c__String*>(elem_data);
        if (data != nullptr) {
          if (pf->is_repeated()) {
            for (size_t i = 0; i < elem_count; ++i) {
              if (data[i].data != nullptr) {
                reflection->AddString(&dest_proto, pf,
                                      std::string(data[i].data, data[i].size));
              } else {
                reflection->AddString(&dest_proto, pf, "");
              }
            }
          } else if (elem_count > 0 && data[0].data != nullptr) {
            reflection->SetString(&dest_proto, pf,
                                  std::string(data[0].data, data[0].size));
          }
        }
      } else {
        DispatchPrimitivesToProto(decoded.base_type_id, elem_data, elem_count,
                                  &dest_proto, reflection, pf);
      }
      continue;
    }

    if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
      google::protobuf::Message* nested_proto =
          reflection->MutableMessage(&dest_proto, pf);
      const auto* nested_desc =
          FindIndividualType(parent_desc, f.nested_type_name);
      if (nested_desc != nullptr) {
        auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
        MapCStructToProtobuf(nested_layout, source_buf + f.offset,
                             *nested_proto, parent_desc);
      }
      continue;
    }

    if (IsStringTypeId(decoded.base_type_id)) {
      rosidl_runtime_c__String v{};
      std::memcpy(&v, source_buf + f.offset, sizeof(rosidl_runtime_c__String));
      if (v.data != nullptr) {
        reflection->SetString(&dest_proto, pf, std::string(v.data, v.size));
      } else {
        reflection->SetString(&dest_proto, pf, "");
      }
      continue;
    }

    DispatchPrimitivesToProto(decoded.base_type_id, source_buf + f.offset, 1,
                              &dest_proto, reflection, pf);
  }
}

void MapProtobufToCStruct(
    const google::protobuf::Message& proto_msg, const DynamicLayout& layout,
    uint8_t* dest_buf, std::vector<std::unique_ptr<uint8_t[]>>& allocations,
    const type_description_interfaces::msg::TypeDescription& parent_desc) {
  if (layout.total_size > 0 && dest_buf != nullptr) {
    std::memset(dest_buf, 0, layout.total_size);
  }

  const google::protobuf::Descriptor* desc = proto_msg.GetDescriptor();
  const google::protobuf::Reflection* reflection = proto_msg.GetReflection();

  for (const auto& f : layout.fields) {
    const google::protobuf::FieldDescriptor* pf =
        ResolveProtoField(desc, f.name);
    if (pf == nullptr || f.name == "action_name") {
      InitEmptyCStructField(f, dest_buf, allocations, parent_desc);
      continue;
    }

    const DecodedFieldType decoded = DecodeFieldTypeId(f.type_id);

    if (decoded.container == ContainerKind::kFixedArray) {
      if ((decoded.base_type_id == FieldType::FIELD_TYPE_UINT8 ||
           decoded.base_type_id == FieldType::FIELD_TYPE_BYTE) &&
          pf->type() == google::protobuf::FieldDescriptor::TYPE_BYTES) {
        std::string v = reflection->GetString(proto_msg, pf);
        const size_t n = std::min(v.size(), f.array_capacity);
        if (n > 0) {
          std::memcpy(dest_buf + f.offset, v.data(), n);
        }
        continue;
      }

      size_t proto_count = 0;
      if (pf->is_repeated()) {
        proto_count = static_cast<size_t>(reflection->FieldSize(proto_msg, pf));
      } else if (pf->has_presence()) {
        proto_count = reflection->HasField(proto_msg, pf) ? 1 : 0;
      } else {
        proto_count = 1;
      }
      const size_t copy_count = std::min(proto_count, f.array_capacity);

      if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
        const auto* nested_desc =
            FindIndividualType(parent_desc, f.nested_type_name);
        if (nested_desc != nullptr) {
          auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
          const size_t elem_size = nested_layout.total_size;
          for (size_t i = 0; i < copy_count; ++i) {
            const google::protobuf::Message& nested_proto =
                pf->is_repeated() ? reflection->GetRepeatedMessage(
                                        proto_msg, pf, static_cast<int>(i))
                                  : reflection->GetMessage(proto_msg, pf);
            MapProtobufToCStruct(nested_proto, nested_layout,
                                 dest_buf + f.offset + i * elem_size,
                                 allocations, parent_desc);
          }
          for (size_t i = copy_count; i < f.array_capacity; ++i) {
            uint8_t* elem_ptr = dest_buf + f.offset + i * elem_size;
            for (const auto& nf : nested_layout.fields) {
              InitEmptyCStructField(nf, elem_ptr, allocations, parent_desc);
            }
          }
        }
      } else if (IsStringTypeId(decoded.base_type_id)) {
        const size_t elem_size = sizeof(rosidl_runtime_c__String);
        for (size_t i = 0; i < copy_count; ++i) {
          std::string v = pf->is_repeated()
                              ? reflection->GetRepeatedString(
                                    proto_msg, pf, static_cast<int>(i))
                              : reflection->GetString(proto_msg, pf);
          WriteRosStringAt(dest_buf + f.offset + i * elem_size, v, allocations);
        }
        for (size_t i = copy_count; i < f.array_capacity; ++i) {
          WriteRosStringAt(dest_buf + f.offset + i * elem_size, "",
                           allocations);
        }
      } else {
        DispatchProtoToPrimitives(decoded.base_type_id, proto_msg, reflection,
                                  pf, copy_count, dest_buf + f.offset);
      }
      continue;
    }

    if (decoded.is_sequence()) {
      if ((decoded.base_type_id == FieldType::FIELD_TYPE_UINT8 ||
           decoded.base_type_id == FieldType::FIELD_TYPE_BYTE) &&
          pf->type() == google::protobuf::FieldDescriptor::TYPE_BYTES) {
        std::string v = reflection->GetString(proto_msg, pf);
        GenericPrimitiveSequence seq{};
        if (!v.empty()) {
          auto arr_data = std::make_unique<uint8_t[]>(v.size());
          std::memcpy(arr_data.get(), v.data(), v.size());
          seq.data = arr_data.get();
          seq.size = v.size();
          seq.capacity = v.size();
          allocations.push_back(std::move(arr_data));
        }
        std::memcpy(dest_buf + f.offset, &seq,
                    sizeof(GenericPrimitiveSequence));
        continue;
      }

      size_t proto_count = 0;
      if (pf->is_repeated()) {
        proto_count = static_cast<size_t>(reflection->FieldSize(proto_msg, pf));
      } else if (pf->has_presence()) {
        proto_count = reflection->HasField(proto_msg, pf) ? 1 : 0;
      } else {
        proto_count = 1;
      }

      if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
        const auto* nested_desc =
            FindIndividualType(parent_desc, f.nested_type_name);
        if (nested_desc != nullptr) {
          auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
          const size_t elem_size = nested_layout.total_size;
          GenericMessageSequence seq{};
          if (proto_count > 0 && elem_size > 0) {
            auto arr_data =
                std::make_unique<uint8_t[]>(proto_count * elem_size);
            std::memset(arr_data.get(), 0, proto_count * elem_size);
            for (size_t i = 0; i < proto_count; ++i) {
              const google::protobuf::Message& nested_proto =
                  pf->is_repeated() ? reflection->GetRepeatedMessage(
                                          proto_msg, pf, static_cast<int>(i))
                                    : reflection->GetMessage(proto_msg, pf);
              MapProtobufToCStruct(nested_proto, nested_layout,
                                   arr_data.get() + i * elem_size, allocations,
                                   parent_desc);
            }
            seq.data = arr_data.get();
            seq.size = proto_count;
            seq.capacity = proto_count;
            allocations.push_back(std::move(arr_data));
          }
          std::memcpy(dest_buf + f.offset, &seq,
                      sizeof(GenericMessageSequence));
        }
      } else if (IsStringTypeId(decoded.base_type_id)) {
        const size_t elem_size = sizeof(rosidl_runtime_c__String);
        GenericPrimitiveSequence seq{};
        if (proto_count > 0) {
          auto arr_data = std::make_unique<uint8_t[]>(proto_count * elem_size);
          std::memset(arr_data.get(), 0, proto_count * elem_size);
          for (size_t i = 0; i < proto_count; ++i) {
            std::string v = pf->is_repeated()
                                ? reflection->GetRepeatedString(
                                      proto_msg, pf, static_cast<int>(i))
                                : reflection->GetString(proto_msg, pf);
            WriteRosStringAt(arr_data.get() + i * elem_size, v, allocations);
          }
          seq.data = arr_data.get();
          seq.size = proto_count;
          seq.capacity = proto_count;
          allocations.push_back(std::move(arr_data));
        }
        std::memcpy(dest_buf + f.offset, &seq,
                    sizeof(GenericPrimitiveSequence));
      } else {
        size_t elem_size = 0;
        size_t elem_align = 1;
        GetBaseElementSizeAndAlignment(decoded.base_type_id, "", parent_desc,
                                       elem_size, elem_align);
        GenericPrimitiveSequence seq{};
        if (proto_count > 0 && elem_size > 0) {
          auto arr_data = std::make_unique<uint8_t[]>(proto_count * elem_size);
          std::memset(arr_data.get(), 0, proto_count * elem_size);
          DispatchProtoToPrimitives(decoded.base_type_id, proto_msg, reflection,
                                    pf, proto_count, arr_data.get());
          seq.data = arr_data.get();
          seq.size = proto_count;
          seq.capacity = proto_count;
          allocations.push_back(std::move(arr_data));
        }
        std::memcpy(dest_buf + f.offset, &seq,
                    sizeof(GenericPrimitiveSequence));
      }
      continue;
    }

    if (decoded.base_type_id == FieldType::FIELD_TYPE_NESTED_TYPE) {
      if (pf->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        const google::protobuf::Message& nested_proto =
            reflection->GetMessage(proto_msg, pf);
        const auto* nested_desc =
            FindIndividualType(parent_desc, f.nested_type_name);
        if (nested_desc != nullptr) {
          auto nested_layout = ComputeDynamicLayout(*nested_desc, parent_desc);
          MapProtobufToCStruct(nested_proto, nested_layout, dest_buf + f.offset,
                               allocations, parent_desc);
        }
      } else {
        InitEmptyCStructField(f, dest_buf, allocations, parent_desc);
      }
      continue;
    }

    if (IsStringTypeId(decoded.base_type_id)) {
      WriteRosStringAt(dest_buf + f.offset,
                       reflection->GetString(proto_msg, pf), allocations);
      continue;
    }

    DispatchProtoToPrimitives(decoded.base_type_id, proto_msg, reflection, pf,
                              1, dest_buf + f.offset);
  }
}

}  // namespace aquarium_gateway
