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

#ifndef FLOWSTATE_ROS_BRIDGE__ROS_NAME_UTILS_HPP_
#define FLOWSTATE_ROS_BRIDGE__ROS_NAME_UTILS_HPP_

#include <cstddef>
#include <string>
#include <string_view>

#include "rmw/error_handling.h"
#include "rmw/types.h"
#include "rmw/validate_namespace.h"

namespace flowstate_ros_bridge {

/// Strips leading and trailing slashes: "/wc1/" -> "wc1".
inline std::string_view TrimSlashes(std::string_view name) {
  const std::size_t begin = name.find_first_not_of('/');
  if (begin == std::string_view::npos) {
    return {};
  }
  const std::size_t end = name.find_last_not_of('/');
  return name.substr(begin, end - begin + 1);
}

/// Normalizes a TF frame prefix: "/wc1/" -> "wc1/", "/" -> "".
inline std::string NormalizeTfPrefix(std::string_view prefix) {
  const std::string_view trimmed = TrimSlashes(prefix);
  if (trimmed.empty()) {
    return {};
  }
  std::string normalized(trimmed);
  normalized += '/';
  return normalized;
}

/// Prepends a normalized TF prefix ("" or "<prefix>/") to a frame ID:
/// ("wc1/", "/base_link") -> "wc1/base_link". Leading slashes are dropped
/// since tf2 rejects them, and an empty frame ID stays empty.
inline std::string PrefixFrameId(std::string_view tf_prefix,
                                 std::string_view frame_id) {
  const std::size_t begin = frame_id.find_first_not_of('/');
  if (begin == std::string_view::npos) {
    return {};
  }
  std::string prefixed(tf_prefix);
  prefixed.append(frame_id.substr(begin));
  return prefixed;
}

/// Checks that "/<workcell_id>" is a valid ROS namespace, e.g. "wc1" or
/// "cell_a/arm". Returns an empty string if it is valid (an empty workcell_id
/// is the root namespace), otherwise the reason it is invalid.
inline std::string ValidateWorkcellId(std::string_view workcell_id) {
  const std::string ns = "/" + std::string(workcell_id);
  int validation_result = RMW_NAMESPACE_VALID;
  std::size_t invalid_index = 0;
  if (rmw_validate_namespace(ns.c_str(), &validation_result, &invalid_index) !=
      RMW_RET_OK) {
    rmw_reset_error();
    return "unable to validate namespace '" + ns + "'";
  }
  if (validation_result == RMW_NAMESPACE_VALID) {
    return {};
  }
  const char* reason =
      rmw_namespace_validation_result_string(validation_result);
  return std::string(reason != nullptr ? reason : "invalid namespace") +
         ", at index " + std::to_string(invalid_index) + " of '" + ns + "'";
}

}  // namespace flowstate_ros_bridge

#endif  // FLOWSTATE_ROS_BRIDGE__ROS_NAME_UTILS_HPP_
