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

}  // namespace flowstate_ros_bridge

#endif  // FLOWSTATE_ROS_BRIDGE__ROS_NAME_UTILS_HPP_
