// Copyright 2023 Intrinsic Innovation LLC

#include "intrinsic/platform/pubsub/zenoh_util/zenoh_helpers.h"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
// #include "ortools/base/path.h"

namespace intrinsic {

bool RunningUnderTest() {
  return (getenv("TEST_TMPDIR") != nullptr) ||
         (getenv("PORTSERVER_ADDRESS") != nullptr);
}

bool RunningInKubernetes() {
  return getenv("KUBERNETES_SERVICE_HOST") != nullptr;
}

#ifndef INTRINSIC_SDK_CMAKE_LIB_PATH
#error "Compiler definition INTRINSIC_SDK_CMAKE_LIB_PATH is required."
#endif

#ifndef INTRINSIC_SDK_CMAKE_SHARE_PATH
#error "Compiler definition INTRINSIC_SDK_CMAKE_SHARE_PATH is required."
#endif

std::string GetZenohRunfilesPath(absl::string_view file_path) {
  // Patched by intrinsic-dev/intrinsic_sdk_ros.git to work in a CMake context.
  // TODO(wjwwood): consider using ament_index to make this code relocatable.
  std::array<std::filesystem::path, 2> prefix_paths = {
    std::filesystem::path(INTRINSIC_SDK_CMAKE_LIB_PATH),
    std::filesystem::path(INTRINSIC_SDK_CMAKE_SHARE_PATH)
  };

  // Prefer a more complete path in the lib or share path over just the give file_path.
  for (const auto & prefix_path : prefix_paths) {
    std::filesystem::path possible_path = prefix_path / file_path;
    if (std::filesystem::exists(possible_path)) {
      return possible_path.string();
    }
  }

  // Return the original file_path if nothing better was found.
  return std::string(file_path);
}

absl::Status ValidZenohKeyexpr(absl::string_view keyexpr) {
  if (keyexpr.empty()) {
    return absl::InvalidArgumentError("Keyexpr must not be empty");
  }
  if (absl::StartsWith(keyexpr, "/")) {
    return absl::InvalidArgumentError("Keyexpr must not start with /");
  }
  if (absl::EndsWith(keyexpr, "/")) {
    return absl::InvalidArgumentError("Keyexpr must not end with /");
  }
  std::vector<std::string> parts = absl::StrSplit(keyexpr, '/');
  for (absl::string_view part : parts) {
    if (part.empty()) {
      return absl::InvalidArgumentError("Keyexpr must not contain empty parts");
    }
    if (part == "*" || part == "$*" || part == "**") {
      continue;
    }
  }
  return absl::OkStatus();
}

absl::Status ValidZenohKey(absl::string_view key) {
  if (key.empty()) {
    return absl::InvalidArgumentError("Keyexpr must not be empty");
  }
  if (absl::StartsWith(key, "/")) {
    return absl::InvalidArgumentError("Keyexpr must not start with /");
  }
  if (absl::EndsWith(key, "/")) {
    return absl::InvalidArgumentError("Keyexpr must not end with /");
  }
  std::vector<std::string> parts = absl::StrSplit(key, '/');
  for (absl::string_view part : parts) {
    if (part.empty()) {
      return absl::InvalidArgumentError("Keyexpr must not contain empty parts");
    }
  }
  return absl::OkStatus();
}

}  // namespace intrinsic
