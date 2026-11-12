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

#include "aquarium_setup_assistant/config_writer.hpp"

#include <algorithm>
#include <fstream>

namespace aquarium_setup_assistant {

bool parse_cli_target_args(const std::vector<std::string>& args,
                           TargetSpec& out_spec) {
  out_spec = TargetSpec{};
  std::vector<std::string> pos_args;

  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--selection" || args[i] == "-s") {
      if (i + 1 < args.size()) {
        out_spec.selection_file = args[i + 1];
        ++i;
      }
      continue;
    }
    pos_args.push_back(args[i]);
  }

  if (pos_args.empty()) {
    return false;
  }

  if (pos_args[0] == "launch" || pos_args[0] == "run") {
    if (pos_args.size() < 3) {
      return false;
    }
    out_spec.type = (pos_args[0] == "run") ? "node" : "launch";
    out_spec.package = pos_args[1];
    out_spec.name = pos_args[2];
    out_spec.arguments.assign(pos_args.begin() + 3, pos_args.end());
    return true;
  }

  if (pos_args.size() < 2) {
    return false;
  }
  out_spec.type = "launch";
  out_spec.package = pos_args[0];
  out_spec.name = pos_args[1];
  out_spec.arguments.assign(pos_args.begin() + 2, pos_args.end());
  return true;
}

bool write_config_yaml(const std::shared_ptr<rclcpp::Node>& node,
                       const std::vector<InterfaceRecord>& selected_interfaces,
                       const std::string& filename) {
  std::string target_type = "";
  std::string target_package = "";
  std::string target_name = "";
  std::vector<std::string> target_arguments;

  try {
    target_type = node->get_parameter("target_type").as_string();
    target_package = node->get_parameter("target_package").as_string();
    target_name = node->get_parameter("target_name").as_string();
    target_arguments =
        node->get_parameter("target_arguments").as_string_array();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
    RCLCPP_WARN(
        node->get_logger(),
        "Target parameters not declared: %s. Writing empty target spec.",
        e.what());
  }

  std::ofstream out(filename);
  if (!out.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open file %s for writing",
                 filename.c_str());
    return false;
  }

  out << "target:\n";
  out << "  type: \"" << target_type << "\"\n";
  out << "  package: \"" << target_package << "\"\n";
  out << "  name: \"" << target_name << "\"\n";
  if (!target_arguments.empty()) {
    out << "  arguments:\n";
    for (const auto& arg : target_arguments) {
      out << "    - \"" << arg << "\"\n";
    }
  }
  out << "\n";

  if (!selected_interfaces.empty()) {
    out << "interfaces:\n";
    for (const auto& inf : selected_interfaces) {
      std::string ext_name = inf.name;
      size_t last_slash = inf.name.rfind('/');
      if (last_slash != std::string::npos) {
        ext_name = inf.name.substr(last_slash + 1);
      }

      std::string ext_type = inf.type;
      std::replace(ext_type.begin(), ext_type.end(), '/', '.');

      std::string internal_interface;
      if (inf.is_action) {
        internal_interface = "ros_action_client";
      } else if (inf.is_service) {
        internal_interface = "ros_service";
      } else {
        internal_interface = "ros_topic_subscription";
      }

      out << "  - internal_interface: \"" << internal_interface << "\"\n";
      out << "    internal_name: \"" << inf.name << "\"\n";
      out << "    internal_type: \"" << inf.type << "\"\n";
      out << "    external_interface: \"zenoh_queryable_proto\"\n";
      out << "    external_name: \"" << ext_name << "\"\n";
      out << "    external_type: \"" << ext_type << "\"\n";

      if (inf.has_introspection) {
        out << "    type_hash: \"" << inf.type_hash << "\"\n";
        if (!inf.request_fields.empty()) {
          out << "    request_fields:\n";
          for (const auto& f : inf.request_fields) {
            out << "      - \"" << f << "\"\n";
          }
        }
        if (!inf.response_fields.empty()) {
          out << "    response_fields:\n";
          for (const auto& f : inf.response_fields) {
            out << "      - \"" << f << "\"\n";
          }
        }
      }
    }
  }

  out.close();
  return true;
}

}  // namespace aquarium_setup_assistant
