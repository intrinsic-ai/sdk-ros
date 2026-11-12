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

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "aquarium_gateway/serialization.hpp"
#include "aquarium_setup_assistant/config_writer.hpp"
#include "aquarium_setup_assistant/terminal_ui.hpp"
#include "rcl_action/graph.h"
#include "rcl_action/rcl_action.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "type_description_interfaces/msg/field_type.hpp"
#include "type_description_interfaces/srv/get_type_description.hpp"
#include "zenoh.hxx"

namespace aquarium_setup_assistant {

static void SpinNodeSome(const std::shared_ptr<rclcpp::Node>& node) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

// Query Zenoh liveliness tokens to resolve the RIHS01 type hash on the wire.
static std::string discover_interface_type_hash(
    const std::shared_ptr<rclcpp::Node>& logger_node,
    const std::string& interface_name,
    const std::vector<std::string>& allowed_kinds) {
  std::string resolved_hash;
  std::mutex match_mtx;
  std::condition_variable match_cv;
  bool match_found = false;

  // Replace slashes with percent signs to match rmw_zenoh liveliness tokens.
  std::string match_token = interface_name;
  std::replace(match_token.begin(), match_token.end(), '/', '%');

  // Delimit with slashes to avoid matching partial prefix segments.
  const std::string strict_search = "/" + match_token + "/";

  try {
    const std::string json_config = R"({
      "mode": "client",
      "connect": {
        "endpoints": ["tcp/127.0.0.1:7447"]
      },
      "adminspace": {
        "enabled": true
      }
    })";

    auto config = zenoh::Config::from_str(json_config);
    auto session = zenoh::Session::open(std::move(config));

    RCLCPP_INFO(logger_node->get_logger(),
                "Querying Zenoh liveliness tokens for '%s'...",
                strict_search.c_str());

    session.liveliness_get(
        "@ros2_lv/**",
        [&](zenoh::Reply& reply) {
          if (!reply.is_ok()) {
            return;
          }
          const std::string key_str(
              reply.get_ok().get_keyexpr().as_string_view());

          bool kind_match = false;
          for (const auto& kind : allowed_kinds) {
            if (key_str.find("/" + kind + "/") != std::string::npos) {
              kind_match = true;
              break;
            }
          }

          if (key_str.find(strict_search) != std::string::npos && kind_match) {
            std::stringstream ss(key_str);
            std::string segment;
            while (std::getline(ss, segment, '/')) {
              if (segment.find("RIHS01_") == 0 && segment.length() == 71) {
                std::lock_guard<std::mutex> lock(match_mtx);
                resolved_hash = segment;
                match_found = true;
                match_cv.notify_one();
                break;
              }
            }
          }
        },
        []() {});

    std::unique_lock<std::mutex> lock(match_mtx);
    match_cv.wait_for(lock, std::chrono::seconds(2),
                      [&] { return match_found; });
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_node->get_logger(), "Zenoh liveliness lookup failed: %s",
                e.what());
  }

  return resolved_hash;
}

static bool fetch_dynamic_introspection(
    const std::shared_ptr<rclcpp::Node>& client_node, const std::string& name,
    const std::string& type, UiMode mode, IntrospectionRecord& out_record) {
  out_record.interface_name = name;
  out_record.type_name = type;
  out_record.request_fields.clear();
  out_record.response_fields.clear();
  out_record.type_hash =
      "RIHS01_0000000000000000000000000000000000000000000000000000000000000000 "
      "(NOT RETRIEVED)";

  // 1. Resolve dynamic type hash via action typesupport or Zenoh liveliness.
  std::string genuine_hash;

  if (mode == MODE_ACTION) {
    try {
      auto ts_lib =
          rclcpp::get_typesupport_library(type, "rosidl_typesupport_c");
      if (ts_lib) {
        std::string pkg;
        std::string mod;
        std::string typ;
        std::tie(pkg, mod, typ) = rclcpp::extract_type_identifier(type);
        if (mod.empty()) mod = "action";
        const std::string symbol_name =
            "rosidl_typesupport_c__get_action_type_support_handle__" + pkg +
            "__" + mod + "__" + typ;
        auto get_ts =
            reinterpret_cast<const rosidl_action_type_support_t* (*)()>(
                ts_lib->get_symbol(symbol_name));
        if (get_ts != nullptr) {
          const rosidl_action_type_support_t* ts = get_ts();
          if (ts != nullptr && ts->get_type_hash_func != nullptr) {
            const rosidl_type_hash_t* th = ts->get_type_hash_func(ts);
            if (th != nullptr) {
              char* hash_c_str = nullptr;
              rcutils_allocator_t allocator = rcutils_get_default_allocator();
              if (rosidl_stringify_type_hash(th, allocator, &hash_c_str) ==
                      RCUTILS_RET_OK &&
                  hash_c_str != nullptr) {
                genuine_hash = hash_c_str;
                allocator.deallocate(hash_c_str, allocator.state);
              }
            }
          }
        }
      }
    } catch (...) {
    }

    if (genuine_hash.empty()) {
      genuine_hash = discover_interface_type_hash(
          client_node, name + "/_action/send_goal", {"SS"});
      if (genuine_hash.empty()) {
        genuine_hash = discover_interface_type_hash(client_node, name, {"SS"});
      }
    }
  } else {
    const std::vector<std::string> allowed_kinds =
        (mode == MODE_SERVICE) ? std::vector<std::string>{"SS"}
                               : std::vector<std::string>{"MP", "MS"};
    genuine_hash =
        discover_interface_type_hash(client_node, name, allowed_kinds);
  }

  if (genuine_hash.empty()) {
    RCLCPP_ERROR(client_node->get_logger(),
                 "TDS query failed: unable to resolve dynamic type hash.");
    const std::string err_msg =
        "(None) - Introspection failed: Liveliness token lookup timed out over "
        "Zenoh.";
    if (mode == MODE_SERVICE || mode == MODE_ACTION) {
      out_record.request_fields = {err_msg};
    }
    out_record.response_fields = {err_msg};
    return false;
  }

  out_record.type_hash = genuine_hash;

  // 2. Discover the host ROS 2 node exposing the target endpoint.
  std::string host_node_name;
  const auto start_search = std::chrono::steady_clock::now();
  const auto search_timeout = std::chrono::seconds(5);

  while ((std::chrono::steady_clock::now() - start_search) < search_timeout &&
         host_node_name.empty()) {
    auto node_names = client_node->get_node_names();
    for (const auto& n : node_names) {
      const size_t last_slash = n.rfind('/');
      std::string bare_name = n;
      std::string ns = "/";
      if (last_slash != std::string::npos) {
        bare_name = n.substr(last_slash + 1);
        ns = n.substr(0, last_slash);
        if (ns.empty()) ns = "/";
      }

      auto graph = client_node->get_node_graph_interface();
      if (mode == MODE_ACTION) {
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
        auto* rcl_node =
            client_node->get_node_base_interface()->get_rcl_node_handle();
        rcl_ret_t ret = rcl_action_get_server_names_and_types_by_node(
            rcl_node, &allocator, bare_name.c_str(), ns.c_str(), &nat);
        if (ret == RCL_RET_OK) {
          for (size_t i = 0; i < nat.names.size; ++i) {
            if (nat.names.data[i] == name) {
              host_node_name = n;
              break;
            }
          }
          rcl_ret_t fini_ret = rcl_names_and_types_fini(&nat);
          (void)fini_ret;
        }
        if (!host_node_name.empty()) break;
      } else if (mode == MODE_SERVICE) {
        auto srvs = graph->get_service_names_and_types_by_node(bare_name, ns);
        auto it = srvs.find(name);
        if (it != srvs.end() && !it->second.empty()) {
          host_node_name = n;
          break;
        }
      } else {
        auto pubs = graph->get_publisher_names_and_types_by_node(bare_name, ns);
        if (pubs.find(name) != pubs.end()) {
          host_node_name = n;
          break;
        }
        auto subs =
            graph->get_subscriber_names_and_types_by_node(bare_name, ns);
        if (subs.find(name) != subs.end()) {
          host_node_name = n;
          break;
        }
      }
    }
    if (host_node_name.empty()) {
      SpinNodeSome(client_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // 3. Query the remote node's GetTypeDescription service using the hash.
  if (!host_node_name.empty()) {
    const std::string tds_path = host_node_name + "/get_type_description";
    auto client = client_node->create_client<
        type_description_interfaces::srv::GetTypeDescription>(tds_path);

    if (client->wait_for_service(std::chrono::seconds(2))) {
      auto req = std::make_shared<
          type_description_interfaces::srv::GetTypeDescription::Request>();
      req->type_name = type;
      req->type_hash = genuine_hash;
      req->include_type_sources = false;

      auto future = client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(client_node, future,
                                             std::chrono::seconds(3)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        auto resp = future.get();
        if (resp->successful) {
          if (mode == MODE_ACTION || mode == MODE_SERVICE) {
            const char* req_key =
                (mode == MODE_ACTION) ? "goal" : "request_message";
            const char* resp_key =
                (mode == MODE_ACTION) ? "result" : "response_message";
            std::string req_type_name;
            std::string resp_type_name;

            for (const auto& f :
                 resp->type_description.type_description.fields) {
              if (f.name == req_key) {
                req_type_name = f.type.nested_type_name;
              } else if (f.name == resp_key) {
                resp_type_name = f.type.nested_type_name;
              }
            }

            for (const auto& nested :
                 resp->type_description.referenced_type_descriptions) {
              if (!req_type_name.empty() && nested.type_name == req_type_name) {
                out_record.request_fields.clear();
                for (const auto& f : nested.fields) {
                  out_record.request_fields.push_back(
                      aquarium_gateway::FormatFieldType(f.type) + " " + f.name);
                }
              } else if (!resp_type_name.empty() &&
                         nested.type_name == resp_type_name) {
                out_record.response_fields.clear();
                for (const auto& f : nested.fields) {
                  out_record.response_fields.push_back(
                      aquarium_gateway::FormatFieldType(f.type) + " " + f.name);
                }
              }
            }

            if (out_record.request_fields.empty()) {
              out_record.request_fields = {
                  (mode == MODE_ACTION)
                      ? "(None) - Empty goal message payload"
                      : "(None) - Empty request message payload"};
            }
            if (out_record.response_fields.empty()) {
              out_record.response_fields = {
                  (mode == MODE_ACTION)
                      ? "(None) - Empty result message payload"
                      : "(None) - Empty response message payload"};
            }
            return true;
          }

          out_record.response_fields.clear();
          for (const auto& f : resp->type_description.type_description.fields) {
            out_record.response_fields.push_back(
                aquarium_gateway::FormatFieldType(f.type) + " " + f.name);
          }
          if (out_record.response_fields.empty()) {
            out_record.response_fields = {"(None) - Empty message payload"};
          }
          return true;
        }
      }
    }
  }

  // Fallback for actions: inspect local C typesupport if TDS RPC is offline.
  if (mode == MODE_ACTION) {
    try {
      auto ts_lib =
          rclcpp::get_typesupport_library(type, "rosidl_typesupport_c");
      if (ts_lib) {
        std::string pkg;
        std::string mod;
        std::string typ;
        std::tie(pkg, mod, typ) = rclcpp::extract_type_identifier(type);
        if (mod.empty()) mod = "action";
        const std::string symbol_name =
            "rosidl_typesupport_c__get_action_type_support_handle__" + pkg +
            "__" + mod + "__" + typ;
        auto get_ts =
            reinterpret_cast<const rosidl_action_type_support_t* (*)()>(
                ts_lib->get_symbol(symbol_name));
        if (get_ts != nullptr) {
          const rosidl_action_type_support_t* ts = get_ts();
          if (ts != nullptr && ts->get_type_description_func != nullptr) {
            const rosidl_runtime_c__type_description__TypeDescription* desc =
                ts->get_type_description_func(ts);
            if (desc != nullptr) {
              for (size_t r = 0; r < desc->referenced_type_descriptions.size;
                   ++r) {
                const auto& ref = desc->referenced_type_descriptions.data[r];
                const std::string ref_name =
                    ref.type_name.data ? ref.type_name.data : "";
                std::vector<std::string>* target_vec = nullptr;
                if (ref_name.find("_Goal") != std::string::npos) {
                  target_vec = &out_record.request_fields;
                } else if (ref_name.find("_Result") != std::string::npos) {
                  target_vec = &out_record.response_fields;
                }
                if (target_vec != nullptr) {
                  target_vec->clear();
                  for (size_t f = 0; f < ref.fields.size; ++f) {
                    const auto& field = ref.fields.data[f];
                    type_description_interfaces::msg::FieldType ft;
                    ft.type_id = field.type.type_id;
                    ft.capacity = field.type.capacity;
                    ft.string_capacity = field.type.string_capacity;
                    if (field.type.nested_type_name.data) {
                      ft.nested_type_name = field.type.nested_type_name.data;
                    }
                    const std::string f_name =
                        field.name.data ? field.name.data : "";
                    target_vec->push_back(
                        aquarium_gateway::FormatFieldType(ft) + " " + f_name);
                  }
                }
              }
              if (out_record.request_fields.empty()) {
                out_record.request_fields = {
                    "(None) - Empty goal message payload"};
              }
              if (out_record.response_fields.empty()) {
                out_record.response_fields = {
                    "(None) - Empty result message payload"};
              }
              return true;
            }
          }
        }
      }
    } catch (...) {
    }
  }

  const std::string err_msg =
      "(None) - Discovery failed. Active host node missing or TDS offline.";
  if (mode == MODE_SERVICE || mode == MODE_ACTION) {
    out_record.request_fields = {err_msg};
  }
  out_record.response_fields = {err_msg};
  return false;
}

void scan_graph(const std::shared_ptr<rclcpp::Node>& node, TerminalUi& ui,
                bool show_system_services = false) {
  auto graph = node->get_node_graph_interface();

  // 1. Scan ROS 2 services.
  std::vector<InterfaceRecord> service_records;
  for (const auto& [name, types] : graph->get_service_names_and_types()) {
    if (!show_system_services &&
        (name.find("/_action/") != std::string::npos ||
         name.find("/get_type_description") != std::string::npos ||
         name.find("/get_parameters") != std::string::npos ||
         name.find("/set_parameters") != std::string::npos ||
         name.find("/set_parameters_atomically") != std::string::npos ||
         name.find("/describe_parameters") != std::string::npos ||
         name.find("/list_parameters") != std::string::npos)) {
      continue;
    }
    if (types.empty()) continue;

    InterfaceRecord r;
    r.name = name;
    r.type = types[0];
    r.is_service = true;
    r.is_action = false;
    service_records.push_back(r);
  }
  ui.update_interfaces(service_records, MODE_SERVICE);

  // 2. Scan ROS 2 topics.
  std::vector<InterfaceRecord> topic_records;
  for (const auto& [name, types] : graph->get_topic_names_and_types()) {
    if (!show_system_services &&
        (name.find("/_action/") != std::string::npos ||
         name.find("/rosout") != std::string::npos ||
         name.find("/parameter_events") != std::string::npos)) {
      continue;
    }
    if (types.empty()) continue;

    InterfaceRecord r;
    r.name = name;
    r.type = types[0];
    r.is_service = false;
    r.is_action = false;
    topic_records.push_back(r);
  }
  ui.update_interfaces(topic_records, MODE_TOPIC);

  // 3. Scan ROS 2 actions.
  std::vector<InterfaceRecord> action_records;
  rcl_names_and_types_t action_nat = rcl_get_zero_initialized_names_and_types();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto* rcl_node = node->get_node_base_interface()->get_rcl_node_handle();
  rcl_ret_t ret =
      rcl_action_get_names_and_types(rcl_node, &allocator, &action_nat);
  if (ret == RCL_RET_OK) {
    for (size_t i = 0; i < action_nat.names.size; ++i) {
      if (action_nat.types[i].size == 0) continue;
      InterfaceRecord r;
      r.name = action_nat.names.data[i];
      r.type = action_nat.types[i].data[0];
      r.is_service = false;
      r.is_action = true;
      action_records.push_back(r);
    }
    rcl_ret_t fini_ret = rcl_names_and_types_fini(&action_nat);
    (void)fini_ret;
  }
  ui.update_interfaces(action_records, MODE_ACTION);
}

static bool parse_selection_file(const std::string& filename,
                                 std::vector<std::string>& out_services,
                                 std::vector<std::string>& out_topics,
                                 std::vector<std::string>& out_actions) {
  try {
    YAML::Node config = YAML::LoadFile(filename);
    if (config["exposed_services"]) {
      for (const auto& item : config["exposed_services"]) {
        out_services.push_back(item.as<std::string>());
      }
    }
    if (config["exposed_topics"]) {
      for (const auto& item : config["exposed_topics"]) {
        out_topics.push_back(item.as<std::string>());
      }
    }
    if (config["exposed_actions"]) {
      for (const auto& item : config["exposed_actions"]) {
        out_actions.push_back(item.as<std::string>());
      }
    }
    if (!config["exposed_services"] && !config["exposed_topics"] &&
        !config["exposed_actions"]) {
      std::cerr << "SELECTION ERROR: Missing 'exposed_services', "
                   "'exposed_topics', and 'exposed_actions' in "
                << filename << std::endl;
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "SELECTION ERROR: Failed to parse selection file: " << e.what()
              << std::endl;
    return false;
  }
  return true;
}

static bool FindTargetIndices(const std::shared_ptr<rclcpp::Node>& node,
                              const TerminalUi& ui, UiMode mode,
                              const char* kind_label,
                              const std::vector<std::string>& targets,
                              std::vector<size_t>& out_indices) {
  out_indices.clear();
  bool all_found = true;
  const size_t count = ui.get_interfaces_count(mode);
  for (const auto& target_name : targets) {
    bool found = false;
    for (size_t i = 0; i < count; ++i) {
      if (ui.get_interface_at(i, mode).name == target_name) {
        found = true;
        out_indices.push_back(i);
        break;
      }
    }
    if (!found) {
      all_found = false;
      RCLCPP_WARN(node->get_logger(), "Target %s '%s' not yet found on graph.",
                  kind_label, target_name.c_str());
    }
  }
  return all_found;
}

static bool IntrospectSelectedTargets(const std::shared_ptr<rclcpp::Node>& node,
                                      TerminalUi& ui, UiMode mode,
                                      const char* kind_label,
                                      const std::vector<size_t>& indices) {
  bool ok = true;
  for (size_t idx : indices) {
    const auto rec_item = ui.get_interface_at(idx, mode);
    RCLCPP_INFO(node->get_logger(), "Introspecting %s '%s'...", kind_label,
                rec_item.name.c_str());
    ui.select_interface(idx, mode, true);

    IntrospectionRecord record;
    if (fetch_dynamic_introspection(node, rec_item.name, rec_item.type, mode,
                                    record)) {
      RCLCPP_INFO(node->get_logger(), "Resolved type hash: %s",
                  record.type_hash.c_str());
      ui.update_introspection_details(record);
      ui.set_interface_introspection(idx, mode, record);
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to resolve type hash for '%s'.",
                  rec_item.name.c_str());
      ok = false;
    }
  }
  return ok;
}

class ScopedBackgroundProcess {
 public:
  ScopedBackgroundProcess() = default;
  ~ScopedBackgroundProcess() { stop(); }

  ScopedBackgroundProcess(const ScopedBackgroundProcess&) = delete;
  ScopedBackgroundProcess& operator=(const ScopedBackgroundProcess&) = delete;

  bool start(const rclcpp::Logger& logger, const TargetSpec& spec) {
    if (spec.package.empty() || spec.name.empty()) {
      return false;
    }

    const std::filesystem::path log_dir =
        std::filesystem::current_path() / "log" / "aquarium";
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    const std::filesystem::path log_file_path =
        log_dir / "background_launch.log";

    const std::string subverb = (spec.type == "node") ? "run" : "launch";
    RCLCPP_INFO(logger, "Launching background target: ros2 %s %s %s",
                subverb.c_str(), spec.package.c_str(), spec.name.c_str());

    pid_ = fork();
    if (pid_ < 0) {
      RCLCPP_ERROR(logger, "Failed to fork background target process.");
      return false;
    }

    if (pid_ == 0) {
      setpgid(0, 0);
      setenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp", 0);

      int dev_null = open("/dev/null", O_RDONLY);
      if (dev_null >= 0) {
        dup2(dev_null, STDIN_FILENO);
        close(dev_null);
      }

      int log_fd =
          open(log_file_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (log_fd < 0) {
        log_fd = open("/dev/null", O_WRONLY);
      }
      if (log_fd >= 0) {
        dup2(log_fd, STDOUT_FILENO);
        dup2(log_fd, STDERR_FILENO);
        close(log_fd);
      }

      std::vector<std::string> cmd_args = {"ros2", subverb, spec.package,
                                           spec.name};
      for (const auto& arg : spec.arguments) {
        cmd_args.push_back(arg);
      }

      std::vector<char*> argv_ptrs;
      argv_ptrs.reserve(cmd_args.size() + 1);
      for (auto& arg : cmd_args) {
        argv_ptrs.push_back(arg.data());
      }
      argv_ptrs.push_back(nullptr);

      execvp("ros2", argv_ptrs.data());
      _exit(1);
    }

    setpgid(pid_, pid_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    int status = 0;
    const pid_t wait_ret = waitpid(pid_, &status, WNOHANG);
    if (wait_ret == pid_) {
      RCLCPP_ERROR(logger,
                   "Background ros2 %s exited immediately. Check %s for logs.",
                   subverb.c_str(), log_file_path.c_str());
      std::ifstream log_in(log_file_path);
      if (log_in.is_open()) {
        std::cerr << log_in.rdbuf();
      }
      pid_ = -1;
      return false;
    }
    return true;
  }

  void stop() {
    if (pid_ <= 0) {
      return;
    }
    kill(-pid_, SIGINT);
    const auto start_wait = std::chrono::steady_clock::now();
    bool exited = false;
    while ((std::chrono::steady_clock::now() - start_wait) <
           std::chrono::seconds(5)) {
      int status = 0;
      const pid_t ret = waitpid(pid_, &status, WNOHANG);
      if (ret == pid_ || (ret < 0 && errno == ECHILD)) {
        exited = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!exited) {
      kill(-pid_, SIGKILL);
      int status = 0;
      waitpid(pid_, &status, 0);
    }
    pid_ = -1;
  }

 private:
  pid_t pid_ = -1;
};

}  // namespace aquarium_setup_assistant

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  aquarium_setup_assistant::TargetSpec cli_target;
  const bool has_cli_target =
      aquarium_setup_assistant::parse_cli_target_args(non_ros_args, cli_target);

  auto node = std::make_shared<rclcpp::Node>("setup_wizard");

  node->declare_parameter<std::string>("target_type", cli_target.type);
  node->declare_parameter<std::string>("target_package", cli_target.package);
  node->declare_parameter<std::string>("target_name", cli_target.name);
  node->declare_parameter<std::vector<std::string>>("target_arguments",
                                                    cli_target.arguments);

  node->declare_parameter<std::string>("auto_select_service", "");
  node->declare_parameter<std::string>("auto_select_topic", "");
  node->declare_parameter<std::string>("auto_select_action", "");
  node->declare_parameter<std::string>("selection_file",
                                       cli_target.selection_file);

  std::string auto_select_service;
  node->get_parameter("auto_select_service", auto_select_service);

  std::string auto_select_topic;
  node->get_parameter("auto_select_topic", auto_select_topic);

  std::string auto_select_action;
  node->get_parameter("auto_select_action", auto_select_action);

  std::string selection_file;
  node->get_parameter("selection_file", selection_file);

  aquarium_setup_assistant::ScopedBackgroundProcess bg_process;
  if (has_cli_target) {
    if (!bg_process.start(node->get_logger(), cli_target)) {
      rclcpp::shutdown();
      return 1;
    }
  }

  std::vector<std::string> target_services;
  std::vector<std::string> target_topics;
  std::vector<std::string> target_actions;

  if (!auto_select_service.empty()) {
    target_services.push_back(auto_select_service);
  }
  if (!auto_select_topic.empty()) {
    target_topics.push_back(auto_select_topic);
  }
  if (!auto_select_action.empty()) {
    target_actions.push_back(auto_select_action);
  }

  if (!selection_file.empty()) {
    if (!aquarium_setup_assistant::parse_selection_file(
            selection_file, target_services, target_topics, target_actions)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to parse selection file: %s",
                   selection_file.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  const bool has_headless_targets = !target_services.empty() ||
                                    !target_topics.empty() ||
                                    !target_actions.empty();

  if (has_headless_targets) {
    RCLCPP_INFO(node->get_logger(), "Launching headless setup assistant...");
    for (const auto& s : target_services) {
      RCLCPP_INFO(node->get_logger(), "  Target service: '%s'", s.c_str());
    }
    for (const auto& t : target_topics) {
      RCLCPP_INFO(node->get_logger(), "  Target topic: '%s'", t.c_str());
    }
    for (const auto& a : target_actions) {
      RCLCPP_INFO(node->get_logger(), "  Target action: '%s'", a.c_str());
    }

    aquarium_setup_assistant::TerminalUi ui(true);
    bool success = false;
    constexpr int kMaxRetries = 30;

    for (int retry = 0; retry < kMaxRetries && rclcpp::ok(); ++retry) {
      RCLCPP_INFO(node->get_logger(), "Scanning graph (attempt %d/%d)...",
                  retry + 1, kMaxRetries);
      aquarium_setup_assistant::scan_graph(node, ui, false);

      std::vector<size_t> srv_indices;
      std::vector<size_t> topic_indices;
      std::vector<size_t> action_indices;

      const bool srvs_ok = aquarium_setup_assistant::FindTargetIndices(
          node, ui, aquarium_setup_assistant::MODE_SERVICE, "service",
          target_services, srv_indices);
      const bool topics_ok = aquarium_setup_assistant::FindTargetIndices(
          node, ui, aquarium_setup_assistant::MODE_TOPIC, "topic",
          target_topics, topic_indices);
      const bool actions_ok = aquarium_setup_assistant::FindTargetIndices(
          node, ui, aquarium_setup_assistant::MODE_ACTION, "action",
          target_actions, action_indices);

      if (srvs_ok && topics_ok && actions_ok) {
        RCLCPP_INFO(
            node->get_logger(),
            "All targets found. Introspecting and generating config...");

        bool introspection_ok = true;
        if (!aquarium_setup_assistant::IntrospectSelectedTargets(
                node, ui, aquarium_setup_assistant::MODE_SERVICE, "service",
                srv_indices)) {
          introspection_ok = false;
        }
        if (!aquarium_setup_assistant::IntrospectSelectedTargets(
                node, ui, aquarium_setup_assistant::MODE_TOPIC, "topic",
                topic_indices)) {
          introspection_ok = false;
        }
        if (!aquarium_setup_assistant::IntrospectSelectedTargets(
                node, ui, aquarium_setup_assistant::MODE_ACTION, "action",
                action_indices)) {
          introspection_ok = false;
        }

        if (introspection_ok) {
          auto selected = ui.get_selected_interfaces();
          if (aquarium_setup_assistant::write_config_yaml(node, selected)) {
            RCLCPP_INFO(node->get_logger(),
                        "Successfully wrote configuration to aquarium.yaml");
            success = true;
          } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to write configuration.");
          }
        } else {
          RCLCPP_ERROR(node->get_logger(),
                       "Headless introspection failed for one or more targets. "
                       "Aborting config generation.");
        }
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      aquarium_setup_assistant::SpinNodeSome(node);
    }

    if (!success) {
      RCLCPP_ERROR(node->get_logger(),
                   "Headless configuration generation failed or timed out.");
    }
    bg_process.stop();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Launching interactive setup assistant terminal UI...");

  aquarium_setup_assistant::TerminalUi ui;
  ui.enable_raw_mode();

  bool show_system_services = false;
  aquarium_setup_assistant::scan_graph(node, ui, show_system_services);

  while (rclcpp::ok() && ui.is_running()) {
    ui.render();

    aquarium_setup_assistant::KeyPress key = ui.read_input();
    if (key == aquarium_setup_assistant::KEY_TIMEOUT) {
      aquarium_setup_assistant::scan_graph(node, ui, show_system_services);
    } else if (key == aquarium_setup_assistant::KEY_TOGGLE_SYSTEM) {
      show_system_services = !show_system_services;
      ui.set_status_message(show_system_services ? "Showing system services."
                                                 : "Hiding system services.");
      ui.render();
      aquarium_setup_assistant::scan_graph(node, ui, show_system_services);
    } else if (key == aquarium_setup_assistant::KEY_WRITE_CONFIG) {
      if (ui.get_active_state() == aquarium_setup_assistant::STATE_CATALOG) {
        auto selected = ui.get_selected_interfaces();
        if (selected.empty()) {
          ui.set_status_message(
              "Select at least one interface before writing configuration.");
        } else {
          ui.set_status_message("Introspecting selected interfaces...");
          ui.render();

          bool all_introspection_ok = true;
          for (auto& inf : selected) {
            if (!inf.has_introspection) {
              ui.set_status_message("Introspecting " + inf.name + "...");
              ui.render();

              aquarium_setup_assistant::IntrospectionRecord record;
              aquarium_setup_assistant::UiMode inf_mode =
                  inf.is_action
                      ? aquarium_setup_assistant::MODE_ACTION
                      : (inf.is_service ? aquarium_setup_assistant::MODE_SERVICE
                                        : aquarium_setup_assistant::MODE_TOPIC);
              if (aquarium_setup_assistant::fetch_dynamic_introspection(
                      node, inf.name, inf.type, inf_mode, record)) {
                inf.has_introspection = true;
                inf.type_hash = record.type_hash;
                inf.request_fields = record.request_fields;
                inf.response_fields = record.response_fields;
              } else {
                RCLCPP_WARN(node->get_logger(),
                            "Failed to introspect '%s' during generation.",
                            inf.name.c_str());
                all_introspection_ok = false;
              }
            }
          }

          if (!all_introspection_ok) {
            ui.set_status_message(
                "Error: introspection failed for one or more targets.");
            ui.render();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
          }

          ui.set_status_message("Writing configuration to aquarium.yaml...");
          ui.render();
          if (aquarium_setup_assistant::write_config_yaml(node, selected)) {
            ui.set_status_message(
                "Wrote aquarium.yaml successfully. Exiting...");
            ui.render();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ui.process_key(aquarium_setup_assistant::KEY_QUIT);
          } else {
            ui.set_status_message("Error: failed to write aquarium.yaml.");
          }
        }
      }
    } else if (key == aquarium_setup_assistant::KEY_ESCAPE) {
      ui.process_key(key);
      aquarium_setup_assistant::scan_graph(node, ui, show_system_services);
    } else if (key == aquarium_setup_assistant::KEY_ENTER) {
      const aquarium_setup_assistant::UiMode active_mode = ui.get_active_mode();
      const size_t highlight_idx = ui.get_highlight_index();
      const size_t count = ui.get_interfaces_count(active_mode);

      if (ui.get_active_state() == aquarium_setup_assistant::STATE_CATALOG &&
          count > 0) {
        auto inf = ui.get_interface_at(highlight_idx, active_mode);
        ui.set_status_message(
            "Querying Type Description Service via Zenoh RPC...");
        ui.render();

        aquarium_setup_assistant::IntrospectionRecord record;
        if (aquarium_setup_assistant::fetch_dynamic_introspection(
                node, inf.name, inf.type, active_mode, record)) {
          ui.set_status_message(
              "Resolved type description and hash dynamically.");
          ui.set_interface_introspection(highlight_idx, active_mode, record);
        } else {
          ui.set_status_message(
              "Error: remote Type Description Service timed out.");
        }
        ui.update_introspection_details(record);
      }
      ui.process_key(key);
    } else {
      ui.process_key(key);
    }

    aquarium_setup_assistant::SpinNodeSome(node);
  }

  ui.disable_raw_mode();
  bg_process.stop();

  RCLCPP_INFO(node->get_logger(), "Setup assistant shutdown completed.");
  rclcpp::shutdown();
  return 0;
}
