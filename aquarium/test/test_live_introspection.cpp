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

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "aquarium_gateway/serialization.hpp"
#include "aquarium_setup_assistant/terminal_ui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "type_description_interfaces/srv/get_type_description.hpp"
#include "zenoh.hxx"

namespace aquarium_setup_assistant {

static void SpinNodeSome(const std::shared_ptr<rclcpp::Node>& node) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();
}

static std::string discover_service_type_hash(
    const std::shared_ptr<rclcpp::Node>& logger_node,
    const std::string& service_name) {
  std::string resolved_hash;
  std::mutex match_mtx;
  std::condition_variable match_cv;
  bool match_found = false;

  std::string match_token = service_name;
  std::replace(match_token.begin(), match_token.end(), '/', '%');
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
          if (key_str.find(strict_search) != std::string::npos &&
              key_str.find("/SS/") != std::string::npos) {
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
    const std::shared_ptr<rclcpp::Node>& client_node,
    const std::string& service_name, const std::string& service_type,
    IntrospectionRecord& out_record) {
  out_record.interface_name = service_name;
  out_record.type_name = service_type;
  out_record.request_fields.clear();
  out_record.response_fields.clear();
  out_record.type_hash =
      "RIHS01_0000000000000000000000000000000000000000000000000000000000000000 "
      "(NOT RETRIEVED)";

  const std::string genuine_hash =
      discover_service_type_hash(client_node, service_name);
  if (genuine_hash.empty()) {
    RCLCPP_ERROR(client_node->get_logger(),
                 "TDS query failed: unable to resolve dynamic type hash over "
                 "Zenoh liveliness.");
    return false;
  }
  out_record.type_hash = genuine_hash;

  std::string host_node_name;
  const auto start_search = std::chrono::steady_clock::now();
  const auto search_timeout = std::chrono::seconds(5);

  while ((std::chrono::steady_clock::now() - start_search) < search_timeout &&
         host_node_name.empty()) {
    for (const auto& n : client_node->get_node_names()) {
      const size_t last_slash = n.rfind('/');
      std::string bare_name = n;
      std::string ns = "/";
      if (last_slash != std::string::npos) {
        bare_name = n.substr(last_slash + 1);
        ns = n.substr(0, last_slash);
        if (ns.empty()) ns = "/";
      }

      auto srvs =
          client_node->get_service_names_and_types_by_node(bare_name, ns);
      auto it = srvs.find(service_name);
      if (it != srvs.end() && !it->second.empty()) {
        host_node_name = n;
        break;
      }
    }
    if (host_node_name.empty()) {
      SpinNodeSome(client_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  if (host_node_name.empty()) {
    RCLCPP_ERROR(client_node->get_logger(),
                 "Discovery failed: active service node missing on graph.");
    return false;
  }

  const std::string tds_path = host_node_name + "/get_type_description";
  auto client =
      client_node
          ->create_client<type_description_interfaces::srv::GetTypeDescription>(
              tds_path);
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(client_node->get_logger(), "TDS service offline: %s",
                 tds_path.c_str());
    return false;
  }

  auto req = std::make_shared<
      type_description_interfaces::srv::GetTypeDescription::Request>();
  req->type_name = service_type;
  req->type_hash = genuine_hash;
  req->include_type_sources = false;

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(client_node, future,
                                         std::chrono::seconds(3)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(client_node->get_logger(), "TDS remote RPC call timed out.");
    return false;
  }

  auto resp = future.get();
  if (!resp->successful) {
    RCLCPP_ERROR(client_node->get_logger(), "TDS remote call failed: %s",
                 resp->failure_reason.c_str());
    return false;
  }

  std::string req_type_name;
  std::string resp_type_name;
  for (const auto& f : resp->type_description.type_description.fields) {
    if (f.name == "request_message") {
      req_type_name = f.type.nested_type_name;
    } else if (f.name == "response_message") {
      resp_type_name = f.type.nested_type_name;
    }
  }
  if (req_type_name.empty() || resp_type_name.empty()) {
    return false;
  }

  bool req_found = false;
  bool resp_found = false;
  for (const auto& nested :
       resp->type_description.referenced_type_descriptions) {
    if (nested.type_name == req_type_name) {
      out_record.request_fields.clear();
      for (const auto& f : nested.fields) {
        out_record.request_fields.push_back(
            aquarium_gateway::FormatFieldType(f.type) + " " + f.name);
      }
      req_found = true;
    } else if (nested.type_name == resp_type_name) {
      out_record.response_fields.clear();
      for (const auto& f : nested.fields) {
        out_record.response_fields.push_back(
            aquarium_gateway::FormatFieldType(f.type) + " " + f.name);
      }
      resp_found = true;
    }
  }

  if (!req_found || !resp_found) {
    return false;
  }

  if (out_record.request_fields.empty()) {
    out_record.request_fields = {"(None) - Empty request message payload"};
  }
  if (out_record.response_fields.empty()) {
    out_record.response_fields = {"(None) - Empty response message payload"};
  }
  return true;
}

namespace test {

TEST(LiveIntrospectionTest, VerifyDemoServicesDiscoveredOverRealZenohGraph) {
  auto node = std::make_shared<rclcpp::Node>("test_live_introspection_node");

  bool is_discovered = false;
  std::map<std::string, std::vector<std::string>> discovered_services;

  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(10);

  RCLCPP_INFO(node->get_logger(), "Scanning Zenoh network graph...");

  while ((std::chrono::steady_clock::now() - start_time) < timeout &&
         !is_discovered) {
    SpinNodeSome(node);
    discovered_services = node->get_service_names_and_types();

    auto it_demo = discovered_services.find("/add_two_ints");
    auto it_lc = discovered_services.find("/lc_talker/get_state");
    if (it_demo != discovered_services.end() && !it_demo->second.empty() &&
        it_lc != discovered_services.end() && !it_lc->second.empty()) {
      is_discovered = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  EXPECT_TRUE(is_discovered)
      << "Timeout expired before target services were discovered.";
  if (!is_discovered) return;

  std::vector<InterfaceRecord> records;
  for (const auto& [name, types] : discovered_services) {
    if (types.empty()) continue;
    if (name.find("/get_loggers") != std::string::npos ||
        name.find("/set_parameters") != std::string::npos ||
        name.find("/get_parameters") != std::string::npos ||
        name.find("/list_parameters") != std::string::npos ||
        name.find("/describe_parameters") != std::string::npos ||
        name.find("/get_parameter_types") != std::string::npos ||
        name.find("/get_type_description") != std::string::npos ||
        name.find("/test_live_introspection_node") != std::string::npos) {
      continue;
    }

    InterfaceRecord r;
    r.name = name;
    r.type = types[0];
    r.is_service = true;
    r.is_selected = false;
    records.push_back(r);
  }

  TerminalUi ui(true);
  ui.update_interfaces(records, true);
  ui.process_key(KEY_SWITCH_SERVICE);
  ui.render();

  const std::string clean_screen = ui.get_clean_screen();
  EXPECT_GE(records.size(), 2u);
  EXPECT_NE(clean_screen.find("[ ] /add_two_ints"), std::string::npos);
  EXPECT_NE(clean_screen.find("example_interfaces/srv/AddTwoInts"),
            std::string::npos);
  EXPECT_NE(clean_screen.find("[ ] /lc_talker/get_state"), std::string::npos);
  EXPECT_NE(clean_screen.find("lifecycle_msgs/srv/GetState"),
            std::string::npos);

  IntrospectionRecord detail_rec;
  const bool tds_success = fetch_dynamic_introspection(
      node, "/add_two_ints", "example_interfaces/srv/AddTwoInts", detail_rec);
  EXPECT_TRUE(tds_success)
      << "Remote node GetTypeDescription service call timed out or failed.";

  if (tds_success) {
    EXPECT_EQ(
        detail_rec.type_hash,
        "RIHS01_"
        "e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a");
    ASSERT_EQ(detail_rec.request_fields.size(), 2u);
    EXPECT_EQ(detail_rec.request_fields[0], "int64 a");
    EXPECT_EQ(detail_rec.request_fields[1], "int64 b");
    ASSERT_EQ(detail_rec.response_fields.size(), 1u);
    EXPECT_EQ(detail_rec.response_fields[0], "int64 sum");

    ui.update_introspection_details(detail_rec);
    ui.process_key(KEY_ENTER);
    EXPECT_EQ(ui.get_active_state(), STATE_DETAIL);

    ui.render();
    const std::string detail_screen = ui.get_clean_screen();
    EXPECT_NE(detail_screen.find("Interface Details"), std::string::npos);
    EXPECT_NE(detail_screen.find("Name:  /add_two_ints"), std::string::npos);
    EXPECT_NE(
        detail_screen.find(
            "Type ID Hash:  "
            "RIHS01_"
            "e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a"),
        std::string::npos);
    EXPECT_NE(detail_screen.find("- int64 a"), std::string::npos);
    EXPECT_NE(detail_screen.find("- int64 sum"), std::string::npos);

    ui.process_key(KEY_ESCAPE);
    EXPECT_EQ(ui.get_active_state(), STATE_CATALOG);
  }
}

TEST(LiveIntrospectionTest,
     VerifyParameterServiceDiscoveredOverRealZenohGraph) {
  auto node =
      std::make_shared<rclcpp::Node>("test_live_params_introspection_node");

  bool is_discovered = false;
  std::map<std::string, std::vector<std::string>> discovered_services;

  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(10);

  while ((std::chrono::steady_clock::now() - start_time) < timeout &&
         !is_discovered) {
    SpinNodeSome(node);
    discovered_services = node->get_service_names_and_types();

    auto it = discovered_services.find("/add_two_ints_server/get_parameters");
    if (it != discovered_services.end() && !it->second.empty()) {
      is_discovered = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  EXPECT_TRUE(is_discovered)
      << "Timeout expired before parameter service was discovered.";
  if (!is_discovered) return;

  std::vector<InterfaceRecord> records;
  for (const auto& [name, types] : discovered_services) {
    if (types.empty()) continue;
    if (name.find("/test_live_params_introspection_node") !=
        std::string::npos) {
      continue;
    }

    InterfaceRecord r;
    r.name = name;
    r.type = types[0];
    r.is_service = true;
    r.is_selected = false;
    records.push_back(r);
  }

  TerminalUi ui(true);
  ui.update_interfaces(records, true);
  ui.process_key(KEY_SWITCH_SERVICE);
  ui.render();

  const std::string clean_screen = ui.get_clean_screen();
  EXPECT_NE(clean_screen.find("[ ] /add_two_ints_server/get_parameters"),
            std::string::npos);
  EXPECT_NE(clean_screen.find("rcl_interfaces/srv/GetParameters"),
            std::string::npos);

  IntrospectionRecord param_rec;
  const bool param_tds_success = fetch_dynamic_introspection(
      node, "/add_two_ints_server/get_parameters",
      "rcl_interfaces/srv/GetParameters", param_rec);
  EXPECT_TRUE(param_tds_success)
      << "Remote node parameter TDS service call timed out or failed.";

  if (param_tds_success) {
    EXPECT_EQ(param_rec.type_hash.find("RIHS01_"), 0u);
    EXPECT_EQ(param_rec.type_hash.length(), 71u);
    ASSERT_EQ(param_rec.request_fields.size(), 1u);
    EXPECT_EQ(param_rec.request_fields[0], "string[] names");
    ASSERT_EQ(param_rec.response_fields.size(), 1u);
    EXPECT_EQ(param_rec.response_fields[0],
              "rcl_interfaces/msg/ParameterValue[] values");

    ui.update_introspection_details(param_rec);
    ui.process_key(KEY_ENTER);
    EXPECT_EQ(ui.get_active_state(), STATE_DETAIL);

    ui.render();
    const std::string param_detail = ui.get_clean_screen();
    EXPECT_NE(param_detail.find("Interface Details"), std::string::npos);
    EXPECT_NE(param_detail.find("Name:  /add_two_ints_server/get_parameters"),
              std::string::npos);
    EXPECT_NE(param_detail.find("Type:  rcl_interfaces/srv/GetParameters"),
              std::string::npos);
    EXPECT_NE(param_detail.find("- string[] names"), std::string::npos);
    EXPECT_NE(param_detail.find("- rcl_interfaces/msg/ParameterValue[] values"),
              std::string::npos);

    ui.process_key(KEY_ESCAPE);
    EXPECT_EQ(ui.get_active_state(), STATE_CATALOG);
  }
}

}  // namespace test
}  // namespace aquarium_setup_assistant

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
