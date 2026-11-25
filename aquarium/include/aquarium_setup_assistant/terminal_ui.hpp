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

#ifndef AQUARIUM_SETUP_ASSISTANT__TERMINAL_UI_HPP_
#define AQUARIUM_SETUP_ASSISTANT__TERMINAL_UI_HPP_

#include <termios.h>

#include <cstddef>
#include <string>
#include <vector>

namespace aquarium_setup_assistant {

// Discovered ROS 2 graph endpoint (service, topic, or action).
struct InterfaceRecord {
  std::string name;
  std::string type;
  bool is_service = true;
  bool is_action = false;
  bool is_selected = false;

  bool has_introspection = false;
  std::string type_hash;
  std::vector<std::string> request_fields;
  std::vector<std::string> response_fields;

  InterfaceRecord() = default;
  InterfaceRecord(const std::string& n, const std::string& t,
                  bool service = true, bool selected = false,
                  bool intro = false, const std::string& hash = "",
                  const std::vector<std::string>& req = {},
                  const std::vector<std::string>& resp = {},
                  bool action = false)
      : name(n),
        type(t),
        is_service(service),
        is_action(action),
        is_selected(selected),
        has_introspection(intro),
        type_hash(hash),
        request_fields(req),
        response_fields(resp) {}
};

// Decoupled model holder for dynamic interface schema details.
struct IntrospectionRecord {
  std::string interface_name;
  std::string type_name;
  std::string type_hash;
  std::vector<std::string> request_fields;
  std::vector<std::string> response_fields;
};

enum UiState { STATE_CATALOG, STATE_DETAIL };
enum UiMode { MODE_TOPIC, MODE_SERVICE, MODE_ACTION };

enum KeyPress {
  KEY_UP,
  KEY_DOWN,
  KEY_SPACE,
  KEY_ENTER,
  KEY_ESCAPE,
  KEY_WRITE_CONFIG,
  KEY_TOGGLE_SYSTEM,
  KEY_SWITCH_TOPIC,
  KEY_SWITCH_SERVICE,
  KEY_SWITCH_ACTION,
  KEY_TIMEOUT,
  KEY_QUIT,
  KEY_UNKNOWN
};

// Zero-dependency raw ANSI terminal UI view-model for interface selection.
class TerminalUi {
 public:
  TerminalUi();
  explicit TerminalUi(bool is_test_mode);
  ~TerminalUi();

  void enable_raw_mode();
  void disable_raw_mode();

  void update_interfaces(const std::vector<InterfaceRecord>& new_interfaces,
                         bool is_service);
  void update_interfaces(const std::vector<InterfaceRecord>& new_interfaces,
                         UiMode mode);

  void update_introspection_details(const IntrospectionRecord& record);

  void set_interface_introspection(size_t index, bool is_service,
                                   const IntrospectionRecord& record);
  void set_interface_introspection(size_t index, UiMode mode,
                                   const IntrospectionRecord& record);

  void select_interface(size_t index, bool is_service, bool selected = true);
  void select_interface(size_t index, UiMode mode, bool selected = true);

  bool is_running() const { return is_running_; }

  void set_status_message(const std::string& msg) { status_msg_ = msg; }

  void render();

  KeyPress read_input();

  void process_key(KeyPress key);

  std::vector<InterfaceRecord> get_selected_interfaces() const;

  UiState get_active_state() const { return active_state_; }

  UiMode get_active_mode() const { return active_mode_; }

  size_t get_highlight_index() const;

  InterfaceRecord get_interface_at(size_t index, bool is_service) const;
  InterfaceRecord get_interface_at(size_t index, UiMode mode) const;

  size_t get_interfaces_count(bool is_service) const;
  size_t get_interfaces_count(UiMode mode) const;

  // Headless automated test harness methods.
  void queue_test_key(KeyPress key);
  std::string get_raw_screen() const { return screen_output_; }
  std::string get_clean_screen() const;

 private:
  void clear_screen();
  void move_cursor_to_home();
  void hide_cursor();
  void show_cursor();

  struct termios original_termios_;
  std::vector<InterfaceRecord> services_;
  std::vector<InterfaceRecord> topics_;
  std::vector<InterfaceRecord> actions_;
  size_t service_highlight_index_;
  size_t topic_highlight_index_;
  size_t action_highlight_index_;
  UiMode active_mode_;
  bool is_running_;
  std::string status_msg_;
  UiState active_state_;
  IntrospectionRecord detail_record_;

  bool is_test_mode_;
  std::vector<KeyPress> test_key_queue_;
  std::string screen_output_;
};

}  // namespace aquarium_setup_assistant

#endif  // AQUARIUM_SETUP_ASSISTANT__TERMINAL_UI_HPP_
