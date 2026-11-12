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

#include "aquarium_setup_assistant/terminal_ui.hpp"

#include <poll.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace aquarium_setup_assistant {
namespace {

void WriteUiOutput(std::string& capture_buf, const char* fmt, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  capture_buf.append(buffer);
}

std::string GetSeparatorLine(char c, int length) {
  if (length <= 0) return "";
  return std::string(length, c);
}

}  // namespace

TerminalUi::TerminalUi()
    : service_highlight_index_(0),
      topic_highlight_index_(0),
      action_highlight_index_(0),
      active_mode_(MODE_TOPIC),
      is_running_(true),
      status_msg_("Discovery graph ready. Waiting for introspection run..."),
      active_state_(STATE_CATALOG),
      is_test_mode_(false) {
  tcgetattr(STDIN_FILENO, &original_termios_);
}

TerminalUi::TerminalUi(bool is_test_mode)
    : service_highlight_index_(0),
      topic_highlight_index_(0),
      action_highlight_index_(0),
      active_mode_(MODE_TOPIC),
      is_running_(true),
      status_msg_("Discovery graph ready. Waiting for introspection run..."),
      active_state_(STATE_CATALOG),
      is_test_mode_(is_test_mode) {
  std::memset(&original_termios_, 0, sizeof(original_termios_));
}

TerminalUi::~TerminalUi() {
  if (!is_test_mode_) {
    disable_raw_mode();
    show_cursor();
  }
}

void TerminalUi::enable_raw_mode() {
  if (is_test_mode_) return;

  struct termios raw = original_termios_;
  raw.c_lflag &= ~(ECHO | ICANON);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  hide_cursor();
}

void TerminalUi::disable_raw_mode() {
  if (is_test_mode_) return;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);
  show_cursor();
}

void TerminalUi::update_interfaces(
    const std::vector<InterfaceRecord>& new_interfaces, bool is_service) {
  update_interfaces(new_interfaces, is_service ? MODE_SERVICE : MODE_TOPIC);
}

void TerminalUi::update_interfaces(
    const std::vector<InterfaceRecord>& new_interfaces, UiMode mode) {
  std::vector<InterfaceRecord>& current =
      (mode == MODE_SERVICE) ? services_
                             : ((mode == MODE_ACTION) ? actions_ : topics_);
  size_t& highlight_idx =
      (mode == MODE_SERVICE) ? service_highlight_index_
                             : ((mode == MODE_ACTION) ? action_highlight_index_
                                                      : topic_highlight_index_);

  std::vector<InterfaceRecord> preserved = new_interfaces;
  for (auto& new_inf : preserved) {
    for (const auto& old_inf : current) {
      if (old_inf.name == new_inf.name && old_inf.type == new_inf.type) {
        new_inf.is_selected = old_inf.is_selected;
        new_inf.has_introspection = old_inf.has_introspection;
        new_inf.type_hash = old_inf.type_hash;
        new_inf.request_fields = old_inf.request_fields;
        new_inf.response_fields = old_inf.response_fields;
        break;
      }
    }
  }
  current = preserved;
  if (highlight_idx >= current.size()) {
    highlight_idx = current.empty() ? 0 : current.size() - 1;
  }
}

void TerminalUi::update_introspection_details(
    const IntrospectionRecord& record) {
  detail_record_ = record;
}

void TerminalUi::set_interface_introspection(
    size_t index, bool is_service, const IntrospectionRecord& record) {
  set_interface_introspection(index, is_service ? MODE_SERVICE : MODE_TOPIC,
                              record);
}

void TerminalUi::set_interface_introspection(
    size_t index, UiMode mode, const IntrospectionRecord& record) {
  std::vector<InterfaceRecord>& current =
      (mode == MODE_SERVICE) ? services_
                             : ((mode == MODE_ACTION) ? actions_ : topics_);
  if (index < current.size()) {
    current[index].has_introspection = true;
    current[index].type_hash = record.type_hash;
    current[index].request_fields = record.request_fields;
    current[index].response_fields = record.response_fields;
  }
}

void TerminalUi::select_interface(size_t index, bool is_service,
                                  bool selected) {
  select_interface(index, is_service ? MODE_SERVICE : MODE_TOPIC, selected);
}

void TerminalUi::select_interface(size_t index, UiMode mode, bool selected) {
  std::vector<InterfaceRecord>& current =
      (mode == MODE_SERVICE) ? services_
                             : ((mode == MODE_ACTION) ? actions_ : topics_);
  if (index < current.size()) {
    current[index].is_selected = selected;
  }
}

size_t TerminalUi::get_highlight_index() const {
  if (active_mode_ == MODE_SERVICE) {
    return service_highlight_index_;
  }
  if (active_mode_ == MODE_ACTION) {
    return action_highlight_index_;
  }
  return topic_highlight_index_;
}

InterfaceRecord TerminalUi::get_interface_at(size_t index,
                                             bool is_service) const {
  return get_interface_at(index, is_service ? MODE_SERVICE : MODE_TOPIC);
}

InterfaceRecord TerminalUi::get_interface_at(size_t index, UiMode mode) const {
  const std::vector<InterfaceRecord>& current =
      (mode == MODE_SERVICE) ? services_
                             : ((mode == MODE_ACTION) ? actions_ : topics_);
  return current[index];
}

size_t TerminalUi::get_interfaces_count(bool is_service) const {
  return get_interfaces_count(is_service ? MODE_SERVICE : MODE_TOPIC);
}

size_t TerminalUi::get_interfaces_count(UiMode mode) const {
  const std::vector<InterfaceRecord>& current =
      (mode == MODE_SERVICE) ? services_
                             : ((mode == MODE_ACTION) ? actions_ : topics_);
  return current.size();
}

void TerminalUi::render() {
  screen_output_.clear();

  int terminal_width = 80;
  int terminal_height = 24;
  if (!is_test_mode_) {
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0) {
      if (w.ws_row > 0) terminal_height = w.ws_row;
      if (w.ws_col > 0) terminal_width = w.ws_col;
    }
  }

  auto insert_vertical_padding = [&](int footer_lines) {
    int content_lines = 0;
    for (char c : screen_output_) {
      if (c == '\n') content_lines++;
    }
    int padding = terminal_height - 1 - (content_lines + footer_lines);
    for (int p = 0; p < padding; ++p) {
      screen_output_.append("\n");
    }
  };

  WriteUiOutput(screen_output_, "\033[35;1m Aquarium Setup Assistant\n");

  if (active_state_ == STATE_CATALOG) {
    WriteUiOutput(
        screen_output_,
        " %s[T]opics\033[0m      %s[S]ervices\033[0m      %s[A]ctions\033[0m\n",
        active_mode_ == MODE_TOPIC ? "\033[7m" : "",
        active_mode_ == MODE_SERVICE ? "\033[7m" : "",
        active_mode_ == MODE_ACTION ? "\033[7m" : "");
  } else if (active_state_ == STATE_DETAIL) {
    WriteUiOutput(screen_output_, " Interface Details\n");
  }

  WriteUiOutput(screen_output_, "%s\033[0m\n",
                GetSeparatorLine('=', terminal_width).c_str());

  if (active_state_ == STATE_CATALOG) {
    const auto& current_list =
        (active_mode_ == MODE_SERVICE)
            ? services_
            : ((active_mode_ == MODE_ACTION) ? actions_ : topics_);
    const size_t highlight_idx =
        (active_mode_ == MODE_SERVICE)
            ? service_highlight_index_
            : ((active_mode_ == MODE_ACTION) ? action_highlight_index_
                                             : topic_highlight_index_);

    int checkbox_width = 6;
    int name_width = static_cast<int>(terminal_width * 0.4);
    if (name_width < 28) name_width = 28;
    if (name_width > 50) name_width = 50;
    int spacing = 2;
    int type_width = terminal_width - checkbox_width - spacing - name_width - 2;
    if (type_width < 20) type_width = 20;

    if (current_list.empty()) {
      const char* mode_label =
          (active_mode_ == MODE_SERVICE)
              ? "SERVICES"
              : ((active_mode_ == MODE_ACTION) ? "ACTIONS" : "TOPICS");
      WriteUiOutput(screen_output_,
                    " \033[31;1m  [EMPTY] No %s detected over network graph"
                    ". Scanning...\033[0m\n",
                    mode_label);
    } else {
      for (size_t i = 0; i < current_list.size(); ++i) {
        const auto& inf = current_list[i];
        const bool is_highlighted = (i == highlight_idx);
        if (is_highlighted) {
          WriteUiOutput(screen_output_, "\033[7m");
        }

        const char check_char = inf.is_selected ? 'X' : ' ';
        WriteUiOutput(screen_output_, "  [%c] %-*s  %-*s", check_char,
                      name_width, inf.name.c_str(), type_width,
                      inf.type.c_str());

        if (is_highlighted) {
          WriteUiOutput(screen_output_, "\033[0m");
        }
        WriteUiOutput(screen_output_, "\n");
      }
    }

    insert_vertical_padding(8);
    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('-', terminal_width).c_str());

    size_t selected_count = 0;
    for (const auto& s : services_) {
      if (s.is_selected) selected_count++;
    }
    for (const auto& t : topics_) {
      if (t.is_selected) selected_count++;
    }
    for (const auto& a : actions_) {
      if (a.is_selected) selected_count++;
    }

    const size_t total_discovered =
        services_.size() + topics_.size() + actions_.size();

    WriteUiOutput(screen_output_,
                  " STATUS: [\033[35;1mSelected: %zu\033[0m / Discovered: "
                  "%zu (S:%zu, T:%zu, A:%zu)] %s\n",
                  selected_count, total_discovered, services_.size(),
                  topics_.size(), actions_.size(), status_msg_.c_str());

    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('=', terminal_width).c_str());
    WriteUiOutput(screen_output_, " COMMAND KEYS:\n");
    WriteUiOutput(
        screen_output_,
        "  [Arrow Up/Dn or j/k] Navigate list  [Space] Toggle Select\n");
    WriteUiOutput(screen_output_,
                  "  [Enter] Show Introspection Detail    [W] Write Config "
                  "File\n");
    WriteUiOutput(
        screen_output_,
        "  [T] Topics   [S] Services   [A] Actions     [Q] Quit Setup "
        "Wizard\n");
    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('=', terminal_width).c_str());
  } else if (active_state_ == STATE_DETAIL) {
    WriteUiOutput(screen_output_, "  \033[1mName:\033[0m  %s\n",
                  detail_record_.interface_name.c_str());
    WriteUiOutput(screen_output_, "  \033[1mType:\033[0m  %s\n",
                  detail_record_.type_name.c_str());
    WriteUiOutput(screen_output_,
                  "  \033[1mType ID Hash:\033[0m  \033[36m%s\033[0m\n",
                  detail_record_.type_hash.c_str());

    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('-', terminal_width).c_str());

    if (!detail_record_.request_fields.empty()) {
      WriteUiOutput(screen_output_,
                    "  \033[32;1mRequest Message Schema Fields:\033[0m\n");
      for (const auto& f : detail_record_.request_fields) {
        WriteUiOutput(screen_output_, "    - %s\n", f.c_str());
      }
      WriteUiOutput(screen_output_, "\n");
    }

    WriteUiOutput(screen_output_,
                  "  \033[32;1mResponse/Message Schema Fields:\033[0m\n");
    for (const auto& f : detail_record_.response_fields) {
      WriteUiOutput(screen_output_, "    - %s\n", f.c_str());
    }

    insert_vertical_padding(8);
    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('-', terminal_width).c_str());
    WriteUiOutput(screen_output_, " STATUS: %s\n", status_msg_.c_str());
    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('=', terminal_width).c_str());
    WriteUiOutput(screen_output_, " COMMAND KEYS:\n");
    WriteUiOutput(screen_output_,
                  "  [Escape] Return to list                                 "
                  "[Q] Quit Setup "
                  "Wizard\n");
    WriteUiOutput(screen_output_, "\n\n");
    WriteUiOutput(screen_output_, "%s\n",
                  GetSeparatorLine('=', terminal_width).c_str());
  }

  if (!is_test_mode_) {
    clear_screen();
    move_cursor_to_home();
    std::printf("%s", screen_output_.c_str());
    std::fflush(stdout);
  }
}

KeyPress TerminalUi::read_input() {
  if (is_test_mode_) {
    if (test_key_queue_.empty()) {
      return KEY_QUIT;
    }
    KeyPress queued = test_key_queue_.front();
    test_key_queue_.erase(test_key_queue_.begin());
    return queued;
  }

  struct pollfd fds[1];
  fds[0].fd = STDIN_FILENO;
  fds[0].events = POLLIN;

  const int poll_ret = poll(fds, 1, 1000);
  if (poll_ret == 0) {
    return KEY_TIMEOUT;
  }
  if (poll_ret < 0) {
    if (errno == EINTR) {
      return KEY_TIMEOUT;
    }
    return KEY_QUIT;
  }

  if (fds[0].revents & (POLLHUP | POLLERR)) {
    return KEY_QUIT;
  }

  char c = 0;
  const ssize_t bytes_read = read(STDIN_FILENO, &c, 1);
  if (bytes_read == 0) {
    return KEY_QUIT;
  }
  if (bytes_read < 0) {
    return KEY_UNKNOWN;
  }

  if (c == 27) {
    // Arrow key sequence bytes arrive within <1ms; standalone Escape has none.
    struct pollfd esc_fd[1];
    esc_fd[0].fd = STDIN_FILENO;
    esc_fd[0].events = POLLIN;

    if (poll(esc_fd, 1, 10) <= 0) {
      return KEY_ESCAPE;
    }

    char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) == 1 &&
        read(STDIN_FILENO, &seq[1], 1) == 1 && seq[0] == '[') {
      switch (seq[1]) {
        case 'A':
          return KEY_UP;
        case 'B':
          return KEY_DOWN;
        default:
          return KEY_UNKNOWN;
      }
    }
  } else {
    switch (c) {
      case 'k':
      case 'K':
        return KEY_UP;
      case 'j':
      case 'J':
        return KEY_DOWN;
      case ' ':
        return KEY_SPACE;
      case '\r':
      case '\n':
        return KEY_ENTER;
      case 'w':
      case 'W':
        return KEY_WRITE_CONFIG;
      case 'p':
      case 'P':
        return KEY_TOGGLE_SYSTEM;
      case 't':
      case 'T':
        return KEY_SWITCH_TOPIC;
      case 's':
      case 'S':
        return KEY_SWITCH_SERVICE;
      case 'a':
      case 'A':
        return KEY_SWITCH_ACTION;
      case 'q':
      case 'Q':
        return KEY_QUIT;
      default:
        return KEY_UNKNOWN;
    }
  }
  return KEY_UNKNOWN;
}

void TerminalUi::process_key(KeyPress key) {
  if (active_state_ == STATE_DETAIL && key != KEY_ESCAPE && key != KEY_QUIT) {
    return;
  }

  std::vector<InterfaceRecord>& current =
      (active_mode_ == MODE_SERVICE)
          ? services_
          : ((active_mode_ == MODE_ACTION) ? actions_ : topics_);
  size_t& highlight_idx =
      (active_mode_ == MODE_SERVICE)
          ? service_highlight_index_
          : ((active_mode_ == MODE_ACTION) ? action_highlight_index_
                                           : topic_highlight_index_);

  switch (key) {
    case KEY_UP:
      if (highlight_idx > 0) {
        highlight_idx--;
      } else if (!current.empty()) {
        highlight_idx = current.size() - 1;
      }
      break;

    case KEY_DOWN:
      if (!current.empty()) {
        if (highlight_idx < current.size() - 1) {
          highlight_idx++;
        } else {
          highlight_idx = 0;
        }
      }
      break;

    case KEY_SPACE:
      if (highlight_idx < current.size()) {
        current[highlight_idx].is_selected =
            !current[highlight_idx].is_selected;
      }
      break;

    case KEY_ENTER:
      if (active_state_ == STATE_CATALOG && highlight_idx < current.size()) {
        active_state_ = STATE_DETAIL;
      }
      break;

    case KEY_ESCAPE:
      if (active_state_ == STATE_DETAIL) {
        active_state_ = STATE_CATALOG;
        status_msg_ = "Returned to active catalog listing.";
      }
      break;

    case KEY_SWITCH_TOPIC:
      if (active_state_ == STATE_CATALOG) {
        active_mode_ = MODE_TOPIC;
        status_msg_ = "Switched to Topic Mode.";
      }
      break;

    case KEY_SWITCH_SERVICE:
      if (active_state_ == STATE_CATALOG) {
        active_mode_ = MODE_SERVICE;
        status_msg_ = "Switched to Service Mode.";
      }
      break;

    case KEY_SWITCH_ACTION:
      if (active_state_ == STATE_CATALOG) {
        active_mode_ = MODE_ACTION;
        status_msg_ = "Switched to Action Mode.";
      }
      break;

    case KEY_QUIT:
      is_running_ = false;
      break;

    default:
      break;
  }
}

std::vector<InterfaceRecord> TerminalUi::get_selected_interfaces() const {
  std::vector<InterfaceRecord> selected;
  for (const auto& s : services_) {
    if (s.is_selected) {
      selected.push_back(s);
    }
  }
  for (const auto& t : topics_) {
    if (t.is_selected) {
      selected.push_back(t);
    }
  }
  for (const auto& a : actions_) {
    if (a.is_selected) {
      selected.push_back(a);
    }
  }
  return selected;
}

void TerminalUi::queue_test_key(KeyPress key) {
  test_key_queue_.push_back(key);
}

std::string TerminalUi::get_clean_screen() const {
  std::string clean;
  bool inside_ansi = false;
  for (size_t i = 0; i < screen_output_.length(); ++i) {
    const char c = screen_output_[i];
    if (c == 27) {
      inside_ansi = true;
      continue;
    }
    if (inside_ansi) {
      if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
        inside_ansi = false;
      }
      continue;
    }
    clean.push_back(c);
  }
  return clean;
}

void TerminalUi::clear_screen() {
  if (is_test_mode_) return;
  std::printf("\033[2J");
}

void TerminalUi::move_cursor_to_home() {
  if (is_test_mode_) return;
  std::printf("\033[H");
}

void TerminalUi::hide_cursor() {
  if (is_test_mode_) return;
  std::printf("\033[?25l");
}

void TerminalUi::show_cursor() {
  if (is_test_mode_) return;
  std::printf("\033[?25h");
}

}  // namespace aquarium_setup_assistant
