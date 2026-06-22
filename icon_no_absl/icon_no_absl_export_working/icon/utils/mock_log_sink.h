#ifndef ICON_UTILS_MOCK_LOG_SINK_H_
#define ICON_UTILS_MOCK_LOG_SINK_H_

#include <iostream>
#include <source_location>
#include <string>
#include <vector>

#include "icon/utils/attributes.h"
#include "icon/utils/log.h"

namespace intrinsic::log {

struct LogEntryWithStorage {
  std::string msg;
  std::source_location loc;
  Logger::Severity severity;
};

inline Logger::SinkCallback MockSink(
    std::vector<LogEntryWithStorage>& entries INTR_ATTRIBUTE_LIFETIME_BOUND) {
  return [&entries](const Logger::LogEntry& entry) {
    std::cerr << entry.msg << std::endl;
    entries.emplace_back(LogEntryWithStorage{
        .msg = std::string(entry.msg),
        .loc = entry.loc,
        .severity = entry.severity,
    });
  };
}

}  // namespace intrinsic::log

#endif  // ICON_UTILS_MOCK_LOG_SINK_H_
