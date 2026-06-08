#pragma once

#include <string>
#include <source_location>
#include <vector>

#include "intrinsic/utils/attributes.hpp"
#include "intrinsic/utils/log.hpp"


namespace intrinsic::log
{

struct LogEntryWithStorage
{
  std::string msg;
  std::source_location loc;
  Logger::Severity severity;
};

Logger::SinkCallback MockSink(std::vector<LogEntryWithStorage> & entries)
{
  return [&](const Logger::LogEntry & entry){
           entries.push_back(LogEntryWithStorage{
        .msg = std::string(entry.msg),
        .loc = entry.loc,
        .severity = entry.severity,
      });
         };
}


}
