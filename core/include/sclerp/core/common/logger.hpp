#pragma once
#include <cstdint>
#include <string>

namespace sclerp::core {

enum class LogLevel : std::uint8_t {
  Error = 0,
  Warn = 1,
  Info = 2,
  Debug = 3
};

using LogSink = void(*)(LogLevel, const std::string&);

void setLogLevel(LogLevel level);
LogLevel getLogLevel();

void setLogSink(LogSink sink);
LogSink getLogSink();

bool shouldLog(LogLevel level);
void log(LogLevel level, const std::string& msg);
void log(LogLevel level, const char* msg);

const char* logLevelToString(LogLevel level);

}  // namespace sclerp::core
