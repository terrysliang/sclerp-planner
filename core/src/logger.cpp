#include "sclerp/core/common/logger.hpp"

#include <atomic>
#include <iostream>

namespace sclerp::core {

static std::atomic<LogLevel> g_level{LogLevel::Warn};
static std::atomic<LogSink> g_sink{nullptr};

const char* logLevelToString(LogLevel level) {
  switch (level) {
    case LogLevel::Error: return "ERROR";
    case LogLevel::Warn: return "WARN";
    case LogLevel::Info: return "INFO";
    case LogLevel::Debug: return "DEBUG";
  }
  return "UNKNOWN";
}

static const char* logLevelToColor(LogLevel level) {
  switch (level) {
    case LogLevel::Error: return "\x1b[31m";  // red
    case LogLevel::Warn: return "\x1b[33m";   // yellow
    case LogLevel::Info: return "\x1b[36m";   // cyan
    case LogLevel::Debug: return "\x1b[90m";  // bright black
  }
  return "\x1b[0m";
}

static void defaultSink(LogLevel level, const std::string& msg) {
  const char* color = logLevelToColor(level);
  std::cerr << color << "[sclerp][" << logLevelToString(level) << "] "
            << msg << "\x1b[0m\n";
}

static LogSink sinkOrDefault() {
  LogSink sink = g_sink.load();
  return sink ? sink : &defaultSink;
}

void setLogLevel(LogLevel level) {
  g_level.store(level);
}

LogLevel getLogLevel() {
  return g_level.load();
}

void setLogSink(LogSink sink) {
  g_sink.store(sink);
}

LogSink getLogSink() {
  return g_sink.load();
}

bool shouldLog(LogLevel level) {
  return static_cast<int>(level) <= static_cast<int>(g_level.load());
}

void log(LogLevel level, const std::string& msg) {
  if (!shouldLog(level)) return;
  sinkOrDefault()(level, msg);
}

void log(LogLevel level, const char* msg) {
  if (!shouldLog(level)) return;
  sinkOrDefault()(level, msg ? std::string(msg) : std::string());
}

}  // namespace sclerp::core
