#include "sclerp/core/common/logger.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>

#ifndef _WIN32
#include <unistd.h>
#include <cstdio>
#endif

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

static bool useColor() {
#ifdef _WIN32
  return false;
#else
  static int cached = -1;
  if (cached != -1) {
    return cached == 1;
  }
  if (std::getenv("NO_COLOR") != nullptr) {
    cached = 0;
    return false;
  }
  if (!isatty(fileno(stderr))) {
    cached = 0;
    return false;
  }
  cached = 1;
  return true;
#endif
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
  if (useColor()) {
    const char* color = logLevelToColor(level);
    std::cerr << color << "[sclerp][" << logLevelToString(level) << "] "
              << msg << "\x1b[0m\n";
    return;
  }
  std::cerr << "[sclerp][" << logLevelToString(level) << "] "
            << msg << "\n";
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
