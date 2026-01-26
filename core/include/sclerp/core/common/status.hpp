#pragma once
#include <cstdint>

namespace sclerp::core {

// Refactored replacement for kinlib::ErrorCodes
enum class Status : std::uint8_t {
  Success = 0,
  Failure = 1,
  JointLimit = 2,
  InvalidParameter = 3
};

inline constexpr bool ok(Status s) { return s == Status::Success; }

}  // namespace sclerp::core
