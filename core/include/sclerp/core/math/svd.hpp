#pragma once
#include <Eigen/Core>
#include <cstdint>

namespace sclerp::core {

enum class SvdPseudoInverseMode : std::uint8_t {
  Truncated = 0,
  Damped = 1
};

struct SvdPseudoInverseOptions {
  SvdPseudoInverseMode mode{SvdPseudoInverseMode::Damped};
  double tol{1e-6};     // relative threshold for Truncated
  double lambda{5e-2};  // damping for Damped
};

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd& A,
                                 const SvdPseudoInverseOptions& opt = {});

}  // namespace sclerp::core
