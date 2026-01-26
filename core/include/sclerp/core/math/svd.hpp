#pragma once
#include <Eigen/Core>

namespace sclerp::core {

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd& A, double tol = 1e-6);

}  // namespace sclerp::core
