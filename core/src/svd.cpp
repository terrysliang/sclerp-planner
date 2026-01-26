#include "sclerp/core/math/svd.hpp"
#include <Eigen/SVD>

namespace sclerp::core {

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd& A, double tol) {
  if (A.size() == 0) return Eigen::MatrixXd();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();

  const double s_max = (S.size() > 0) ? S.maxCoeff() : 0.0;
  const double thresh = tol * s_max;  // relative threshold (robust default)

  Eigen::VectorXd S_inv = S;
  for (int i = 0; i < S_inv.size(); ++i) {
    S_inv(i) = (S(i) > thresh) ? (1.0 / S(i)) : 0.0;
  }

  return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

}  // namespace sclerp::core

