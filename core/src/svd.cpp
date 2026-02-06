#include "sclerp/core/math/svd.hpp"
#include <Eigen/SVD>

namespace sclerp::core {

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd& A,
                                 const SvdPseudoInverseOptions& opt) {
  if (A.size() == 0) return Eigen::MatrixXd();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();

  Eigen::VectorXd S_inv = S;
  if (opt.mode == SvdPseudoInverseMode::Damped) {
    const double lambda2 = opt.lambda * opt.lambda;
    for (int i = 0; i < S_inv.size(); ++i) {
      const double s = S(i);
      S_inv(i) = (s == 0.0) ? 0.0 : (s / (s * s + lambda2));
    }
  } else {
    const double s_max = (S.size() > 0) ? S.maxCoeff() : 0.0;
    const double thresh = opt.tol * s_max;  // relative threshold (robust default)
    for (int i = 0; i < S_inv.size(); ++i) {
      S_inv(i) = (S(i) > thresh) ? (1.0 / S(i)) : 0.0;
    }
  }

  return svd.matrixV() * S_inv.asDiagonal() * svd.matrixU().transpose();
}

}  // namespace sclerp::core

