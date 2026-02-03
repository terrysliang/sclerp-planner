#include "sclerp/collision/lemke.hpp"

#include "sclerp/core/common/logger.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

namespace sclerp::collision {

using sclerp::core::Status;
using sclerp::core::LogLevel;
using sclerp::core::log;

Status lemkeSolve(const Eigen::VectorXd& q,
                  const Eigen::MatrixXd& M,
                  LemkeResult* out) {
  if (!out) {
    log(LogLevel::Error, "lemkeSolve: null output");
    return Status::InvalidParameter;
  }

  const int n = static_cast<int>(q.size());
  if (n <= 0) {
    log(LogLevel::Error, "lemkeSolve: q is empty");
    return Status::InvalidParameter;
  }
  if (M.rows() != n || M.cols() != n) {
    log(LogLevel::Error, "lemkeSolve: M size mismatch");
    return Status::InvalidParameter;
  }

  out->w = Eigen::VectorXd::Zero(n);
  out->z = Eigen::VectorXd::Zero(n);
  out->iterations = 0;
  out->ray_termination = false;

  int pivrow = 0;
  if (q.minCoeff(&pivrow) >= 0.0) {
    out->w = q;
    return Status::Success;
  }

  const int dim = n;
  const int max_shift = std::min(dim, 30);
  if (dim > 30) {
    log(LogLevel::Warn, "lemkeSolve: large problem size, max iterations capped");
  }
  const int maxloop = 1 << max_shift;
  int loop = 0;
  int ray = 0;

  Eigen::MatrixXd table(dim, 2 * dim + 2);
  table.leftCols(dim) = Eigen::MatrixXd::Identity(dim, dim);
  table.middleCols(dim, dim) = -M;
  table.col(2 * dim) = -Eigen::VectorXd::Ones(dim);
  table.col(2 * dim + 1) = q;

  std::vector<int> index(dim);
  for (int i = 0; i < dim; ++i) index[i] = i;

  index[pivrow] = 2 * dim;
  int enter = pivrow + dim;

  table.row(pivrow) /= table(pivrow, 2 * dim);
  for (int i = 0; i < dim; ++i) {
    if (i == pivrow) continue;
    table.row(i) -= table.row(pivrow) * table(i, 2 * dim);
  }

  while (*std::max_element(index.begin(), index.end()) == 2 * dim && loop < maxloop) {
    ++loop;

    const Eigen::VectorXd pivcol = table.col(enter);

    std::vector<bool> postest(dim);
    for (int i = 0; i < dim; ++i) postest[i] = (pivcol(i) <= 0.0);
    if (std::accumulate(postest.begin(), postest.end(), 0) == dim) {
      ray = 1;
      break;
    }

    Eigen::VectorXd ratio(dim);
    for (int i = 0; i < dim; ++i) {
      ratio(i) = postest[i] ? std::numeric_limits<double>::infinity()
                            : table(i, 2 * dim + 1) / pivcol(i);
    }
    ratio.minCoeff(&pivrow);

    table.row(pivrow) /= table(pivrow, enter);
    for (int i = 0; i < dim; ++i) {
      if (i == pivrow) continue;
      table.row(i) -= table.row(pivrow) * table(i, enter);
    }

    const int drop = index[pivrow];
    index[pivrow] = enter;
    enter = (drop > dim) ? (drop - dim) : (drop + dim);
  }

  Eigen::VectorXd solution = Eigen::VectorXd::Zero(2 * dim + 1);
  for (int i = 0; i < dim; ++i) {
    solution(index[i]) = table(i, 2 * dim + 1);
  }

  out->w = solution.segment(0, dim);
  out->z = solution.segment(dim, dim);
  out->iterations = loop;
  out->ray_termination = (ray == 1);

  if (out->ray_termination) {
    log(LogLevel::Warn, "lemkeSolve: ray termination");
    return Status::Failure;
  }

  return Status::Success;
}

}  // namespace sclerp::collision
