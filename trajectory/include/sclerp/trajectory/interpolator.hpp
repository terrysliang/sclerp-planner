#pragma once

#include "sclerp/core/common/status.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/path/joint_path.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace sclerp::trajectory {

using sclerp::core::Status;

struct Limits {
  Eigen::VectorXd v_max;  // [rad/s] or [m/s]
  Eigen::VectorXd a_max;  // [rad/s^2] or [m/s^2]
  Eigen::VectorXd j_max;  // [rad/s^3] or [m/s^3] (optional; may be unused)
};

struct Sample {
  double t{0.0};
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;
};

struct PlannedSegment {
  double T{0.0};
  Eigen::VectorXd q0;
  Eigen::VectorXd dQ;
  Eigen::Matrix<double, 6, 1> c;
};

struct PlannedTrajectory {
  std::vector<std::string> joint_names;
  std::vector<PlannedSegment> segs;
  std::vector<Sample> table;
  double total_time{0.0};
  double sample_dt{0.0};
};

std::vector<bool> continuousRevoluteMask(const sclerp::core::ManipulatorModel& model);

class GridTotg {
public:
  struct Config {
    bool unwrap_angles{true};

    // Cleanup
    double dedup_eps{1e-7};     // per-joint max-abs
    double drop_eps{1e-5};      // norm threshold

    // Geometry
    bool resample_uniform{true};
    double resample_ds{0.01};   // radians in chord norm for revolute-only robots
    int max_resample_points{20000};

    // Time-scaling grid
    double grid_ds{0.002};      // about resample_ds / 5
    int max_grid_points{100000};

    double sdot_start{0.0};
    double sdot_goal{0.0};

    bool compute_qd_qdd{false};
    bool verbose{false};
  };

  explicit GridTotg(Limits lim);
  GridTotg(Limits lim, Config cfg);

  // Robust time scaling on a smooth q(s) spline; samples uniformly in time at sample_dt.
  Status plan(const std::vector<Eigen::VectorXd>& qs_in,
              double sample_dt,
              PlannedTrajectory* out,
              const std::vector<bool>& unwrap_revolute_mask = {}) const;

  Status plan(const sclerp::core::JointPath& path,
              double sample_dt,
              PlannedTrajectory* out,
              const std::vector<bool>& unwrap_revolute_mask = {}) const;

private:
  Limits lim_;
  Config cfg_;
};

enum class CsvMode : std::uint8_t {
  PositionOnly = 0,
  PositionVelocityAcceleration = 1
};

Status writeTrajectoryCsv(const PlannedTrajectory& traj,
                          const std::string& csv_path,
                          CsvMode mode = CsvMode::PositionOnly);

Status planWithToppra(const std::vector<Eigen::VectorXd>& qs_in,
                      const Limits& lim,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask = {});

Status planWithToppra(const sclerp::core::JointPath& path,
                      const Limits& lim,
                      double sample_dt,
                      PlannedTrajectory* out,
                      const std::vector<bool>& unwrap_revolute_mask = {});

}  // namespace sclerp::trajectory
