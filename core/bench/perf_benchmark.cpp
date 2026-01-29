#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/model/manipulator_model.hpp"
#include "sclerp/core/dual_quat/dual_quat.hpp"
#include "sclerp/core/math/so3.hpp"

using sclerp::core::DualQuat;
using sclerp::core::KinematicsSolver;
using sclerp::core::ManipulatorBuilder;
using sclerp::core::ManipulatorModel;
using sclerp::core::Transform;
using sclerp::core::Vec3;
using sclerp::core::Mat3;
using sclerp::core::AdjointMatrix;
using sclerp::core::JointType;
using sclerp::core::Status;
using sclerp::core::ok;

static int parseIntArg(int argc, char** argv, const char* key, int def) {
  const std::string prefix = std::string(key) + "=";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0 && i + 1 < argc) {
      return std::stoi(argv[i + 1]);
    }
    if (std::strncmp(argv[i], prefix.c_str(), prefix.size()) == 0) {
      return std::stoi(std::string(argv[i] + prefix.size()));
    }
  }
  return def;
}

static bool parseFlag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) {
      return true;
    }
  }
  return false;
}

static ManipulatorModel makeBenchModel(int dof) {
  const double link_len = 0.3;
  Transform M = Transform::Identity();
  M.translation() = Vec3(link_len * dof, 0.0, 0.0);

  ManipulatorBuilder builder;
  builder.set_ee_home(M);

  for (int i = 0; i < dof; ++i) {
    Vec3 axis;
    switch (i % 3) {
      case 0: axis = Vec3(0.0, 0.0, 1.0); break;
      case 1: axis = Vec3(0.0, 1.0, 0.0); break;
      default: axis = Vec3(1.0, 0.0, 0.0); break;
    }
    const Vec3 point(link_len * i, 0.0, 0.0);
    builder.add_revolute("j" + std::to_string(i + 1), axis, point);
  }
  ManipulatorModel model;
  const Status st = builder.build(&model);
  if (!ok(st)) {
    std::cerr << "ManipulatorBuilder build failed\n";
    std::exit(1);
  }
  return model;
}

static Mat3 hat3Local(const Vec3& w) {
  Mat3 W;
  W <<     0.0, -w.z(),  w.y(),
        w.z(),     0.0, -w.x(),
       -w.y(),  w.x(),     0.0;
  return W;
}

static Transform jointExpBench(const sclerp::core::JointSpec& j, double q) {
  Transform T = Transform::Identity();

  if (j.type == JointType::Revolute) {
    const Vec3 w = j.axis;
    const Mat3 R = Eigen::AngleAxisd(q, w).toRotationMatrix();
    const Vec3 p = (Mat3::Identity() - R) * j.point;
    T.linear() = R;
    T.translation() = p;
  } else if (j.type == JointType::Prismatic) {
    T.translation() = j.axis * q;
  }
  return T;
}

static AdjointMatrix adjointVWBench(const Transform& T) {
  const Mat3 R = T.rotation();
  const Vec3 p = T.translation();

  AdjointMatrix Ad = AdjointMatrix::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(3,3) = R;
  Ad.block<3,3>(0,3) = hat3Local(p) * R;
  return Ad;
}

static Status forwardKinematicsBaseline(const ManipulatorModel& model,
                                        const Eigen::VectorXd& q,
                                        Transform* g_base_tool) {
  if (!g_base_tool) return Status::InvalidParameter;
  if (q.size() != model.dof()) return Status::InvalidParameter;

  const int n = model.dof();
  Transform prod = Transform::Identity();
  for (int i = 0; i < n; ++i) {
    prod = prod * jointExpBench(model.joint(i), q(i));
  }
  *g_base_tool = prod * model.ee_home();
  return Status::Success;
}

static Status spatialJacobianBaseline(const ManipulatorModel& model,
                                      const Eigen::VectorXd& q,
                                      Eigen::MatrixXd* J_space) {
  if (!J_space) return Status::InvalidParameter;

  const int n = model.dof();
  if (q.size() != n) return Status::InvalidParameter;

  J_space->resize(6, n);

  Eigen::MatrixXd joint_twists = Eigen::MatrixXd::Zero(6, n);
  for (int i = 0; i < n; ++i) {
    const auto& j = model.joint(i);
    if (j.type == JointType::Revolute) {
      const Vec3 w = j.axis;
      const Vec3 v = -w.cross(j.point);
      joint_twists.block<3,1>(0, i) = v;
      joint_twists.block<3,1>(3, i) = w;
    } else if (j.type == JointType::Prismatic) {
      joint_twists.block<3,1>(0, i) = j.axis;
    }
  }

  if (n > 0) {
    J_space->col(0) = joint_twists.col(0);
  }

  Transform prod = Transform::Identity();
  for (int i = 1; i < n; ++i) {
    prod = prod * jointExpBench(model.joint(i - 1), q(i - 1));
    const AdjointMatrix Ad = adjointVWBench(prod);
    J_space->col(i) = Ad * joint_twists.col(i);
  }

  return Status::Success;
}

static Status rmrcIncrementBaseline(const ManipulatorModel& model,
                                    const DualQuat& dq_i,
                                    const DualQuat& dq_f,
                                    const Eigen::VectorXd& q_current,
                                    Eigen::VectorXd* dq) {
  if (!dq) return Status::InvalidParameter;

  const Transform g_i = dq_i.toTransform();
  const Transform g_f = dq_f.toTransform();

  Eigen::Matrix<double, 7, 1> gamma_i;
  Eigen::Matrix<double, 7, 1> gamma_f;

  const Vec3 p_i = g_i.translation();
  gamma_i.head<3>() = p_i;
  const auto q_i = dq_i.real();
  gamma_i(3) = q_i.w();
  gamma_i(4) = q_i.x();
  gamma_i(5) = q_i.y();
  gamma_i(6) = q_i.z();

  const Vec3 p_f = g_f.translation();
  gamma_f.head<3>() = p_f;
  const auto q_f = dq_f.real();
  gamma_f(3) = q_f.w();
  gamma_f(4) = q_f.x();
  gamma_f(5) = q_f.y();
  gamma_f(6) = q_f.z();

  Eigen::MatrixXd s_jac;
  const Status st = spatialJacobianBaseline(model, q_current, &s_jac);
  if (!ok(st)) return st;

  Eigen::Vector4d qv;
  qv << q_i.w(), q_i.x(), q_i.y(), q_i.z();

  Eigen::Matrix<double, 3, 4> J1 = Eigen::Matrix<double, 3, 4>::Zero();
  J1.block<3,1>(0,0) = -qv.tail<3>();
  J1(0,1) =  qv(0);
  J1(0,2) = -qv(3);
  J1(0,3) =  qv(2);
  J1(1,1) =  qv(3);
  J1(1,2) =  qv(0);
  J1(1,3) = -qv(1);
  J1(2,1) = -qv(2);
  J1(2,2) =  qv(1);
  J1(2,3) =  qv(0);

  Eigen::Matrix<double, 6, 7> J2 = Eigen::Matrix<double, 6, 7>::Zero();
  J2.block<3,3>(0,0) = Mat3::Identity();
  J2.block<3,4>(0,3) = 2.0 * hat3Local(p_i) * J1;
  J2.block<3,4>(3,3) = 2.0 * J1;

  Eigen::MatrixXd temp = s_jac * s_jac.transpose();
  Eigen::MatrixXd jac_pinv = s_jac.transpose() * temp.inverse();
  Eigen::MatrixXd B = jac_pinv * J2;

  *dq = B * (gamma_f - gamma_i);
  return Status::Success;
}

template <typename Fn>
static double benchMs(Fn&& fn) {
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  const auto t1 = std::chrono::steady_clock::now();
  const std::chrono::duration<double, std::milli> dt = t1 - t0;
  return dt.count();
}

int main(int argc, char** argv) {
  if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 ||
                   std::strcmp(argv[1], "-h") == 0)) {
    std::cout << "Usage: sclerp_core_benchmark [--dof=N] [--jac-iters=N] [--rmrc-iters=N]\n";
    std::cout << "  Optional: --trials=N --warmup=N\n";
    return 0;
  }

  const int dof = parseIntArg(argc, argv, "--dof", 7);
  const int jac_iters = parseIntArg(argc, argv, "--jac-iters", 20000);
  const int rmrc_iters = parseIntArg(argc, argv, "--rmrc-iters", 10000);
  const int trials = parseIntArg(argc, argv, "--trials", 5);
  const int warmup = parseIntArg(argc, argv, "--warmup", 1);
  const bool quiet = parseFlag(argc, argv, "--quiet");

  ManipulatorModel model = makeBenchModel(dof);
  KinematicsSolver solver(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(dof);
  Eigen::MatrixXd J;
  Eigen::MatrixXd J_base;
  double acc = 0.0;

  auto run_jac = [&](bool baseline) {
    for (int i = 0; i < jac_iters; ++i) {
      const double t = 0.001 * static_cast<double>(i);
      for (int j = 0; j < dof; ++j) {
        q(j) = 0.2 * std::sin(t + 0.3 * j);
      }
      const Status st = baseline
        ? spatialJacobianBaseline(model, q, &J_base)
        : solver.spatialJacobian(q, &J);
      if (!ok(st)) {
        std::cerr << "spatialJacobian failed\n";
        std::exit(1);
      }
      acc += baseline ? J_base(0, 0) : J(0, 0);
    }
  };

  Eigen::VectorXd dq = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd dq_base = Eigen::VectorXd::Zero(dof);
  Transform g_i = Transform::Identity();
  Transform g_f = Transform::Identity();
  KinematicsSolver::RmrcWorkspace rmrc_ws;

  Eigen::VectorXd q_goal = q;
  if (dof > 0) q_goal(0) += 0.3;
  if (!ok(forwardKinematicsBaseline(model, q_goal, &g_f))) {
    std::cerr << "forwardKinematics failed\n";
    return 1;
  }
  const DualQuat dq_f(g_f);

  auto run_rmrc = [&](bool baseline) {
    for (int i = 0; i < rmrc_iters; ++i) {
      const double t = 0.002 * static_cast<double>(i);
      for (int j = 0; j < dof; ++j) {
        q(j) = 0.15 * std::sin(t + 0.2 * j);
      }
      const Status fk_st = baseline
        ? forwardKinematicsBaseline(model, q, &g_i)
        : solver.forwardKinematics(q, &g_i);
      if (!ok(fk_st)) {
        std::cerr << "forwardKinematics failed\n";
        std::exit(1);
      }
      const DualQuat dq_i(g_i);
      const Status st = baseline
        ? rmrcIncrementBaseline(model, dq_i, dq_f, q, &dq_base)
        : solver.rmrcIncrement(dq_i, dq_f, q, &dq, &rmrc_ws);
      if (!ok(st)) {
        std::cerr << "rmrcIncrement failed\n";
        std::exit(1);
      }
      acc += baseline ? dq_base.sum() : dq.sum();
    }
  };

  std::vector<double> jac_runs;
  std::vector<double> jac_base_runs;
  std::vector<double> rmrc_runs;
  std::vector<double> rmrc_base_runs;
  jac_runs.reserve(trials);
  jac_base_runs.reserve(trials);
  rmrc_runs.reserve(trials);
  rmrc_base_runs.reserve(trials);

  for (int i = 0; i < warmup; ++i) {
    run_jac(false);
    run_jac(true);
    run_rmrc(false);
    run_rmrc(true);
  }

  for (int i = 0; i < trials; ++i) {
    jac_runs.push_back(benchMs([&]() { run_jac(false); }));
    jac_base_runs.push_back(benchMs([&]() { run_jac(true); }));
    rmrc_runs.push_back(benchMs([&]() { run_rmrc(false); }));
    rmrc_base_runs.push_back(benchMs([&]() { run_rmrc(true); }));
    if (!quiet) {
      std::cout << "trial " << (i + 1) << "/" << trials << " done\n";
    }
  }

  auto median = [](std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  };

  const double jac_ms = median(jac_runs);
  const double jac_base_ms = median(jac_base_runs);
  const double rmrc_ms = median(rmrc_runs);
  const double rmrc_base_ms = median(rmrc_base_runs);

  std::cout << "sclerp_core_benchmark\n";
  std::cout << "  dof: " << dof << "\n";
  std::cout << "  trials: " << trials << " (warmup " << warmup << ")\n";
  std::cout << "  spatialJacobian: " << jac_ms << " ms total, "
            << (jac_ms * 1000.0 / jac_iters) << " us/call\n";
  std::cout << "  spatialJacobian (baseline): " << jac_base_ms << " ms total, "
            << (jac_base_ms * 1000.0 / jac_iters) << " us/call\n";
  std::cout << "  rmrcIncrement:   " << rmrc_ms << " ms total, "
            << (rmrc_ms * 1000.0 / rmrc_iters) << " us/call\n";
  std::cout << "  rmrcIncrement (baseline): " << rmrc_base_ms << " ms total, "
            << (rmrc_base_ms * 1000.0 / rmrc_iters) << " us/call\n";

  if (acc == 0.123456) {
    std::cout << "ignore: " << acc << "\n";
  }
  return 0;
}
