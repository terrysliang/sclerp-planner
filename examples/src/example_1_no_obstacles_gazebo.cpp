#include "example_common.hpp"

#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/gazebo/world_registry.hpp"
#include "sclerp/gazebo/world_sdf.hpp"
#include "sclerp/trajectory/interpolator.hpp"
#include "sclerp/urdf/load_manipulator.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

static void printUsage(const char* exe) {
  std::cout << "Example 1: plan without obstacles, time-parameterize, export Gazebo world + CSV.\n\n"
            << "Usage:\n"
            << "  " << exe << " [--urdf <path> --stl-dir <dir> --base-link <name> --tip-link <name>] [options]\n\n"
            << "If robot args are omitted, uses bundled Diana7 assets.\n\n";
  sclerp::examples::printCommonCli(std::cout);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto parsed = sclerp::examples::parseArgs(argc, argv);
    if (parsed.flags.find("--help") != parsed.flags.end()) {
      printUsage(argv[0]);
      return 0;
    }

    const auto args = sclerp::examples::parseCommonArgsOrThrow(parsed);
    sclerp::examples::ensureOutputDirOrThrow(args.out_dir);

    // 1) Load URDF -> ManipulatorModel.
    sclerp::urdf::LoadOptions uopt;
    uopt.base_link = args.base_link;
    uopt.tip_link = args.tip_link;
    uopt.collapse_fixed_joints = true;
    uopt.strict = true;

    const auto load = sclerp::urdf::loadManipulatorModelFromFile(args.urdf_path.string(), uopt);
    if (!sclerp::core::ok(load.status)) {
      throw std::runtime_error("loadManipulatorModelFromFile failed: " + load.message);
    }

    // 2) Build kinematics solver.
    const sclerp::core::KinematicsSolver solver(load.model);
    const int dof = solver.model().dof();

    // 3) Choose a safe q_init.
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(dof);
    if (args.q_init) {
      if (args.q_init->size() != static_cast<std::size_t>(dof)) {
        throw std::runtime_error("--q-init must have exactly " + std::to_string(dof) + " values; got " +
                                 std::to_string(args.q_init->size()));
      }
      for (int i = 0; i < dof; ++i) q_init[i] = args.q_init->at(static_cast<std::size_t>(i));
    } else {
      // Near-limit -> midpoint heuristic (avoids "q=0" joint limit issues on some robots).
      constexpr double kNearLimitEps = 1e-3;
      for (int i = 0; i < dof; ++i) {
        const auto& lim = solver.model().joint(i).limit;
        if (!lim.enabled) continue;
        if (!std::isfinite(lim.lower) || !std::isfinite(lim.upper)) continue;
        if (!(lim.lower <= lim.upper)) continue;

        const double q0 = 0.0;
        const bool outside = (q0 < lim.lower) || (q0 > lim.upper);
        const bool near_bound = (std::abs(q0 - lim.lower) < kNearLimitEps) ||
                                (std::abs(lim.upper - q0) < kNearLimitEps);
        if (outside || near_bound) {
          q_init[i] = 0.5 * (lim.lower + lim.upper);
        }
      }
    }

    // 4) Build planning request (start pose -> translated goal pose).
    sclerp::core::MotionPlanRequest req;
    req.q_init = q_init;

    const auto st_fk = solver.forwardKinematics(req.q_init, &req.g_i);
    if (!sclerp::core::ok(st_fk)) throw std::runtime_error("forwardKinematics failed");

    req.g_f = req.g_i;
    req.g_f.translation() += Eigen::Vector3d(args.dx, args.dy, args.dz);

    const auto result = sclerp::core::planMotionSclerp(solver, req);
    if (!sclerp::core::ok(result.status)) {
      throw std::runtime_error("planMotionSclerp failed");
    }

    const auto& path = result.path;

    // 5) Time-parameterize (TOTG on a uniform grid).
    sclerp::trajectory::Limits lim;
    lim.v_max = Eigen::VectorXd::Constant(dof, args.vmax);
    lim.a_max = Eigen::VectorXd::Constant(dof, args.amax);

    sclerp::trajectory::GridTotg::Config cfg;
    cfg.unwrap_angles = true;

    const sclerp::trajectory::GridTotg totg(lim, cfg);
    sclerp::trajectory::PlannedTrajectory traj;
    const auto st_totg = totg.plan(path, args.sample_dt, &traj, sclerp::trajectory::continuousRevoluteMask(solver.model()));
    if (!sclerp::core::ok(st_totg)) throw std::runtime_error("GridTotg::plan failed");

    const std::filesystem::path csv_path = args.out_dir / "trajectory.csv";
    const auto st_csv = sclerp::trajectory::writeTrajectoryCsv(traj, csv_path.string(), sclerp::trajectory::CsvMode::PositionOnly);
    if (!sclerp::core::ok(st_csv)) throw std::runtime_error("writeTrajectoryCsv failed");

    // 6) Export a playback-ready Gazebo world with the robot + trajectory plugin.
    sclerp::gazebo::WorldRegistry reg;
    sclerp::gazebo::WorldExportOptions opt;
    opt.world_name = args.world_name;

    sclerp::gazebo::RobotModelFromUrdf robot;
    robot.urdf_path = args.urdf_path.string();
    robot.base_link = args.base_link;
    robot.tip_link = args.tip_link;
    robot.collapse_fixed_joints = true;
    robot.link_mesh_stl_directory = args.stl_dir.string();
    opt.robot_from_urdf = robot;

    sclerp::gazebo::JointTrajectoryPlayback playback;
    playback.csv_path = csv_path.string();
    playback.loop = args.loop;
    playback.rate = args.rate;
    opt.joint_trajectory = playback;

    if (args.plugin_dir) {
      opt.joint_trajectory_plugin_filename = (args.plugin_dir->empty())
          ? opt.joint_trajectory_plugin_filename
          : (args.plugin_dir.value() / opt.joint_trajectory_plugin_filename).string();
    }

    const std::filesystem::path world_path = args.out_dir / "world.sdf";
    const auto st_world = reg.writeSdfWorld(world_path.string(), opt);
    if (!sclerp::core::ok(st_world)) {
      throw std::runtime_error("WorldRegistry::writeSdfWorld failed");
    }

    std::cout << "Wrote:\n"
              << "  world:      " << world_path << "\n"
              << "  trajectory: " << csv_path << "\n\n";

    std::cout << "Run Gazebo:\n";
    if (!args.plugin_dir) {
      std::cout << "  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=\"<sclerp_build>/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}\"\n";
    }
    std::cout << "  ign gazebo \"" << world_path << "\"\n";

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[example_1_no_obstacles_gazebo] ERROR: " << e.what() << "\n";
    return 1;
  }
}
