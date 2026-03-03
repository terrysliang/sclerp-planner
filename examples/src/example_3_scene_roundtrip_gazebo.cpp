#include "example_common.hpp"

#include "sclerp/collision/collision.hpp"
#include "sclerp/collision/motion_plan_collision.hpp"
#include "sclerp/collision/robot_link_meshes.hpp"
#include "sclerp/core/kinematics/kinematics_solver.hpp"
#include "sclerp/core/planning/motion_plan.hpp"
#include "sclerp/gazebo/world_registry.hpp"
#include "sclerp/gazebo/world_sdf.hpp"
#include "sclerp/trajectory/interpolator.hpp"
#include "sclerp/urdf/load_manipulator.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

static void printUsage(const char* exe) {
  std::cout << "Example 3: load a Gazebo world SDF as a planning scene, plan with obstacles, time-parameterize, export playback world.\n\n"
            << "Usage:\n"
            << "  " << exe << " [--scene-world <path>] [--urdf <path> --stl-dir <dir> --base-link <name> --tip-link <name>] [options]\n\n"
            << "If robot args are omitted, uses bundled Diana7 assets.\n\n";
  std::cout << "Scene import:\n"
            << "  --scene-world <path>  Path to an SDF world containing obstacles (e.g., saved from Gazebo via WorldSdfSaver).\n"
            << "                        If omitted, this example generates a small demo scene SDF in-code.\n\n";
  sclerp::examples::printCommonCli(std::cout);
}

static void registerObstaclesOrThrow(sclerp::gazebo::WorldRegistry* reg,
                                     const std::vector<std::shared_ptr<sclerp::collision::FclObject>>& obstacles) {
  if (!reg) throw std::runtime_error("null WorldRegistry");
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    std::string name;
    if (i == 0) name = "obstacle_box";
    else if (i == 1) name = "obstacle_cylinder";
    else name = "obstacle_" + std::to_string(i);
    const auto st = reg->registerObstacle(obstacles[i], name);
    if (!sclerp::core::ok(st)) {
      throw std::runtime_error("WorldRegistry::registerObstacle failed for " + name);
    }
  }
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

    const auto scene_world_arg_it = parsed.values.find("--scene-world");
    const std::string scene_world_arg = (scene_world_arg_it != parsed.values.end() && !scene_world_arg_it->second.empty())
        ? scene_world_arg_it->second.front()
        : std::string{};

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

    // 5) Load a planning scene from a Gazebo SDF world (obstacles only).
    const std::filesystem::path scene_world_path =
        scene_world_arg.empty() ? (args.out_dir / "scene_generated.sdf") : std::filesystem::absolute(scene_world_arg);

    // If the user didn't provide a scene file, generate a small demo SDF scene in-code.
    if (scene_world_arg.empty()) {
      std::cout << "No --scene-world provided; generating a demo planning scene SDF: " << scene_world_path << "\n"
                << "Tip: to plan against a scene saved from Gazebo, pass: --scene-world <world_saved.sdf>\n\n";
      std::vector<std::shared_ptr<sclerp::collision::FclObject>> hardcoded;
      {
        const sclerp::core::Vec3 dims(1.0, 0.06, 0.29326);
        const sclerp::core::Vec3 pos(-0.2813, -0.5606, 0.14663);
        const double yaw = -std::acos(-1.0) / 4.0;  // -45 deg
        const sclerp::core::Mat3 R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        std::shared_ptr<sclerp::collision::FclObject> box;
        const auto st = sclerp::collision::createBox(dims, pos, R, &box);
        if (!sclerp::core::ok(st)) throw std::runtime_error("collision::createBox failed");
        box->computeAABB();
        hardcoded.push_back(std::move(box));
      }

      sclerp::gazebo::WorldRegistry reg_scene;
      registerObstaclesOrThrow(&reg_scene, hardcoded);
      const auto st = reg_scene.writeSdfWorld(scene_world_path.string(), sclerp::gazebo::WorldExportOptions{});
      if (!sclerp::core::ok(st)) throw std::runtime_error("Failed to write scene_generated.sdf");
    }

    sclerp::gazebo::WorldRegistry reg_loaded;
    {
      const auto st = reg_loaded.loadFromSdfWorld(scene_world_path.string());
      if (!sclerp::core::ok(st)) throw std::runtime_error("WorldRegistry::loadFromSdfWorld failed");
    }

    // 6) Plan with collision avoidance against imported obstacles.
    const auto& obstacles = reg_loaded.obstacles();

    std::vector<std::shared_ptr<sclerp::collision::FclObject>> link_meshes;
    std::vector<sclerp::core::Mat4> mesh_offset_transforms;
    {
      const sclerp::collision::RobotLinkMeshSpec mesh_spec{load.fk_frame_names, load.fk_frame_mesh_uris};
      const auto st = sclerp::collision::buildRobotLinkMeshesFromStlDirectory(solver,
                                                                              args.stl_dir,
                                                                              mesh_spec,
                                                                              &link_meshes,
                                                                              &mesh_offset_transforms);
      if (!sclerp::core::ok(st)) throw std::runtime_error("buildRobotLinkMeshesFromStlDirectory failed");
    }

    std::shared_ptr<sclerp::collision::FclObject> grasped_object;  // none

    const sclerp::collision::CollisionContext ctx{link_meshes, obstacles, grasped_object};
    sclerp::collision::CollisionScene scene{ctx, mesh_offset_transforms};

    sclerp::collision::CollisionMotionPlanOptions plan_opt;
    plan_opt.query.check_self_collision = false;
    plan_opt.query.num_links_ignore = 1;
    plan_opt.avoidance.safe_dist = 0.01;

    const auto result = sclerp::collision::planMotionSclerpWithCollision(solver, req, scene, plan_opt);
    if (!sclerp::core::ok(result.status)) throw std::runtime_error("planMotionSclerpWithCollision failed");

    const auto& path = result.path;

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

    // 7) Export a playback-ready world (imported obstacles + robot + JointTrajectoryPlayer plugin).
    const std::filesystem::path playback_world_path = args.out_dir / "world.sdf";
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
    const auto st_world = reg_loaded.writeSdfWorld(playback_world_path.string(), opt);
    if (!sclerp::core::ok(st_world)) throw std::runtime_error("WorldRegistry::writeSdfWorld failed");

    std::cout << "Wrote:\n"
              << "  scene:      " << scene_world_path << "\n"
              << "  world:      " << playback_world_path << "\n"
              << "  trajectory: " << csv_path << "\n\n";

    std::cout << "Run Gazebo:\n";
    if (!args.plugin_dir) {
      std::cout << "  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=\"<sclerp_build>/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}\"\n";
    }
    std::cout << "  ign gazebo \"" << playback_world_path << "\"\n";

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[example_3_scene_roundtrip_gazebo] ERROR: " << e.what() << "\n";
    return 1;
  }
}
