#include <cmath>
#include <cstring>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/core/path/joint_path.hpp"
#include "sclerp/gazebo/joint_trajectory_csv.hpp"
#include "sclerp/gazebo/world_registry.hpp"
#include "sclerp/gazebo/world_sdf.hpp"

using sclerp::core::LogLevel;
using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::setLogLevel;
using sclerp::gazebo::JointTrajectoryPlayback;
using sclerp::gazebo::RobotModelFromUrdf;
using sclerp::gazebo::SdfWorldImportOptions;
using sclerp::gazebo::WorldExportOptions;
using sclerp::gazebo::WorldRegistry;

namespace {

static bool parseFlag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) return true;
  }
  return false;
}

static std::string parseStringArg(int argc, char** argv, const char* key, std::string def = {}) {
  const std::string prefix = std::string(key) + "=";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0 && i + 1 < argc) return std::string(argv[i + 1]);
    if (std::strncmp(argv[i], prefix.c_str(), prefix.size()) == 0) return std::string(argv[i] + prefix.size());
  }
  return def;
}

static double parseDoubleArg(int argc, char** argv, const char* key, double def) {
  const std::string s = parseStringArg(argc, argv, key, "");
  if (s.empty()) return def;
  try {
    return std::stod(s);
  } catch (...) {
    return def;
  }
}

static std::string trim(const std::string& s) {
  std::size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  std::size_t j = s.size();
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) --j;
  return s.substr(i, j - i);
}

static std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string tok;
  std::stringstream ss(line);
  while (std::getline(ss, tok, ',')) out.push_back(trim(tok));
  return out;
}

static Status loadFailureDumpJointPathCsv(const std::filesystem::path& csv_path,
                                          sclerp::core::JointPath* out,
                                          std::string* err) {
  if (!out) return Status::InvalidParameter;

  std::ifstream in(csv_path.string());
  if (!in) {
    if (err) *err = "failed to open dump: " + csv_path.string();
    return Status::Failure;
  }

  std::vector<std::string> joint_names;
  std::vector<Eigen::VectorXd> positions;
  std::string raw;
  while (std::getline(in, raw)) {
    const std::string line = trim(raw);
    if (line.empty()) continue;
    if (line.rfind("#", 0) == 0) continue;

    if (joint_names.empty()) {
      joint_names = splitCsv(line);
      if (joint_names.empty()) {
        if (err) *err = "invalid dump header (empty)";
        return Status::InvalidParameter;
      }
      for (const auto& name : joint_names) {
        if (name.empty()) {
          if (err) *err = "invalid dump header (empty joint name)";
          return Status::InvalidParameter;
        }
      }
      continue;
    }

    const auto toks = splitCsv(line);
    if (toks.size() != joint_names.size()) {
      if (err) {
        std::ostringstream oss;
        oss << "invalid dump row: expected " << joint_names.size() << " values, got " << toks.size();
        *err = oss.str();
      }
      return Status::InvalidParameter;
    }

    Eigen::VectorXd q(static_cast<int>(joint_names.size()));
    for (std::size_t i = 0; i < toks.size(); ++i) {
      if (toks[i].empty()) {
        if (err) *err = "invalid dump row: empty value";
        return Status::InvalidParameter;
      }
      try {
        q(static_cast<int>(i)) = std::stod(toks[i]);
      } catch (...) {
        if (err) *err = "invalid dump row: failed to parse float";
        return Status::InvalidParameter;
      }
    }
    positions.push_back(std::move(q));
  }

  if (joint_names.empty()) {
    if (err) *err = "dump has no header";
    return Status::InvalidParameter;
  }
  if (positions.empty()) {
    if (err) *err = "dump has no waypoints";
    return Status::InvalidParameter;
  }

  out->joint_names = std::move(joint_names);
  out->positions = std::move(positions);
  return Status::Success;
}

static void printHelp() {
  std::cout << "Usage: sclerp_gazebo_visualize_failure_dump \\\n"
            << "  --dump <failure_dump.csv> --urdf <robot.urdf> --stl-dir <meshes_dir> \\\n"
            << "  --base-link <name> --tip-link <name> [options]\n\n"
            << "Options:\n"
            << "  --out-dir <dir>               Output directory (default: <dump_dir>/<dump_stem>_viz)\n"
            << "  --scene-world <world.sdf>     Optional scene world to import obstacles from\n"
            << "  --world-name <name>           World name (default: sclerp_world)\n"
            << "  --dt <sec>                    CSV timestep (default: 0.05)\n"
            << "  --rate <x>                    Playback rate multiplier (default: 1.0)\n"
            << "  --loop                        Loop playback\n"
            << "  --plugin-dir <dir>            Directory containing libsclerp_joint_trajectory_player.so\n"
            << "  --no-collapse-fixed-joints    Do not collapse fixed joints in the URDF chain\n"
            << "  --verbose                     Set log level to INFO\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
    printHelp();
    return 0;
  }

  if (parseFlag(argc, argv, "--verbose")) {
    setLogLevel(LogLevel::Info);
  }

  const std::string dump_path_s = parseStringArg(argc, argv, "--dump");
  const std::string urdf_path = parseStringArg(argc, argv, "--urdf");
  const std::string stl_dir = parseStringArg(argc, argv, "--stl-dir");
  const std::string base_link = parseStringArg(argc, argv, "--base-link");
  const std::string tip_link = parseStringArg(argc, argv, "--tip-link");

  if (dump_path_s.empty() || urdf_path.empty() || stl_dir.empty() || base_link.empty() || tip_link.empty()) {
    std::cerr << "Missing required arguments. Run with --help for usage.\n";
    return 2;
  }

  const std::filesystem::path dump_path = std::filesystem::absolute(std::filesystem::path(dump_path_s));
  if (!std::filesystem::is_regular_file(dump_path)) {
    std::cerr << "Dump not found: " << dump_path.string() << "\n";
    return 2;
  }

  std::filesystem::path out_dir;
  const std::string out_dir_s = parseStringArg(argc, argv, "--out-dir");
  if (!out_dir_s.empty()) {
    out_dir = std::filesystem::absolute(std::filesystem::path(out_dir_s));
  } else {
    out_dir = dump_path.parent_path() / (dump_path.stem().string() + "_viz");
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "Failed to create output dir: " << out_dir.string() << " (" << ec.message() << ")\n";
    return 1;
  }

  const double dt = parseDoubleArg(argc, argv, "--dt", 0.05);
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    std::cerr << "--dt must be > 0\n";
    return 2;
  }

  const double rate = parseDoubleArg(argc, argv, "--rate", 1.0);
  if (!(rate > 0.0) || !std::isfinite(rate)) {
    std::cerr << "--rate must be > 0\n";
    return 2;
  }
  const bool loop = parseFlag(argc, argv, "--loop");
  const std::string plugin_dir_s = parseStringArg(argc, argv, "--plugin-dir");

  sclerp::core::JointPath path;
  std::string parse_err;
  const Status st_load_dump = loadFailureDumpJointPathCsv(dump_path, &path, &parse_err);
  if (!ok(st_load_dump)) {
    std::cerr << "Failed to load failure dump: " << (parse_err.empty() ? dump_path.string() : parse_err) << "\n";
    return 1;
  }

  const std::filesystem::path traj_csv_path = out_dir / "trajectory.csv";
  const Status st_csv = sclerp::gazebo::writeJointTrajectoryCsv(path, dt, traj_csv_path.string());
  if (!ok(st_csv)) {
    std::cerr << "Failed to write trajectory.csv: " << traj_csv_path.string() << "\n";
    return 1;
  }

  WorldRegistry reg;
  const std::string scene_world_s = parseStringArg(argc, argv, "--scene-world");
  if (!scene_world_s.empty()) {
    const std::filesystem::path scene_world = std::filesystem::absolute(std::filesystem::path(scene_world_s));
    const Status st_scene = reg.loadFromSdfWorld(scene_world.string(), SdfWorldImportOptions{});
    if (!ok(st_scene)) {
      std::cerr << "Failed to load scene world: " << scene_world.string() << "\n";
      return 1;
    }
  }

  WorldExportOptions wopt;
  wopt.world_name = parseStringArg(argc, argv, "--world-name", "sclerp_world");

  RobotModelFromUrdf robot;
  robot.urdf_path = urdf_path;
  robot.base_link = base_link;
  robot.tip_link = tip_link;
  robot.link_mesh_stl_directory = stl_dir;
  robot.collapse_fixed_joints = !parseFlag(argc, argv, "--no-collapse-fixed-joints");
  robot.static_model = true;
  wopt.robot_from_urdf = robot;

  JointTrajectoryPlayback playback;
  playback.csv_path = traj_csv_path.string();
  playback.loop = loop;
  playback.rate = rate;
  wopt.joint_trajectory = playback;

  if (!plugin_dir_s.empty()) {
    const std::filesystem::path pdir = std::filesystem::absolute(std::filesystem::path(plugin_dir_s));
    wopt.joint_trajectory_plugin_filename = (pdir / wopt.joint_trajectory_plugin_filename).string();
  }

  const std::filesystem::path world_path = out_dir / "world.sdf";
  const Status st_world = reg.writeSdfWorld(world_path.string(), wopt);
  if (!ok(st_world)) {
    std::cerr << "Failed to write SDF world: " << world_path.string() << "\n";
    return 1;
  }

  std::cout << "Wrote:\n";
  std::cout << "  dump:       " << dump_path.string() << "\n";
  if (!scene_world_s.empty()) {
    std::cout << "  scene:      " << std::filesystem::absolute(std::filesystem::path(scene_world_s)).string() << "\n";
  }
  std::cout << "  world:      " << world_path.string() << "\n";
  std::cout << "  trajectory: " << traj_csv_path.string() << "\n";

  if (plugin_dir_s.empty()) {
    std::cout << "\nRun Gazebo:\n";
    std::cout << "  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=\"<sclerp_build>/gazebo:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}\"\n";
    std::cout << "  ign gazebo \"" << world_path.string() << "\"\n";
  } else {
    std::cout << "\nRun Gazebo:\n";
    std::cout << "  ign gazebo \"" << world_path.string() << "\"\n";
  }

  return 0;
}
