#include <cstring>
#include <iostream>
#include <string>

#include "sclerp/core/common/logger.hpp"
#include "sclerp/core/common/status.hpp"
#include "sclerp/gazebo/obstacle_registry.hpp"

using sclerp::core::LogLevel;
using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::setLogLevel;
using sclerp::gazebo::ObstacleRegistry;
using sclerp::gazebo::RobotModelFromUrdf;
using sclerp::gazebo::WorldExportOptions;

static bool parseFlag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) {
      return true;
    }
  }
  return false;
}

static std::string parseStringArg(int argc, char** argv, const char* key, std::string def = {}) {
  const std::string prefix = std::string(key) + "=";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0 && i + 1 < argc) {
      return std::string(argv[i + 1]);
    }
    if (std::strncmp(argv[i], prefix.c_str(), prefix.size()) == 0) {
      return std::string(argv[i] + prefix.size());
    }
  }
  return def;
}

static void printHelp() {
  std::cout << "Usage: sclerp_gazebo_generate_world \\\n";
  std::cout << "  --urdf <robot.urdf> --stl-dir <meshes_dir> --base-link <name> --tip-link <name> [options]\n\n";
  std::cout << "Options:\n";
  std::cout << "  --out <world.sdf>                 Output path (default: world.sdf)\n";
  std::cout << "  --robot-name <name>               Robot model name (default: robot)\n";
  std::cout << "  --world-name <name>               World name (default: sclerp_world)\n";
  std::cout << "  --no-ground-plane                 Omit ground plane\n";
  std::cout << "  --no-sun                          Omit sun\n";
  std::cout << "  --dynamic                         Export robot as non-static (default: static)\n";
  std::cout << "  --no-collapse-fixed-joints        Do not collapse fixed joints in the URDF chain\n";
  std::cout << "  --enable-world-sdf-saver          Emit the WorldSdfSaver plugin stanza\n";
  std::cout << "  --saved-world <path>              Default saved-world path (default: world_saved.sdf)\n";
  std::cout << "  --enable-primitive-resizer        Emit the PrimitiveResizer plugin stanza\n";
  std::cout << "  --verbose                         Set log level to INFO\n";
}

int main(int argc, char** argv) {
  if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
    printHelp();
    return 0;
  }

  if (parseFlag(argc, argv, "--verbose")) {
    setLogLevel(LogLevel::Info);
  }

  const std::string urdf_path = parseStringArg(argc, argv, "--urdf");
  const std::string stl_dir = parseStringArg(argc, argv, "--stl-dir");
  const std::string base_link = parseStringArg(argc, argv, "--base-link");
  const std::string tip_link = parseStringArg(argc, argv, "--tip-link");

  if (urdf_path.empty() || stl_dir.empty() || base_link.empty() || tip_link.empty()) {
    std::cerr << "Missing required arguments. Run with --help for usage.\n";
    return 2;
  }

  const std::string out_path = parseStringArg(argc, argv, "--out", "world.sdf");
  const std::string robot_name = parseStringArg(argc, argv, "--robot-name", "robot");
  const std::string world_name = parseStringArg(argc, argv, "--world-name", "sclerp_world");

  WorldExportOptions opt;
  opt.world_name = world_name;
  opt.include_ground_plane = !parseFlag(argc, argv, "--no-ground-plane");
  opt.include_sun = !parseFlag(argc, argv, "--no-sun");

  RobotModelFromUrdf robot;
  robot.name = robot_name;
  robot.urdf_path = urdf_path;
  robot.base_link = base_link;
  robot.tip_link = tip_link;
  robot.collapse_fixed_joints = !parseFlag(argc, argv, "--no-collapse-fixed-joints");
  robot.static_model = !parseFlag(argc, argv, "--dynamic");
  robot.link_mesh_stl_directory = stl_dir;
  opt.robot_from_urdf = robot;

  if (parseFlag(argc, argv, "--enable-world-sdf-saver")) {
    opt.world_sdf_saver = sclerp::gazebo::WorldSdfSaverOptions{};
    opt.world_sdf_saver->output_path = parseStringArg(argc, argv, "--saved-world", "world_saved.sdf");
  }

  if (parseFlag(argc, argv, "--enable-primitive-resizer")) {
    opt.primitive_resizer = sclerp::gazebo::PrimitiveResizerOptions{};
  }

  ObstacleRegistry reg;
  const Status st = reg.writeSdfWorld(out_path, opt);
  if (!ok(st)) {
    std::cerr << "Failed to write SDF world: " << out_path << "\n";
    return 1;
  }

  std::cout << "Wrote: " << out_path << "\n";
  return 0;
}

