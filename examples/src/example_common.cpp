#include "example_common.hpp"

#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace sclerp::examples {
namespace {

static bool hasFlag(const ParsedArgs& args, const std::string& key) {
  return args.flags.find(key) != args.flags.end();
}

static std::optional<std::string> getOne(const ParsedArgs& args, const std::string& key) {
  const auto it = args.values.find(key);
  if (it == args.values.end() || it->second.empty()) return std::nullopt;
  return it->second.front();
}

static std::string requireOne(const ParsedArgs& args, const std::string& key) {
  const auto v = getOne(args, key);
  if (!v) {
    throw std::runtime_error("Missing required argument: " + key);
  }
  return *v;
}

static double parseDoubleOrThrow(const std::string& s, const std::string& key) {
  try {
    std::size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size()) throw std::runtime_error("");
    if (!std::isfinite(v)) throw std::runtime_error("");
    return v;
  } catch (...) {
    throw std::runtime_error("Invalid numeric value for " + key + ": " + s);
  }
}

static std::string joinCsv(const std::vector<std::string>& xs) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    if (i) oss << ", ";
    oss << xs[i];
  }
  return oss.str();
}

static std::string trimCopy(std::string s) {
  auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!s.empty() && isSpace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && isSpace(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

static std::vector<double> parseCsvDoublesOrThrow(const std::string& s, const std::string& key) {
  std::vector<double> out;
  std::string tok;
  std::istringstream iss(s);
  while (std::getline(iss, tok, ',')) {
    tok = trimCopy(tok);
    if (tok.empty()) {
      throw std::runtime_error("Invalid CSV value for " + key + ": empty element");
    }
    out.push_back(parseDoubleOrThrow(tok, key));
  }
  if (out.empty()) {
    throw std::runtime_error("Invalid CSV value for " + key + ": empty list");
  }
  return out;
}

static CommonArgs makeBundledDianaPresetOrThrow() {
  CommonArgs out;

#ifndef SCLERP_EXAMPLES_ASSETS_DIR
  throw std::runtime_error("Bundled demo assets are unavailable (SCLERP_EXAMPLES_ASSETS_DIR is not defined)");
#else
  const std::filesystem::path assets_dir = std::filesystem::path(SCLERP_EXAMPLES_ASSETS_DIR);
  if (assets_dir.empty()) {
    throw std::runtime_error("Bundled demo assets are unavailable (SCLERP_EXAMPLES_ASSETS_DIR is empty)");
  }

  out.urdf_path = std::filesystem::absolute(assets_dir / "diana7/urdf/diana_robot.urdf");
  out.stl_dir = std::filesystem::absolute(assets_dir / "diana7/meshes");
  out.base_link = "base_link";
  out.tip_link = "link7";
  out.q_init = std::vector<double>{1.886428995278953, 0.12622070334762633, 0.036244670128887949, 2.4665118153771788, -0.3020034302671557, -0.5708880960015188, 2.208232253505976};

  if (!std::filesystem::exists(out.urdf_path)) {
    throw std::runtime_error("Bundled demo URDF not found: " + out.urdf_path.string());
  }
  if (!std::filesystem::is_directory(out.stl_dir)) {
    throw std::runtime_error("Bundled demo STL directory not found: " + out.stl_dir.string());
  }

  return out;
#endif
}

}  // namespace

ParsedArgs parseArgs(int argc, char** argv) {
  ParsedArgs out;
  for (int i = 1; i < argc; ++i) {
    const std::string tok = argv[i] ? argv[i] : "";
    if (tok == "--help" || tok == "-h") {
      out.flags.insert("--help");
      continue;
    }

    if (tok.rfind("--", 0) == 0) {
      const std::string key = tok;
      const bool has_value = (i + 1 < argc) && (std::string(argv[i + 1]).rfind("--", 0) != 0);
      if (has_value) {
        out.values[key].push_back(argv[i + 1]);
        ++i;
      } else {
        out.flags.insert(key);
      }
      continue;
    }

    out.positionals.push_back(tok);
  }
  return out;
}

void printCommonCli(std::ostream& os) {
  os << "Common options:\n"
     << "  --urdf <path>         Robot URDF path\n"
     << "  --stl-dir <dir>       Directory with per-link STL meshes (named after URDF link names)\n"
     << "  --base-link <name>    Base link name\n"
     << "  --tip-link <name>     Tip link name\n"
     << "  --q-init <csv>        Start joint configuration (comma-separated, size must equal DOF)\n"
     << "  --out-dir <dir>       Output directory (default: sclerp_example_out)\n"
     << "  --world-name <name>   SDF world name (default: sclerp_world)\n"
     << "  --plugin-dir <dir>    Directory containing libsclerp_joint_trajectory_player.so (optional)\n"
     << "  --dx <m> --dy <m> --dz <m>   Goal translation applied to the start pose (default: dx=0.2)\n"
     << "  --sample-dt <s>       Trajectory sample dt (default: 0.01)\n"
     << "  --vmax <scalar>       Joint velocity limit (broadcast) (default: 1.0)\n"
     << "  --amax <scalar>       Joint acceleration limit (broadcast) (default: 2.0)\n"
     << "  --rate <scalar>       Playback speed multiplier in Gazebo (default: 1.0)\n"
     << "  --loop                Loop playback in Gazebo\n";
}

CommonArgs parseCommonArgsOrThrow(const ParsedArgs& args) {
  const bool has_any_robot_arg = getOne(args, "--urdf") || getOne(args, "--stl-dir") ||
                                 getOne(args, "--base-link") || getOne(args, "--tip-link");

  CommonArgs out;
  if (!has_any_robot_arg) {
    out = makeBundledDianaPresetOrThrow();
  } else {
    std::vector<std::string> missing;
    if (!getOne(args, "--urdf")) missing.push_back("--urdf");
    if (!getOne(args, "--stl-dir")) missing.push_back("--stl-dir");
    if (!getOne(args, "--base-link")) missing.push_back("--base-link");
    if (!getOne(args, "--tip-link")) missing.push_back("--tip-link");
    if (!missing.empty()) {
      throw std::runtime_error("When providing robot arguments, all of --urdf, --stl-dir, --base-link, --tip-link must be set. Missing: " +
                               joinCsv(missing));
    }

    out.urdf_path = std::filesystem::absolute(requireOne(args, "--urdf"));
    out.stl_dir = std::filesystem::absolute(requireOne(args, "--stl-dir"));
    out.base_link = requireOne(args, "--base-link");
    out.tip_link = requireOne(args, "--tip-link");
  }

  if (const auto v = getOne(args, "--q-init")) out.q_init = parseCsvDoublesOrThrow(*v, "--q-init");

  if (const auto v = getOne(args, "--out-dir")) out.out_dir = std::filesystem::absolute(*v);
  else out.out_dir = std::filesystem::absolute(out.out_dir);
  if (const auto v = getOne(args, "--world-name")) out.world_name = *v;
  if (const auto v = getOne(args, "--plugin-dir")) out.plugin_dir = std::filesystem::absolute(*v);

  if (const auto v = getOne(args, "--dx")) out.dx = parseDoubleOrThrow(*v, "--dx");
  if (const auto v = getOne(args, "--dy")) out.dy = parseDoubleOrThrow(*v, "--dy");
  if (const auto v = getOne(args, "--dz")) out.dz = parseDoubleOrThrow(*v, "--dz");

  if (const auto v = getOne(args, "--sample-dt")) out.sample_dt = parseDoubleOrThrow(*v, "--sample-dt");
  if (const auto v = getOne(args, "--vmax")) out.vmax = parseDoubleOrThrow(*v, "--vmax");
  if (const auto v = getOne(args, "--amax")) out.amax = parseDoubleOrThrow(*v, "--amax");

  if (const auto v = getOne(args, "--rate")) out.rate = parseDoubleOrThrow(*v, "--rate");
  out.loop = hasFlag(args, "--loop");

  if (!(out.sample_dt > 0.0)) throw std::runtime_error("--sample-dt must be > 0");
  if (!(out.vmax > 0.0)) throw std::runtime_error("--vmax must be > 0");
  if (!(out.amax > 0.0)) throw std::runtime_error("--amax must be > 0");
  if (!(out.rate > 0.0)) throw std::runtime_error("--rate must be > 0");

  return out;
}

void ensureOutputDirOrThrow(const std::filesystem::path& out_dir) {
  if (out_dir.empty()) throw std::runtime_error("out_dir is empty");
  try {
    std::filesystem::create_directories(out_dir);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to create out_dir: ") + e.what());
  }
}

}  // namespace sclerp::examples
