#pragma once

#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sclerp::examples {

struct ParsedArgs {
  std::unordered_map<std::string, std::vector<std::string>> values;
  std::unordered_set<std::string> flags;
  std::vector<std::string> positionals;
};

ParsedArgs parseArgs(int argc, char** argv);

struct CommonArgs {
  std::filesystem::path urdf_path;
  std::filesystem::path stl_dir;
  std::filesystem::path out_dir{"sclerp_example_out"};
  std::optional<std::filesystem::path> plugin_dir;

  std::string base_link;
  std::string tip_link;

  // Optional explicit starting joint configuration (size must equal model dof).
  std::optional<std::vector<double>> q_init;

  std::string world_name{"sclerp_world"};

  double dx{-0.15};
  double dy{-0.184};
  double dz{0.059};

  double sample_dt{0.01};
  double vmax{1.0};
  double amax{2.0};

  bool loop{false};
  double rate{1.0};
};

void printCommonCli(std::ostream& os);
CommonArgs parseCommonArgsOrThrow(const ParsedArgs& args);

void ensureOutputDirOrThrow(const std::filesystem::path& out_dir);

}  // namespace sclerp::examples
