// Helpers for building per-link collision meshes from a directory of STLs.
#include "sclerp/collision/robot_link_meshes.hpp"

#include "sclerp/core/common/logger.hpp"

#include <cmath>
#include <cctype>
#include <filesystem>
#include <string_view>
#include <system_error>
#include <unordered_map>

namespace sclerp::collision {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::Status;
using sclerp::core::log;
using sclerp::core::ok;

static inline bool validOut(const void* out, const char* msg) {
  if (out) return true;
  log(LogLevel::Error, msg);
  return false;
}

static std::string toLowerAscii(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

static bool isStlFile(const std::filesystem::path& p) {
  const std::string ext = toLowerAscii(p.extension().string());
  return ext == ".stl";
}

static std::string basenameFromUriOrPath(std::string_view uri_or_path) {
  // Handles common URDF mesh filename conventions (relative paths, file://..., package://...).
  std::string_view s = uri_or_path;
  if (s.rfind("file://", 0) == 0) s = s.substr(std::string_view("file://").size());
  if (s.rfind("package://", 0) == 0) s = s.substr(std::string_view("package://").size());

  const std::size_t pos = s.find_last_of("/\\");
  if (pos == std::string_view::npos) return std::string(s);
  return std::string(s.substr(pos + 1));
}

static Status indexStlDirectory(const std::filesystem::path& stl_dir,
                                std::unordered_map<std::string, std::filesystem::path>* by_filename_lower,
                                std::unordered_map<std::string, std::filesystem::path>* by_stem_lower) {
  if (!validOut(by_filename_lower, "indexStlDirectory: null by_filename_lower")) return Status::InvalidParameter;
  if (!validOut(by_stem_lower, "indexStlDirectory: null by_stem_lower")) return Status::InvalidParameter;
  by_filename_lower->clear();
  by_stem_lower->clear();

  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(stl_dir, ec) || !fs::is_directory(stl_dir, ec)) {
    log(LogLevel::Error, "indexStlDirectory: stl_dir does not exist or is not a directory");
    return Status::InvalidParameter;
  }

  fs::recursive_directory_iterator it(stl_dir, fs::directory_options::skip_permission_denied, ec);
  fs::recursive_directory_iterator end;
  if (ec) {
    log(LogLevel::Error, "indexStlDirectory: failed to iterate stl_dir");
    return Status::Failure;
  }

  for (; it != end; it.increment(ec)) {
    if (ec) {
      ec.clear();
      continue;
    }

    const fs::directory_entry& e = *it;
    if (!e.is_regular_file(ec)) {
      ec.clear();
      continue;
    }

    const fs::path p = e.path();
    if (!isStlFile(p)) continue;

    const std::string filename_key = toLowerAscii(p.filename().string());
    const std::string stem_key = toLowerAscii(p.stem().string());
    if (filename_key.empty() || stem_key.empty()) continue;

    fs::path abs_p = fs::absolute(p, ec);
    if (ec) {
      abs_p = p;
      ec.clear();
    }

    auto ins_fn = by_filename_lower->emplace(filename_key, abs_p);
    if (!ins_fn.second && ins_fn.first->second != abs_p) {
      log(LogLevel::Error, "indexStlDirectory: duplicate STL filename in directory tree");
      return Status::InvalidParameter;
    }

    auto ins_stem = by_stem_lower->emplace(stem_key, abs_p);
    if (!ins_stem.second && ins_stem.first->second != abs_p) {
      log(LogLevel::Error, "indexStlDirectory: duplicate STL stem in directory tree");
      return Status::InvalidParameter;
    }
  }

  if (by_filename_lower->empty()) {
    log(LogLevel::Error, "indexStlDirectory: no .stl files found in directory");
    return Status::InvalidParameter;
  }

  return Status::Success;
}

}  // namespace

Status buildRobotLinkMeshesFromStlDirectory(const sclerp::core::KinematicsSolver& solver,
                                           const std::filesystem::path& stl_dir,
                                           const RobotLinkMeshSpec& spec,
                                           std::vector<std::shared_ptr<FclObject>>* out_link_meshes,
                                           std::vector<sclerp::core::Mat4>* out_mesh_offset_transforms,
                                           const RobotLinkMeshBuildOptions& opt) {
  if (!validOut(out_link_meshes, "buildRobotLinkMeshesFromStlDirectory: null out_link_meshes")) {
    return Status::InvalidParameter;
  }
  if (!validOut(out_mesh_offset_transforms, "buildRobotLinkMeshesFromStlDirectory: null out_mesh_offset_transforms")) {
    return Status::InvalidParameter;
  }

  out_link_meshes->clear();
  out_mesh_offset_transforms->clear();

  const int dof = solver.model().dof();
  if (dof <= 0) {
    log(LogLevel::Error, "buildRobotLinkMeshesFromStlDirectory: invalid dof");
    return Status::InvalidParameter;
  }

  const std::size_t expected_size = static_cast<std::size_t>(1 + dof + (solver.model().has_tool_frame() ? 1 : 0));
  if (spec.frame_names.size() != expected_size) {
    log(LogLevel::Error, "buildRobotLinkMeshesFromStlDirectory: spec.frame_names size mismatch");
    return Status::InvalidParameter;
  }
  if (!spec.frame_mesh_uris.empty() && spec.frame_mesh_uris.size() != spec.frame_names.size()) {
    log(LogLevel::Error, "buildRobotLinkMeshesFromStlDirectory: spec.frame_mesh_uris size mismatch");
    return Status::InvalidParameter;
  }
  if (solver.model().has_tool_frame()) {
    if (!(opt.tool_placeholder_radius > 0.0) || !std::isfinite(opt.tool_placeholder_radius)) {
      log(LogLevel::Error, "buildRobotLinkMeshesFromStlDirectory: tool_placeholder_radius must be > 0 and finite");
      return Status::InvalidParameter;
    }
  }

  std::unordered_map<std::string, std::filesystem::path> stl_by_filename_lower;
  std::unordered_map<std::string, std::filesystem::path> stl_by_stem_lower;
  const Status idx_st = indexStlDirectory(stl_dir, &stl_by_filename_lower, &stl_by_stem_lower);
  if (!ok(idx_st)) return idx_st;

  out_link_meshes->reserve(expected_size);
  out_mesh_offset_transforms->reserve(expected_size);

  for (std::size_t i = 0; i < expected_size; ++i) {
    const std::string& frame_name = spec.frame_names[i];
    if (frame_name.empty()) {
      log(LogLevel::Error, "buildRobotLinkMeshesFromStlDirectory: empty frame name");
      return Status::InvalidParameter;
    }

    std::filesystem::path mesh_path;
    bool found = false;

    if (!spec.frame_mesh_uris.empty()) {
      const std::string& uri = spec.frame_mesh_uris[i];
      if (!uri.empty()) {
        const std::string key = toLowerAscii(basenameFromUriOrPath(uri));
        const auto it = stl_by_filename_lower.find(key);
        if (it != stl_by_filename_lower.end()) {
          mesh_path = it->second;
          found = true;
        }
      }
    }

    if (!found) {
      const std::string key = toLowerAscii(frame_name);
      const auto it = stl_by_stem_lower.find(key);
      if (it != stl_by_stem_lower.end()) {
        mesh_path = it->second;
        found = true;
      }
    }

    const bool is_tool_mesh = solver.model().has_tool_frame() && (i + 1 == expected_size);

    std::shared_ptr<FclObject> mesh;
    if (!found) {
      if (is_tool_mesh) {
        const Status st = createSphere(opt.tool_placeholder_radius,
                                       sclerp::core::Vec3::Zero(),
                                       sclerp::core::Mat3::Identity(),
                                       &mesh);
        if (!ok(st)) return st;
      } else {
        log(LogLevel::Error, ("buildRobotLinkMeshesFromStlDirectory: missing STL for frame: " + frame_name).c_str());
        return Status::InvalidParameter;
      }
    } else {
      const Status st = createMeshFromSTL(mesh_path.string(), sclerp::core::Mat4::Identity(), &mesh);
      if (!ok(st)) return st;
    }

    if (mesh) mesh->computeAABB();
    out_link_meshes->push_back(std::move(mesh));
    out_mesh_offset_transforms->push_back(sclerp::core::Mat4::Identity());
  }

  return Status::Success;
}

}  // namespace sclerp::collision

