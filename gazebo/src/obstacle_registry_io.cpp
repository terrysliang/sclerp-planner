#include "sclerp/gazebo/obstacle_registry.hpp"

#include "sclerp/core/common/logger.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace sclerp::gazebo {
namespace {

using sclerp::core::LogLevel;
using sclerp::core::Transform;
using sclerp::core::log;
using sclerp::core::ok;

struct GeometrySpec {
  enum class Kind : std::uint8_t { Box = 0, Sphere = 1, Cylinder = 2, Plane = 3, Mesh = 4 };

  Kind kind{Kind::Box};

  // Box
  Vec3 box_size{Vec3::Zero()};

  // Sphere
  double sphere_radius{0.0};

  // Cylinder (SDF expects length along z)
  double cylinder_radius{0.0};
  double cylinder_length{0.0};

  // Plane: n dot x = offset
  Vec3 plane_normal{Vec3::UnitZ()};
  double plane_offset{0.0};

  // Mesh
  std::string mesh_uri;
};

static inline bool validOut(const void* out, const char* msg) {
  if (out) return true;
  log(LogLevel::Error, msg);
  return false;
}

static std::string jsonEscape(std::string_view s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '\"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c));
          out += oss.str();
        } else {
          out += c;
        }
    }
  }
  return out;
}

static std::string xmlEscape(std::string_view s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '\"': out += "&quot;"; break;
      case '\'': out += "&apos;"; break;
      default: out += c; break;
    }
  }
  return out;
}

static std::string sanitizeSdfName(std::string_view name) {
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '-') {
      out.push_back(c);
    } else if (std::isspace(uc)) {
      out.push_back('_');
    } else {
      out.push_back('_');
    }
  }
  if (out.empty()) out = "object";
  return out;
}

static std::string makeUniqueName(std::string base, std::unordered_set<std::string>* used) {
  if (!used) return base;
  if (used->insert(base).second) return base;
  for (int i = 1; i < 1000000; ++i) {
    std::string cand = base + "_" + std::to_string(i);
    if (used->insert(cand).second) return cand;
  }
  // Fallback: give up on uniqueness guarantee after a lot of attempts.
  return base;
}

static Vec3 rpyFromRot(const Mat3& R) {
  // SDF `<pose>` uses roll-pitch-yaw such that:
  //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // which matches URDF's fixed-axis rpy convention.
  //
  // This implementation follows the common ZYX (yaw-pitch-roll) decomposition.
  const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  const bool singular = sy < 1e-12;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  pitch = std::atan2(-R(2, 0), sy);
  if (!singular) {
    yaw = std::atan2(R(1, 0), R(0, 0));
    roll = std::atan2(R(2, 1), R(2, 2));
  } else {
    // Gimbal lock: choose roll = 0 and solve yaw from another pair of terms.
    yaw = std::atan2(-R(0, 1), R(1, 1));
    roll = 0.0;
  }

  return Vec3(roll, pitch, yaw);
}

static std::string sdfUriFromPath(const std::string& path) {
  if (path.rfind("model://", 0) == 0) return path;
  if (path.rfind("file://", 0) == 0) return path;
  if (!path.empty() && path.front() == '/') return std::string("file://") + path;
  return path;
}

static std::string pathFromSdfUri(std::string_view uri) {
  if (uri.rfind("file://", 0) == 0) {
    return std::string(uri.substr(std::string_view("file://").size()));
  }
  return std::string(uri);
}

static Mat4 mat4FromPose(const gz::math::Pose3d& p) {
  const auto& pos = p.Pos();
  const auto& rot = p.Rot();
  const Eigen::Quaterniond q(rot.W(), rot.X(), rot.Y(), rot.Z());

  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = Vec3(pos.X(), pos.Y(), pos.Z());
  return T;
}

static Transform poseToEigen(const ::urdf::Pose& p) {
  Transform T = Transform::Identity();
  T.translation() = Vec3(p.position.x, p.position.y, p.position.z);

  double rr = 0.0, rp = 0.0, ry = 0.0;
  p.rotation.getRPY(rr, rp, ry);

  const Eigen::AngleAxisd Rx(rr, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd Ry(rp, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd Rz(ry, Eigen::Vector3d::UnitZ());
  // URDF rpy: fixed-axis roll-pitch-yaw, applied in X-Y-Z order
  T.linear() = (Rz * Ry * Rx).toRotationMatrix();
  return T;
}

static bool isIdentityMat4(const Mat4& T, double tol = 1e-12) {
  return T.isApprox(Mat4::Identity(), tol);
}

static Status readFileToString(const std::string& path, std::string* out) {
  if (!validOut(out, "readFileToString: null output")) return Status::InvalidParameter;
  std::ifstream ifs(path);
  if (!ifs) return Status::Failure;
  std::ostringstream oss;
  oss << ifs.rdbuf();
  *out = oss.str();
  return Status::Success;
}

// Declared later in this file (used by URDF chain helpers). Forward-declare for mesh auto-discovery helpers.
static inline bool isMovableUrdfJointType(decltype(::urdf::Joint().type) t);

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

static std::string urdfMeshFilenameFromGeometry(const ::urdf::GeometrySharedPtr& geom) {
  if (!geom) return {};
  if (geom->type != ::urdf::Geometry::MESH) return {};
  const auto* mesh = dynamic_cast<const ::urdf::Mesh*>(geom.get());
  if (!mesh) return {};
  return mesh->filename;
}

static std::string urdfMeshFilenameForLink(const ::urdf::LinkConstSharedPtr& link) {
  if (!link) return {};

  // Prefer visual geometry (looks nicer in Gazebo), fall back to collision.
  if (link->visual) {
    const std::string f = urdfMeshFilenameFromGeometry(link->visual->geometry);
    if (!f.empty()) return f;
  }
  for (const auto& v : link->visual_array) {
    const std::string f = v ? urdfMeshFilenameFromGeometry(v->geometry) : std::string{};
    if (!f.empty()) return f;
  }

  if (link->collision) {
    const std::string f = urdfMeshFilenameFromGeometry(link->collision->geometry);
    if (!f.empty()) return f;
  }
  for (const auto& c : link->collision_array) {
    const std::string f = c ? urdfMeshFilenameFromGeometry(c->geometry) : std::string{};
    if (!f.empty()) return f;
  }

  return {};
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

static bool hasFixedAfterLastMovable(const std::vector<::urdf::JointConstSharedPtr>& joints_base_to_tip) {
  int last_movable = -1;
  for (std::size_t i = 0; i < joints_base_to_tip.size(); ++i) {
    const auto& j = joints_base_to_tip[i];
    if (!j) continue;
    if (isMovableUrdfJointType(j->type)) last_movable = static_cast<int>(i);
  }

  if (last_movable < 0) {
    // No movable joints in the chain: any joint implies a distinct (fixed) tip frame.
    return !joints_base_to_tip.empty();
  }
  return static_cast<std::size_t>(last_movable + 1) < joints_base_to_tip.size();
}

static bool resolveLinkMeshStl(const ::urdf::ModelInterfaceSharedPtr& model,
                               const std::string& link_name,
                               const std::unordered_map<std::string, std::filesystem::path>& stl_by_filename_lower,
                               const std::unordered_map<std::string, std::filesystem::path>& stl_by_stem_lower,
                               std::filesystem::path* out_path) {
  if (!out_path) return false;
  out_path->clear();

  if (model) {
    const ::urdf::LinkConstSharedPtr link = model->getLink(link_name);
    const std::string mesh_filename = urdfMeshFilenameForLink(link);
    if (!mesh_filename.empty()) {
      const std::string base = toLowerAscii(basenameFromUriOrPath(mesh_filename));
      const auto it = stl_by_filename_lower.find(base);
      if (it != stl_by_filename_lower.end()) {
        *out_path = it->second;
        return true;
      }
    }
  }

  const std::string key = toLowerAscii(link_name);
  const auto it2 = stl_by_stem_lower.find(key);
  if (it2 != stl_by_stem_lower.end()) {
    *out_path = it2->second;
    return true;
  }

  return false;
}

static Status discoverRobotLinkMeshesFromStlDirectory(const ::urdf::ModelInterfaceSharedPtr& model,
                                                      const std::vector<::urdf::JointConstSharedPtr>& chain,
                                                      const RobotModelFromUrdf& robot,
                                                      std::vector<std::string>* out_mesh_files) {
  if (!validOut(out_mesh_files, "discoverRobotLinkMeshesFromStlDirectory: null out_mesh_files")) {
    return Status::InvalidParameter;
  }
  out_mesh_files->clear();

  if (robot.link_mesh_stl_directory.empty()) {
    log(LogLevel::Error, "discoverRobotLinkMeshesFromStlDirectory: link_mesh_stl_directory is empty");
    return Status::InvalidParameter;
  }

  namespace fs = std::filesystem;
  const fs::path stl_dir(robot.link_mesh_stl_directory);

  std::unordered_map<std::string, fs::path> stl_by_filename_lower;
  std::unordered_map<std::string, fs::path> stl_by_stem_lower;
  const Status idx_st = indexStlDirectory(stl_dir, &stl_by_filename_lower, &stl_by_stem_lower);
  if (!ok(idx_st)) return idx_st;

  std::vector<std::string> required_links;
  required_links.reserve(1 + chain.size());
  required_links.push_back(robot.base_link);
  for (const auto& j : chain) {
    if (!j) continue;
    if (robot.collapse_fixed_joints && j->type == ::urdf::Joint::FIXED) continue;
    required_links.push_back(j->child_link_name);
  }

  out_mesh_files->reserve(required_links.size() + 1);

  for (const auto& link_name : required_links) {
    fs::path mesh_path;
    if (!resolveLinkMeshStl(model, link_name, stl_by_filename_lower, stl_by_stem_lower, &mesh_path)) {
      log(LogLevel::Error, ("discoverRobotLinkMeshesFromStlDirectory: missing STL for link: " + link_name).c_str());
      return Status::InvalidParameter;
    }
    out_mesh_files->push_back(mesh_path.string());
  }

  // Optional tool mesh (when the exporter will emit a tool link).
  const bool tool_offset_specified = !isIdentityMat4(robot.tool_offset);
  const bool fixed_after_movable = robot.collapse_fixed_joints ? hasFixedAfterLastMovable(chain) : false;

  if (tool_offset_specified || fixed_after_movable) {
    const std::string tool_link_name = tool_offset_specified ? (robot.tip_link + "_tool") : robot.tip_link;
    fs::path tool_mesh;
    if (resolveLinkMeshStl(model, tool_link_name, stl_by_filename_lower, stl_by_stem_lower, &tool_mesh)) {
      out_mesh_files->push_back(tool_mesh.string());
    } else {
      log(LogLevel::Warn,
          ("discoverRobotLinkMeshesFromStlDirectory: no STL found for tool link (optional): " + tool_link_name).c_str());
    }
  }

  return Status::Success;
}

static Status describeFcl(const sclerp::collision::FclObject& obj,
                          GeometrySpec* geom_out,
                          Pose* pose_out) {
  if (!validOut(geom_out, "describeFcl: null geom_out")) return Status::InvalidParameter;
  if (!validOut(pose_out, "describeFcl: null pose_out")) return Status::InvalidParameter;

  const auto& co = obj.collisionObject();
  const auto geom_ptr = co.collisionGeometry();
  if (!geom_ptr) {
    log(LogLevel::Error, "describeFcl: null collisionGeometry()");
    return Status::Failure;
  }

  Pose pose;
  pose.position = co.getTranslation();
  pose.orientation = co.getRotation();

  GeometrySpec geom;

  if (const auto* box = dynamic_cast<const fcl::Boxd*>(geom_ptr.get())) {
    geom.kind = GeometrySpec::Kind::Box;
    geom.box_size = box->side;
    *geom_out = std::move(geom);
    *pose_out = pose;
    return Status::Success;
  }
  if (const auto* sph = dynamic_cast<const fcl::Sphered*>(geom_ptr.get())) {
    geom.kind = GeometrySpec::Kind::Sphere;
    geom.sphere_radius = sph->radius;
    *geom_out = std::move(geom);
    *pose_out = pose;
    return Status::Success;
  }
  if (const auto* cyl = dynamic_cast<const fcl::Cylinderd*>(geom_ptr.get())) {
    geom.kind = GeometrySpec::Kind::Cylinder;
    geom.cylinder_radius = cyl->radius;
    geom.cylinder_length = cyl->lz;
    *geom_out = std::move(geom);
    *pose_out = pose;
    return Status::Success;
  }
  if (const auto* pl = dynamic_cast<const fcl::Planed*>(geom_ptr.get())) {
    geom.kind = GeometrySpec::Kind::Plane;
    geom.plane_normal = pl->n;
    geom.plane_offset = pl->d;

    // In FCL, plane is defined by n·x = d (no pose). In SDF, we express that as:
    // - link pose at p = n*d (identity orientation)
    // - plane normal = n (in link frame)
    const double nn = geom.plane_normal.norm();
    if (!(nn > 0.0) || !std::isfinite(nn)) {
      log(LogLevel::Error, "describeFcl: plane normal is invalid");
      return Status::Failure;
    }
    geom.plane_normal /= nn;
    pose.position = geom.plane_normal * geom.plane_offset;
    pose.orientation = Mat3::Identity();

    *geom_out = std::move(geom);
    *pose_out = pose;
    return Status::Success;
  }

  if (dynamic_cast<const fcl::BVHModel<fcl::OBBRSS<double>>*>(geom_ptr.get())) {
    geom.kind = GeometrySpec::Kind::Mesh;
    const auto* mesh_obj = dynamic_cast<const sclerp::collision::MeshObject*>(&obj);
    if (mesh_obj) geom.mesh_uri = mesh_obj->stlPath();
    *geom_out = std::move(geom);
    *pose_out = pose;
    return Status::Success;
  }

  log(LogLevel::Error, "describeFcl: unsupported obstacle geometry type");
  return Status::Failure;
}

static void writeSdfGeometry(std::ostream& os, const GeometrySpec& g) {
  os << "      <geometry>\n";
  switch (g.kind) {
    case GeometrySpec::Kind::Box:
      os << "        <box><size>" << g.box_size.x() << " " << g.box_size.y() << " " << g.box_size.z()
         << "</size></box>\n";
      break;
    case GeometrySpec::Kind::Sphere:
      os << "        <sphere><radius>" << g.sphere_radius << "</radius></sphere>\n";
      break;
    case GeometrySpec::Kind::Cylinder:
      os << "        <cylinder><radius>" << g.cylinder_radius << "</radius><length>" << g.cylinder_length
         << "</length></cylinder>\n";
      break;
    case GeometrySpec::Kind::Plane:
      // Provide a finite visualization size. Physics plane is infinite in contact, but visuals need a size.
      os << "        <plane><normal>" << g.plane_normal.x() << " " << g.plane_normal.y() << " " << g.plane_normal.z()
         << "</normal><size>100 100</size></plane>\n";
      break;
    case GeometrySpec::Kind::Mesh: {
      const std::string uri = sdfUriFromPath(g.mesh_uri);
      os << "        <mesh><uri>" << xmlEscape(uri) << "</uri></mesh>\n";
      break;
    }
  }
  os << "      </geometry>\n";
}

static inline bool isMovableUrdfJointType(decltype(::urdf::Joint().type) t) {
  return t == ::urdf::Joint::REVOLUTE ||
         t == ::urdf::Joint::CONTINUOUS ||
         t == ::urdf::Joint::PRISMATIC;
}

static inline bool isSupportedUrdfJointType(decltype(::urdf::Joint().type) t) {
  return isMovableUrdfJointType(t) || t == ::urdf::Joint::FIXED;
}

static Status buildChainJointsBaseToTip(const ::urdf::ModelInterfaceSharedPtr& model,
                                       const std::string& base_link,
                                       const std::string& tip_link,
                                       std::vector<::urdf::JointConstSharedPtr>* joints_base_to_tip) {
  if (!validOut(joints_base_to_tip, "buildChainJointsBaseToTip: null output")) return Status::InvalidParameter;
  joints_base_to_tip->clear();

  if (!model) {
    log(LogLevel::Error, "buildChainJointsBaseToTip: null URDF model");
    return Status::InvalidParameter;
  }

  ::urdf::LinkConstSharedPtr cur = model->getLink(tip_link);
  if (!cur) {
    log(LogLevel::Error, "buildChainJointsBaseToTip: tip_link not found");
    return Status::InvalidParameter;
  }

  std::vector<::urdf::JointConstSharedPtr> joints_tip_to_base;
  while (cur && cur->name != base_link) {
    if (!cur->parent_joint) {
      log(LogLevel::Error, "buildChainJointsBaseToTip: base_link is not an ancestor of tip_link");
      return Status::InvalidParameter;
    }
    joints_tip_to_base.push_back(cur->parent_joint);
    cur = cur->getParent();
  }
  if (!cur) {
    log(LogLevel::Error, "buildChainJointsBaseToTip: base_link is not an ancestor of tip_link");
    return Status::InvalidParameter;
  }

  std::reverse(joints_tip_to_base.begin(), joints_tip_to_base.end());
  for (const auto& j : joints_tip_to_base) {
    if (!j) continue;
    if (!isSupportedUrdfJointType(j->type)) {
      log(LogLevel::Error, "buildChainJointsBaseToTip: unsupported URDF joint type");
      return Status::InvalidParameter;
    }
    joints_base_to_tip->push_back(j);
  }

  return Status::Success;
}

static void writeSimpleInertial(std::ostream& out, double mass, double inertia_diagonal) {
  out << "        <inertial>\n";
  out << "          <pose>0 0 0 0 0 0</pose>\n";
  out << "          <mass>" << mass << "</mass>\n";
  out << "          <inertia>\n";
  out << "            <ixx>" << inertia_diagonal << "</ixx>\n";
  out << "            <ixy>0</ixy>\n";
  out << "            <ixz>0</ixz>\n";
  out << "            <iyy>" << inertia_diagonal << "</iyy>\n";
  out << "            <iyz>0</iyz>\n";
  out << "            <izz>" << inertia_diagonal << "</izz>\n";
  out << "          </inertia>\n";
  out << "        </inertial>\n";
}

static void writeLinkMesh(std::ostream& out, const std::string& stl_path, const Mat4& mesh_offset) {
  const std::string uri = sdfUriFromPath(stl_path);

  auto writePoseIfNeeded = [&](const char* indent) {
    if (isIdentityMat4(mesh_offset)) return;
    const Transform T = sclerp::core::transformFromMatrix4(mesh_offset);
    const Vec3 p = T.translation();
    const Vec3 rpy = rpyFromRot(T.rotation());
    out << indent << "<pose>"
        << p.x() << " " << p.y() << " " << p.z() << " "
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "</pose>\n";
  };

  out << "        <collision name=\"collision\">\n";
  writePoseIfNeeded("          ");
  out << "          <geometry>\n";
  out << "            <mesh><uri>" << xmlEscape(uri) << "</uri></mesh>\n";
  out << "          </geometry>\n";
  out << "        </collision>\n";

  out << "        <visual name=\"visual\">\n";
  writePoseIfNeeded("          ");
  out << "          <geometry>\n";
  out << "            <mesh><uri>" << xmlEscape(uri) << "</uri></mesh>\n";
  out << "          </geometry>\n";
  out << "          <material>\n";
  out << "            <ambient>0.7 0.7 0.7 1</ambient>\n";
  out << "            <diffuse>0.7 0.7 0.7 1</diffuse>\n";
  out << "          </material>\n";
  out << "        </visual>\n";
}

static Status writeRobotModelFromUrdf(std::ostream& out, const RobotModelFromUrdf& robot) {
  if (robot.urdf_path.empty()) {
    log(LogLevel::Error, "writeRobotModelFromUrdf: urdf_path is empty");
    return Status::InvalidParameter;
  }
  if (robot.base_link.empty() || robot.tip_link.empty()) {
    log(LogLevel::Error, "writeRobotModelFromUrdf: base_link / tip_link must be set");
    return Status::InvalidParameter;
  }

  std::string urdf_string;
  const Status read_st = readFileToString(robot.urdf_path, &urdf_string);
  if (!ok(read_st)) {
    log(LogLevel::Error, "writeRobotModelFromUrdf: failed to read URDF");
    return read_st;
  }

  const ::urdf::ModelInterfaceSharedPtr model = ::urdf::parseURDF(urdf_string);
  if (!model) {
    log(LogLevel::Error, "writeRobotModelFromUrdf: failed to parse URDF");
    return Status::Failure;
  }

  std::vector<::urdf::JointConstSharedPtr> chain;
  const Status chain_st = buildChainJointsBaseToTip(model, robot.base_link, robot.tip_link, &chain);
  if (!ok(chain_st)) return chain_st;

  std::vector<std::string> link_mesh_files = robot.link_mesh_stl_files;
  if (link_mesh_files.empty() && !robot.link_mesh_stl_directory.empty()) {
    const Status st = discoverRobotLinkMeshesFromStlDirectory(model, chain, robot, &link_mesh_files);
    if (!ok(st)) return st;
  }

  // Track used names for tool joint/link generation.
  std::unordered_set<std::string> used_link_names;
  std::unordered_set<std::string> used_joint_names;

  const auto makeUnique = [](const std::string& base, std::unordered_set<std::string>& used) -> std::string {
    if (used.insert(base).second) return base;
    for (int i = 1; i < 1000000; ++i) {
      std::string cand = base + "_" + std::to_string(i);
      if (used.insert(cand).second) return cand;
    }
    return base;
  };

  const auto meshPathAt = [&](std::size_t i) -> std::string {
    if (i >= link_mesh_files.size()) return {};
    return link_mesh_files[i];
  };
  const auto meshOffsetAt = [&](std::size_t i) -> Mat4 {
    if (robot.mesh_offset_transforms.size() == link_mesh_files.size() &&
        i < robot.mesh_offset_transforms.size()) {
      return robot.mesh_offset_transforms[i];
    }
    return Mat4::Identity();
  };

  const Vec3 model_rpy = rpyFromRot(robot.pose.orientation);
  out << "    <model name=\"" << xmlEscape(robot.name) << "\">\n";
  out << "      <static>" << (robot.static_model ? "true" : "false") << "</static>\n";
  out << "      <pose>"
      << robot.pose.position.x() << " " << robot.pose.position.y() << " " << robot.pose.position.z() << " "
      << model_rpy.x() << " " << model_rpy.y() << " " << model_rpy.z()
      << "</pose>\n";

  std::size_t mesh_index = 0;
  const std::size_t mesh_count = link_mesh_files.size();

  // Base link
  const std::string base_link_name = makeUnique(robot.base_link, used_link_names);
  out << "      <link name=\"" << xmlEscape(base_link_name) << "\">\n";
  out << "        <pose>0 0 0 0 0 0</pose>\n";
  out << "        <gravity>false</gravity>\n";
  writeSimpleInertial(out, robot.link_mass, robot.inertia_diagonal);
  if (mesh_index < mesh_count) {
    writeLinkMesh(out, meshPathAt(mesh_index), meshOffsetAt(mesh_index));
    ++mesh_index;
  }
  out << "      </link>\n";
  used_link_names.insert(base_link_name);

  std::string parent_link = base_link_name;
  Transform collapsed = Transform::Identity();

  auto writeJointAndChildLink = [&](const ::urdf::JointConstSharedPtr& joint,
                                    const std::string& child_link_name,
                                    const Transform& parent_T_joint) {
    std::string joint_type = "fixed";
    if (joint->type == ::urdf::Joint::REVOLUTE) joint_type = "revolute";
    else if (joint->type == ::urdf::Joint::CONTINUOUS) joint_type = "revolute";
    else if (joint->type == ::urdf::Joint::PRISMATIC) joint_type = "prismatic";

    const Vec3 p = parent_T_joint.translation();
    const Vec3 rpy = rpyFromRot(parent_T_joint.rotation());

    const std::string joint_name = makeUnique(joint->name, used_joint_names);
    out << "      <joint name=\"" << xmlEscape(joint_name) << "\" type=\"" << xmlEscape(joint_type) << "\">\n";
    out << "        <pose relative_to=\"" << xmlEscape(parent_link) << "\">"
        << p.x() << " " << p.y() << " " << p.z() << " "
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "</pose>\n";
    out << "        <parent>" << xmlEscape(parent_link) << "</parent>\n";
    out << "        <child>" << xmlEscape(child_link_name) << "</child>\n";

    if (joint->type == ::urdf::Joint::REVOLUTE ||
        joint->type == ::urdf::Joint::CONTINUOUS ||
        joint->type == ::urdf::Joint::PRISMATIC) {
      out << "        <axis>\n";
      out << "          <xyz>" << joint->axis.x << " " << joint->axis.y << " " << joint->axis.z << "</xyz>\n";
      out << "        </axis>\n";
    }
    out << "      </joint>\n";

    out << "      <link name=\"" << xmlEscape(child_link_name) << "\">\n";
    out << "        <pose relative_to=\"" << xmlEscape(joint_name) << "\">0 0 0 0 0 0</pose>\n";
    out << "        <gravity>false</gravity>\n";
    writeSimpleInertial(out, robot.link_mass, robot.inertia_diagonal);
    if (mesh_index < mesh_count) {
      writeLinkMesh(out, meshPathAt(mesh_index), meshOffsetAt(mesh_index));
      ++mesh_index;
    }
    out << "      </link>\n";

    parent_link = child_link_name;
  };

  for (const auto& joint : chain) {
    if (!joint) continue;

    if (robot.collapse_fixed_joints && joint->type == ::urdf::Joint::FIXED) {
      collapsed = collapsed * poseToEigen(joint->parent_to_joint_origin_transform);
      continue;
    }

    const std::string child_link_name = makeUnique(joint->child_link_name, used_link_names);
    const Transform parent_T_joint = collapsed * poseToEigen(joint->parent_to_joint_origin_transform);
    collapsed = Transform::Identity();
    writeJointAndChildLink(joint, child_link_name, parent_T_joint);
    used_link_names.insert(child_link_name);
  }

  // Optional tool link (uses remaining collapsed transform and optional explicit tool offset).
  const bool tool_offset_specified = !isIdentityMat4(robot.tool_offset);
  const bool fixed_after_movable =
      robot.collapse_fixed_joints ? hasFixedAfterLastMovable(chain) : false;
  Transform parent_T_tool = collapsed;
  if (tool_offset_specified) {
    parent_T_tool = parent_T_tool * sclerp::core::transformFromMatrix4(robot.tool_offset);
  }

  if (tool_offset_specified || fixed_after_movable || mesh_index < mesh_count) {
    std::string tool_link_name = robot.tip_link;
    if (tool_offset_specified) tool_link_name = robot.tip_link + "_tool";
    tool_link_name = makeUnique(tool_link_name, used_link_names);
    used_link_names.insert(tool_link_name);

    const std::string tool_joint_name = makeUnique("tool_fixed", used_joint_names);
    used_joint_names.insert(tool_joint_name);

    const Vec3 p = parent_T_tool.translation();
    const Vec3 rpy = rpyFromRot(parent_T_tool.rotation());

    out << "      <joint name=\"" << xmlEscape(tool_joint_name) << "\" type=\"fixed\">\n";
    out << "        <pose relative_to=\"" << xmlEscape(parent_link) << "\">"
        << p.x() << " " << p.y() << " " << p.z() << " "
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "</pose>\n";
    out << "        <parent>" << xmlEscape(parent_link) << "</parent>\n";
    out << "        <child>" << xmlEscape(tool_link_name) << "</child>\n";
    out << "      </joint>\n";

    out << "      <link name=\"" << xmlEscape(tool_link_name) << "\">\n";
    out << "        <pose relative_to=\"" << xmlEscape(tool_joint_name) << "\">0 0 0 0 0 0</pose>\n";
    out << "        <gravity>false</gravity>\n";
    writeSimpleInertial(out, robot.link_mass, robot.inertia_diagonal);
    if (mesh_index < mesh_count) {
      writeLinkMesh(out, meshPathAt(mesh_index), meshOffsetAt(mesh_index));
    }
    out << "      </link>\n";
  }

  out << "    </model>\n";
  return Status::Success;
}

static Status importGeometry(const sdf::Geometry& geom,
                             const Mat4& world_T_col,
                             std::shared_ptr<sclerp::collision::FclObject>* out) {
  if (!validOut(out, "importGeometry: null output")) return Status::InvalidParameter;
  *out = {};

  const Vec3 p = world_T_col.block<3, 1>(0, 3);
  const Mat3 R = world_T_col.block<3, 3>(0, 0);

  if (geom.Type() == sdf::GeometryType::BOX) {
    const sdf::Box* box = geom.BoxShape();
    if (!box) return Status::Failure;
    const auto& s = box->Size();
    return sclerp::collision::createBox(Vec3(s.X(), s.Y(), s.Z()), p, R, out);
  }

  if (geom.Type() == sdf::GeometryType::SPHERE) {
    const sdf::Sphere* s = geom.SphereShape();
    if (!s) return Status::Failure;
    return sclerp::collision::createSphere(s->Radius(), p, R, out);
  }

  if (geom.Type() == sdf::GeometryType::CYLINDER) {
    const sdf::Cylinder* c = geom.CylinderShape();
    if (!c) return Status::Failure;
    return sclerp::collision::createCylinder(c->Radius(), c->Length(), p, R, out);
  }

  if (geom.Type() == sdf::GeometryType::PLANE) {
    const sdf::Plane* pl = geom.PlaneShape();
    if (!pl) return Status::Failure;
    const auto& n = pl->Normal();
    const Vec3 n_link(n.X(), n.Y(), n.Z());
    if (n_link.norm() <= 0.0 || !n_link.allFinite()) {
      log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: invalid plane normal");
      return Status::InvalidParameter;
    }
    const Vec3 n_world = R * (n_link.normalized());
    const double d = n_world.dot(p);
    return sclerp::collision::createPlane(n_world, d, out);
  }

  if (geom.Type() == sdf::GeometryType::MESH) {
    const sdf::Mesh* mesh = geom.MeshShape();
    if (!mesh) return Status::Failure;
    const auto& sc = mesh->Scale();
    if (std::abs(sc.X() - 1.0) > 1e-12 || std::abs(sc.Y() - 1.0) > 1e-12 || std::abs(sc.Z() - 1.0) > 1e-12) {
      log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: mesh <scale> is not supported (must be 1 1 1)");
      return Status::InvalidParameter;
    }

    const std::string uri = mesh->Uri();
    if (uri.rfind("model://", 0) == 0) {
      log(LogLevel::Error,
          "ObstacleRegistry::loadFromSdfWorld: mesh URI uses model://; save the world with resources_use_absolute_paths=true");
      return Status::InvalidParameter;
    }

    const std::string path = pathFromSdfUri(uri);
    return sclerp::collision::createMeshFromSTL(path, world_T_col, out);
  }

  log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: unsupported geometry type");
  return Status::InvalidParameter;
}

static std::string sdfErrorsToString(const sdf::Errors& errs) {
  std::ostringstream oss;
  for (const auto& e : errs) {
    oss << "- [" << static_cast<int>(e.Code()) << "] " << e.Message() << "\n";
  }
  return oss.str();
}

}  // namespace

Status ObstacleRegistry::writeJson(const std::string& json_path) const {
  std::ofstream out(json_path);
  if (!out) {
    log(LogLevel::Error, "ObstacleRegistry::writeJson: failed to open output file");
    return Status::Failure;
  }

  out << std::setprecision(17);
  out << "{\n";
  out << "  \"version\": 1,\n";
  out << "  \"obstacles\": [\n";

  for (std::size_t i = 0; i < obstacles_.size(); ++i) {
    if (!obstacles_[i]) {
      log(LogLevel::Error, "ObstacleRegistry::writeJson: null obstacle entry");
      return Status::Failure;
    }

    GeometrySpec geom;
    Pose pose;
    const Status st = describeFcl(*obstacles_[i], &geom, &pose);
    if (!ok(st)) return st;

    const Vec3 rpy = rpyFromRot(pose.orientation);

    out << "    {\n";
    out << "      \"name\": \"" << jsonEscape(obstacle_names_[i]) << "\",\n";
    out << "      \"pose\": {\n";
    out << "        \"position\": [" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
        << "],\n";
    out << "        \"rpy\": [" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]\n";
    out << "      },\n";

    out << "      \"geometry\": {\n";
    switch (geom.kind) {
      case GeometrySpec::Kind::Box:
        out << "        \"type\": \"box\",\n";
        out << "        \"size\": [" << geom.box_size.x() << ", " << geom.box_size.y() << ", " << geom.box_size.z()
            << "]\n";
        break;
      case GeometrySpec::Kind::Sphere:
        out << "        \"type\": \"sphere\",\n";
        out << "        \"radius\": " << geom.sphere_radius << "\n";
        break;
      case GeometrySpec::Kind::Cylinder:
        out << "        \"type\": \"cylinder\",\n";
        out << "        \"radius\": " << geom.cylinder_radius << ",\n";
        out << "        \"length\": " << geom.cylinder_length << "\n";
        break;
      case GeometrySpec::Kind::Plane:
        out << "        \"type\": \"plane\",\n";
        out << "        \"normal\": [" << geom.plane_normal.x() << ", " << geom.plane_normal.y() << ", "
            << geom.plane_normal.z() << "],\n";
        out << "        \"offset\": " << geom.plane_offset << "\n";
        break;
      case GeometrySpec::Kind::Mesh:
        out << "        \"type\": \"mesh\",\n";
        out << "        \"uri\": \"" << jsonEscape(geom.mesh_uri) << "\"\n";
        break;
    }
    out << "      }\n";
    out << "    }" << (i + 1 < obstacles_.size() ? "," : "") << "\n";
  }

  out << "  ]\n";
  out << "}\n";
  return Status::Success;
}

Status ObstacleRegistry::writeSdfWorld(const std::string& sdf_path, const WorldExportOptions& opt) const {
  std::ofstream out(sdf_path);
  if (!out) {
    log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: failed to open output file");
    return Status::Failure;
  }

  out << std::setprecision(17);
  out << "<?xml version=\"1.0\"?>\n";
  out << "<sdf version=\"1.7\">\n";
  out << "  <world name=\"" << xmlEscape(opt.world_name) << "\">\n";

  if (opt.joint_trajectory || opt.world_sdf_saver || opt.primitive_resizer) {
    // When an SDF world loads at least one system plugin successfully, Gazebo Sim does *not* auto-load
    // its default systems. Since the export can add system plugins, explicitly include the minimum
    // default systems needed for GUI visualization and interaction.
    out << "    <plugin filename=\"libignition-gazebo-physics-system.so\" name=\"gz::sim::systems::Physics\"/>\n";
    out << "    <plugin filename=\"libignition-gazebo-user-commands-system.so\" name=\"gz::sim::systems::UserCommands\"/>\n";
    out << "    <plugin filename=\"libignition-gazebo-scene-broadcaster-system.so\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n";
  }

  if (opt.include_sun) {
    out << "    <light name=\"sun\" type=\"directional\">\n";
    out << "      <pose>0 0 10 0 0 0</pose>\n";
    out << "      <diffuse>0.8 0.8 0.8 1</diffuse>\n";
    out << "      <specular>0.2 0.2 0.2 1</specular>\n";
    out << "      <direction>-0.5 0.1 -0.9</direction>\n";
    out << "      <attenuation>\n";
    out << "        <range>1000</range>\n";
    out << "        <constant>0.9</constant>\n";
    out << "        <linear>0.01</linear>\n";
    out << "        <quadratic>0.001</quadratic>\n";
    out << "      </attenuation>\n";
    out << "      <cast_shadows>true</cast_shadows>\n";
    out << "    </light>\n";
  }

  if (opt.include_ground_plane) {
    out << "    <model name=\"ground_plane\">\n";
    out << "      <static>true</static>\n";
    out << "      <link name=\"link\">\n";
    out << "        <collision name=\"collision\">\n";
    out << "          <geometry>\n";
    out << "            <plane><normal>0 0 1</normal><size>100 100</size></plane>\n";
    out << "          </geometry>\n";
    out << "        </collision>\n";
    out << "        <visual name=\"visual\">\n";
    out << "          <geometry>\n";
    out << "            <plane><normal>0 0 1</normal><size>100 100</size></plane>\n";
    out << "          </geometry>\n";
    out << "          <material>\n";
    out << "            <ambient>0.8 0.8 0.8 1</ambient>\n";
    out << "            <diffuse>0.8 0.8 0.8 1</diffuse>\n";
    out << "          </material>\n";
    out << "        </visual>\n";
    out << "      </link>\n";
    out << "    </model>\n";
  }

  if (opt.robot && opt.robot_from_urdf) {
    log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: set only one of robot or robot_from_urdf");
    return Status::InvalidParameter;
  }

  std::optional<std::string> robot_name_for_plugin;
  if (opt.robot_from_urdf) {
    const Status st = writeRobotModelFromUrdf(out, *opt.robot_from_urdf);
    if (!ok(st)) return st;
    robot_name_for_plugin = opt.robot_from_urdf->name;
  } else if (opt.robot) {
    const auto& r = *opt.robot;
    const Vec3 rpy = rpyFromRot(r.pose.orientation);
    out << "    <include>\n";
    out << "      <uri>" << xmlEscape(r.uri) << "</uri>\n";
    out << "      <name>" << xmlEscape(r.name) << "</name>\n";
    if (r.static_model) {
      out << "      <static>" << (*r.static_model ? "true" : "false") << "</static>\n";
    }
    out << "      <pose>"
        << r.pose.position.x() << " " << r.pose.position.y() << " " << r.pose.position.z() << " "
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "</pose>\n";
    out << "    </include>\n";
    robot_name_for_plugin = r.name;
  }

  if (opt.joint_trajectory) {
    if (!robot_name_for_plugin) {
      log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: joint_trajectory requires a robot (robot or robot_from_urdf)");
      return Status::InvalidParameter;
    }
    const auto& jt = *opt.joint_trajectory;
    if (jt.csv_path.empty()) {
      log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: joint_trajectory csv_path is empty");
      return Status::InvalidParameter;
    }
    if (!(jt.rate > 0.0) || !std::isfinite(jt.rate)) {
      log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: joint_trajectory rate must be > 0");
      return Status::InvalidParameter;
    }

    out << "    <plugin filename=\"" << xmlEscape(opt.joint_trajectory_plugin_filename)
        << "\" name=\"" << xmlEscape(opt.joint_trajectory_plugin_name) << "\">\n";
    out << "      <robot_model>" << xmlEscape(*robot_name_for_plugin) << "</robot_model>\n";
    out << "      <trajectory_csv>" << xmlEscape(jt.csv_path) << "</trajectory_csv>\n";
    out << "      <loop>" << (jt.loop ? "true" : "false") << "</loop>\n";
    out << "      <rate>" << jt.rate << "</rate>\n";
    out << "    </plugin>\n";
  }

  if (opt.world_sdf_saver) {
    const auto& saver = *opt.world_sdf_saver;
    out << "    <plugin filename=\"" << xmlEscape(opt.world_sdf_saver_plugin_filename)
        << "\" name=\"" << xmlEscape(opt.world_sdf_saver_plugin_name) << "\">\n";
    out << "      <output_path>" << xmlEscape(saver.output_path) << "</output_path>\n";
    out << "      <timeout_ms>" << saver.timeout_ms << "</timeout_ms>\n";
    out << "      <expand_include_tags>" << (saver.expand_include_tags ? "true" : "false")
        << "</expand_include_tags>\n";
    out << "      <resources_use_absolute_paths>" << (saver.resources_use_absolute_paths ? "true" : "false")
        << "</resources_use_absolute_paths>\n";
    out << "      <copy_model_resources>" << (saver.copy_model_resources ? "true" : "false")
        << "</copy_model_resources>\n";
    out << "    </plugin>\n";
  }

  if (opt.primitive_resizer) {
    out << "    <plugin filename=\"" << xmlEscape(opt.primitive_resizer_plugin_filename)
        << "\" name=\"" << xmlEscape(opt.primitive_resizer_plugin_name) << "\"/>\n";
  }

  for (std::size_t i = 0; i < obstacles_.size(); ++i) {
    if (!obstacles_[i]) {
      log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: null obstacle entry");
      return Status::Failure;
    }

    GeometrySpec geom;
    Pose pose;
    const Status st = describeFcl(*obstacles_[i], &geom, &pose);
    if (!ok(st)) return st;

    // Mesh URIs are required for visualization.
    if (geom.kind == GeometrySpec::Kind::Mesh && geom.mesh_uri.empty()) {
      log(LogLevel::Error, "ObstacleRegistry::writeSdfWorld: mesh obstacle is missing STL path");
      return Status::InvalidParameter;
    }

    const Vec3 rpy = rpyFromRot(pose.orientation);

    out << "    <model name=\"" << xmlEscape(obstacle_names_[i]) << "\">\n";
    out << "      <static>true</static>\n";
    out << "      <pose>"
        << pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " "
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "</pose>\n";
    out << "      <link name=\"link\">\n";
    out << "        <collision name=\"collision\">\n";
    writeSdfGeometry(out, geom);
    out << "        </collision>\n";
    out << "        <visual name=\"visual\">\n";
    writeSdfGeometry(out, geom);
    out << "          <material>\n";
    out << "            <ambient>0.7 0.7 0.7 1</ambient>\n";
    out << "            <diffuse>0.7 0.7 0.7 1</diffuse>\n";
    out << "          </material>\n";
    out << "        </visual>\n";
    out << "      </link>\n";
    out << "    </model>\n";
  }

  out << "  </world>\n";
  out << "</sdf>\n";

  return Status::Success;
}

Status ObstacleRegistry::loadFromSdfWorld(const std::string& sdf_path, const SdfWorldImportOptions& opt) {
  if (sdf_path.empty()) {
    log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: sdf_path is empty");
    return Status::InvalidParameter;
  }

  sdf::Root root;
  const sdf::Errors errors = root.Load(sdf_path);
  if (!errors.empty()) {
    std::ostringstream oss;
    oss << "ObstacleRegistry::loadFromSdfWorld: failed to load SDF world:\n";
    oss << sdfErrorsToString(errors);
    log(LogLevel::Error, oss.str());
    return Status::Failure;
  }

  const sdf::World* world = nullptr;
  if (root.WorldCount() > 0) world = root.WorldByIndex(0);
  if (!world) {
    log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: no <world> in SDF");
    return Status::Failure;
  }

  std::unordered_set<std::string> ignore;
  ignore.reserve(opt.ignore_models.size());
  for (const auto& n : opt.ignore_models) ignore.insert(n);

  std::vector<std::shared_ptr<sclerp::collision::FclObject>> new_obstacles;
  std::vector<std::string> new_names;
  std::unordered_set<std::string> used_names;

  const auto importModel = [&](const sdf::Model& model,
                               const Mat4& world_T_parent,
                               const std::string& scoped_name,
                               const auto& self) -> Status {
    if (opt.static_only && !model.Static()) return Status::Success;

    Mat4 parent_T_model = mat4FromPose(model.RawPose());
    {
      gz::math::Pose3d resolved;
      const sdf::Errors pose_errors = model.SemanticPose().Resolve(resolved);
      if (pose_errors.empty()) {
        parent_T_model = mat4FromPose(resolved);
      }
    }
    const Mat4 world_T_model = world_T_parent * parent_T_model;

    for (std::size_t li = 0; li < model.LinkCount(); ++li) {
      const sdf::Link* link = model.LinkByIndex(li);
      if (!link) continue;

      Mat4 model_T_link = mat4FromPose(link->RawPose());
      {
        gz::math::Pose3d resolved;
        const sdf::Errors pose_errors = link->SemanticPose().Resolve(resolved);
        if (pose_errors.empty()) {
          model_T_link = mat4FromPose(resolved);
        }
      }
      const Mat4 world_T_link = world_T_model * model_T_link;

      for (std::size_t ci = 0; ci < link->CollisionCount(); ++ci) {
        const sdf::Collision* col = link->CollisionByIndex(ci);
        if (!col) continue;
        const sdf::Geometry* geom = col->Geom();
        if (!geom) continue;

        Mat4 link_T_col = mat4FromPose(col->RawPose());
        {
          gz::math::Pose3d resolved;
          const sdf::Errors pose_errors = col->SemanticPose().Resolve(resolved);
          if (pose_errors.empty()) {
            link_T_col = mat4FromPose(resolved);
          }
        }
        const Mat4 world_T_col = world_T_link * link_T_col;

        std::shared_ptr<sclerp::collision::FclObject> obj;
        const Status st = importGeometry(*geom, world_T_col, &obj);
        if (!ok(st)) return st;
        if (!obj) {
          log(LogLevel::Error, "ObstacleRegistry::loadFromSdfWorld: failed to create obstacle");
          return Status::Failure;
        }
        obj->computeAABB();

        const std::string base =
          sanitizeSdfName(scoped_name) + "__" +
          sanitizeSdfName(link->Name()) + "__" +
          sanitizeSdfName(col->Name());
        const std::string unique = makeUniqueName(base, &used_names);

        new_obstacles.push_back(std::move(obj));
        new_names.push_back(unique);
      }
    }

    for (std::size_t mi = 0; mi < model.ModelCount(); ++mi) {
      const sdf::Model* child = model.ModelByIndex(mi);
      if (!child) continue;
      const std::string child_scoped = scoped_name + "::" + child->Name();
      const Status st = self(*child, world_T_model, child_scoped, self);
      if (!ok(st)) return st;
    }

    return Status::Success;
  };

  for (std::size_t i = 0; i < world->ModelCount(); ++i) {
    const sdf::Model* model = world->ModelByIndex(i);
    if (!model) continue;

    const std::string model_name = model->Name();
    if (ignore.find(model_name) != ignore.end()) continue;

    const Status st = importModel(*model, Mat4::Identity(), model_name, importModel);
    if (!ok(st)) return st;
  }

  this->obstacles_ = std::move(new_obstacles);
  this->obstacle_names_ = std::move(new_names);
  return Status::Success;
}

}  // namespace sclerp::gazebo
