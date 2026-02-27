#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sclerp::gazebo {
namespace detail {

static double toSeconds(const std::chrono::steady_clock::duration& d) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

static inline std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) s.remove_prefix(1);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
  return s;
}

static std::vector<std::string> splitCsvSimple(const std::string& line) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (start <= line.size()) {
    const std::size_t comma = line.find(',', start);
    const std::size_t end = (comma == std::string::npos) ? line.size() : comma;
    std::string_view token(line.data() + start, end - start);
    token = trim(token);
    out.emplace_back(token);
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return out;
}

static bool iequals(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const char ca = a[i];
    const char cb = b[i];
    if (std::tolower(static_cast<unsigned char>(ca)) != std::tolower(static_cast<unsigned char>(cb))) return false;
  }
  return true;
}

struct Trajectory {
  std::vector<std::string> joint_names;
  std::vector<double> times;
  std::vector<std::vector<double>> positions;  // positions[frame][joint]
};

static std::optional<Trajectory> loadTrajectoryCsv(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: failed to open CSV: " << path << "\n";
    return std::nullopt;
  }

  std::string line;
  std::size_t line_no = 0;
  std::size_t header_line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    line = std::string(trim(line));
    if (line.empty()) continue;
    if (!line.empty() && line[0] == '#') continue;
    header_line_no = line_no;
    break;
  }
  if (line.empty()) {
    std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: CSV is empty: " << path << "\n";
    return std::nullopt;
  }

  const auto header = splitCsvSimple(line);
  if (header.size() < 2 || !iequals(header[0], "time")) {
    std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << header_line_no
              << ": expected header 'time,<joint...>'\n";
    return std::nullopt;
  }

  Trajectory traj;
  traj.joint_names.assign(header.begin() + 1, header.end());

  std::unordered_set<std::string> unique;
  for (const auto& j : traj.joint_names) {
    if (!unique.insert(j).second) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << header_line_no
                << ": duplicate joint name in CSV: " << j << "\n";
      return std::nullopt;
    }
  }

  while (std::getline(in, line)) {
    ++line_no;
    line = std::string(trim(line));
    if (line.empty()) continue;
    if (!line.empty() && line[0] == '#') continue;

    const auto cols = splitCsvSimple(line);
    if (cols.size() != header.size()) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                << ": malformed CSV row (expected " << header.size() << " cols): " << line << "\n";
      return std::nullopt;
    }

    double t = 0.0;
    try {
      t = std::stod(cols[0]);
    } catch (...) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                << ": invalid time value: " << cols[0] << "\n";
      return std::nullopt;
    }
    if (!std::isfinite(t)) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                << ": invalid time value (non-finite): " << cols[0] << "\n";
      return std::nullopt;
    }
    if (!traj.times.empty() && t < traj.times.back()) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                << ": time is not non-decreasing (t=" << t << " < prev=" << traj.times.back() << ")\n";
      return std::nullopt;
    }

    std::vector<double> q;
    q.reserve(traj.joint_names.size());
    for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
      double v = 0.0;
      try {
        v = std::stod(cols[i + 1]);
      } catch (...) {
        std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                  << ": invalid value for joint '" << traj.joint_names[i] << "': " << cols[i + 1] << "\n";
        return std::nullopt;
      }
      if (!std::isfinite(v)) {
        std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: " << path << ":" << line_no
                  << ": invalid value for joint '" << traj.joint_names[i] << "' (non-finite): " << cols[i + 1]
                  << "\n";
        return std::nullopt;
      }
      q.push_back(v);
    }

    traj.times.push_back(t);
    traj.positions.push_back(std::move(q));
  }

  if (traj.times.empty()) {
    std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: no trajectory rows in CSV: " << path << "\n";
    return std::nullopt;
  }

  return traj;
}

}  // namespace detail

class JointTrajectoryPlayer final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager&,
                 gz::sim::EventManager&) override {
    this->world_entity_ = entity;

    if (!sdf) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: missing SDF element\n";
      return;
    }

    if (sdf->HasElement("robot_model")) {
      this->robot_model_name_ = sdf->Get<std::string>("robot_model");
    }
    if (sdf->HasElement("trajectory_csv")) {
      this->trajectory_csv_ = sdf->Get<std::string>("trajectory_csv");
    }
    if (sdf->HasElement("loop")) {
      this->loop_ = sdf->Get<bool>("loop");
    }
    if (sdf->HasElement("rate")) {
      this->rate_ = sdf->Get<double>("rate");
    }

    if (this->robot_model_name_.empty()) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: <robot_model> is required\n";
      return;
    }
    if (this->trajectory_csv_.empty()) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: <trajectory_csv> is required\n";
      return;
    }
    if (!(this->rate_ > 0.0) || !std::isfinite(this->rate_)) {
      std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: <rate> must be > 0\n";
      return;
    }

    auto maybe = detail::loadTrajectoryCsv(this->trajectory_csv_);
    if (!maybe) return;
    this->traj_ = std::move(*maybe);

    this->enabled_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (!this->enabled_) return;
    if (this->traj_.times.empty()) return;

    if (!this->robot_model_entity_) {
      this->tryResolveRobot(ecm);
      if (!this->robot_model_entity_) return;
    }

    if (this->joint_entities_.empty()) {
      if (!this->tryResolveJoints(ecm)) {
        this->enabled_ = false;
        return;
      }
    }

    if (!this->start_time_sec_) this->start_time_sec_ = detail::toSeconds(info.simTime);
    const double t = (detail::toSeconds(info.simTime) - *this->start_time_sec_) * this->rate_;

    const double t_end = this->traj_.times.back();
    if (t > t_end) {
      if (!this->loop_) {
        // Hold final frame.
        this->applyFrame(ecm, this->traj_.positions.size() - 1);
        this->enabled_ = false;
        return;
      }
      // Restart.
      this->start_time_sec_ = detail::toSeconds(info.simTime);
      this->frame_index_ = 0;
      this->last_applied_ = std::nullopt;
      this->applyFrame(ecm, 0);
      return;
    }

    // Find the last frame with time <= t (times are assumed non-decreasing).
    while ((this->frame_index_ + 1) < this->traj_.times.size() &&
           this->traj_.times[this->frame_index_ + 1] <= t) {
      ++this->frame_index_;
    }

    // While paused, don't advance, but still ensure we apply the current frame.
    this->applyFrame(ecm, this->frame_index_);
  }

private:
  void tryResolveRobot(gz::sim::EntityComponentManager& ecm) {
    const auto entities = gz::sim::entitiesFromScopedName(this->robot_model_name_, ecm, this->world_entity_);
    for (const auto& e : entities) {
      gz::sim::Model m(e);
      if (m.Valid(ecm)) {
        this->robot_model_entity_ = e;
        this->robot_model_ = m;
        return;
      }
    }
  }

  bool tryResolveJoints(gz::sim::EntityComponentManager& ecm) {
    if (!this->robot_model_entity_) return false;
    for (const auto& name : this->traj_.joint_names) {
      const auto joint_entity = this->robot_model_.JointByName(ecm, name);
      if (joint_entity == gz::sim::kNullEntity) {
        std::cerr << "[sclerp_gazebo] JointTrajectoryPlayer: joint not found on model '" << this->robot_model_name_
                  << "': " << name << "\n";
        return false;
      }
      this->joint_entities_.push_back(joint_entity);
    }
    return true;
  }

  void applyFrame(gz::sim::EntityComponentManager& ecm, std::size_t frame) {
    if (this->last_applied_ && *this->last_applied_ == frame) return;
    if (frame >= this->traj_.positions.size()) return;

    const auto& q = this->traj_.positions[frame];
    if (q.size() != this->joint_entities_.size()) return;

    for (std::size_t i = 0; i < this->joint_entities_.size(); ++i) {
      gz::sim::Joint j(this->joint_entities_[i]);
      j.ResetPosition(ecm, {q[i]});
    }

    this->last_applied_ = frame;
  }

private:
  bool enabled_{false};

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};

  std::string robot_model_name_;
  std::string trajectory_csv_;
  bool loop_{false};
  double rate_{1.0};

  detail::Trajectory traj_;

  std::optional<gz::sim::Entity> robot_model_entity_;
  gz::sim::Model robot_model_{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> joint_entities_;

  std::optional<double> start_time_sec_;
  std::size_t frame_index_{0};
  std::optional<std::size_t> last_applied_;
};

}  // namespace sclerp::gazebo

IGNITION_ADD_PLUGIN(sclerp::gazebo::JointTrajectoryPlayer,
                    gz::sim::System,
                    gz::sim::ISystemConfigure,
                    gz::sim::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(sclerp::gazebo::JointTrajectoryPlayer,
                          "sclerp::gazebo::JointTrajectoryPlayer")
