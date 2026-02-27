#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
// NOLINTNEXTLINE(modernize-deprecated-headers)
#include <gz/msgs/boolean.pb.h>
// NOLINTNEXTLINE(modernize-deprecated-headers)
#include <gz/msgs/stringmsg.pb.h>

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Sphere.hh>
#include <sdf/Visual.hh>

#include <cmath>
#include <cctype>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sclerp::gazebo {
namespace detail {

static inline std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r' || s.front() == '\n')) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) {
    s.remove_suffix(1);
  }
  return s;
}

static std::vector<std::string> splitWs(std::string_view s) {
  s = trim(s);
  std::vector<std::string> out;
  while (!s.empty()) {
    std::size_t i = 0;
    while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    out.emplace_back(s.substr(0, i));
    s.remove_prefix(i);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
  }
  return out;
}

static std::string toLower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

struct ResizeCommand {
  enum class Kind { Box = 0, Sphere = 1, Cylinder = 2 };
  Kind kind{Kind::Box};
  std::string model_name;
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

static bool parseCommand(const std::string& cmd, ResizeCommand* out, std::string* err) {
  if (!out || !err) return false;
  *out = ResizeCommand{};
  err->clear();

  const auto toks = splitWs(cmd);
  if (toks.empty() || toLower(toks[0]) == "help" || toks[0] == "?") {
    *err = "usage: <box|sphere|cylinder> <model_name> <params...>  "
           "(box: x y z | sphere: r | cylinder: r length)";
    return false;
  }

  const std::string kind = toLower(toks[0]);
  if (toks.size() < 3) {
    *err = "resize_primitive: expected at least 3 tokens (type, model_name, params)";
    return false;
  }

  out->model_name = toks[1];
  auto parseDouble = [&](std::size_t idx, double* v) -> bool {
    try {
      *v = std::stod(toks[idx]);
    } catch (...) {
      return false;
    }
    return std::isfinite(*v);
  };

  if (kind == "box") {
    if (toks.size() != 5) {
      *err = "resize_primitive: box expects: box <model_name> <x> <y> <z>";
      return false;
    }
    out->kind = ResizeCommand::Kind::Box;
    if (!parseDouble(2, &out->a) || !parseDouble(3, &out->b) || !parseDouble(4, &out->c)) {
      *err = "resize_primitive: invalid numeric value for box size";
      return false;
    }
    if (!(out->a > 0.0) || !(out->b > 0.0) || !(out->c > 0.0)) {
      *err = "resize_primitive: box size must be > 0";
      return false;
    }
    return true;
  }

  if (kind == "sphere") {
    if (toks.size() != 3) {
      *err = "resize_primitive: sphere expects: sphere <model_name> <radius>";
      return false;
    }
    out->kind = ResizeCommand::Kind::Sphere;
    if (!parseDouble(2, &out->a)) {
      *err = "resize_primitive: invalid numeric value for sphere radius";
      return false;
    }
    if (!(out->a > 0.0)) {
      *err = "resize_primitive: sphere radius must be > 0";
      return false;
    }
    return true;
  }

  if (kind == "cylinder") {
    if (toks.size() != 4) {
      *err = "resize_primitive: cylinder expects: cylinder <model_name> <radius> <length>";
      return false;
    }
    out->kind = ResizeCommand::Kind::Cylinder;
    if (!parseDouble(2, &out->a) || !parseDouble(3, &out->b)) {
      *err = "resize_primitive: invalid numeric value for cylinder radius/length";
      return false;
    }
    if (!(out->a > 0.0) || !(out->b > 0.0)) {
      *err = "resize_primitive: cylinder radius/length must be > 0";
      return false;
    }
    return true;
  }

  *err = "resize_primitive: unknown type; expected box, sphere, or cylinder";
  return false;
}

static sdf::Geometry makeGeometry(const ResizeCommand& cmd) {
  sdf::Geometry geom;
  switch (cmd.kind) {
    case ResizeCommand::Kind::Box: {
      geom.SetType(sdf::GeometryType::BOX);
      sdf::Box box;
      box.SetSize({cmd.a, cmd.b, cmd.c});
      geom.SetBoxShape(box);
      break;
    }
    case ResizeCommand::Kind::Sphere: {
      geom.SetType(sdf::GeometryType::SPHERE);
      sdf::Sphere s;
      s.SetRadius(cmd.a);
      geom.SetSphereShape(s);
      break;
    }
    case ResizeCommand::Kind::Cylinder: {
      geom.SetType(sdf::GeometryType::CYLINDER);
      sdf::Cylinder c;
      c.SetRadius(cmd.a);
      c.SetLength(cmd.b);
      geom.SetCylinderShape(c);
      break;
    }
  }
  return geom;
}

}  // namespace detail

class PrimitiveResizer final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>&,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override {
    this->world_entity_ = entity;
    this->event_manager_ = &event_manager;

    gz::sim::World world(this->world_entity_);
    const auto maybe_name = world.Name(ecm);
    if (!maybe_name || maybe_name->empty()) {
      std::cerr << "[sclerp_gazebo] PrimitiveResizer: failed to resolve world name\n";
      return;
    }
    this->world_name_ = *maybe_name;
    this->service_ = "/world/" + this->world_name_ + "/sclerp/resize_primitive";

    if (!this->node_.Advertise(this->service_, &PrimitiveResizer::onResize, this)) {
      std::cerr << "[sclerp_gazebo] PrimitiveResizer: failed to advertise service: " << this->service_ << "\n";
      return;
    }
    std::cerr << "[sclerp_gazebo] PrimitiveResizer: ready; call service: " << this->service_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm) override {
    if (!this->event_manager_) return;

    if (!this->active_) {
      detail::ResizeCommand cmd;
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        if (this->queue_.empty()) return;
        cmd = std::move(this->queue_.front());
        this->queue_.pop_front();
      }
      this->active_ = Active{};
      this->active_->cmd = std::move(cmd);
      this->active_->phase = Active::Phase::RequestRemove;
    }

    gz::sim::World world(this->world_entity_);
    gz::sim::SdfEntityCreator creator(ecm, *this->event_manager_);

    if (this->active_->phase == Active::Phase::RequestRemove) {
      const auto model_entity = world.ModelByName(ecm, this->active_->cmd.model_name);
      if (model_entity == gz::sim::kNullEntity) {
        std::cerr << "[sclerp_gazebo] PrimitiveResizer: model not found: " << this->active_->cmd.model_name << "\n";
        this->active_ = std::nullopt;
        return;
      }

      gz::math::Pose3d pose = gz::math::Pose3d::Zero;
      if (auto* p = ecm.Component<gz::sim::components::Pose>(model_entity)) {
        pose = p->Data();
      } else if (auto* wp = ecm.Component<gz::sim::components::WorldPose>(model_entity)) {
        pose = wp->Data();
      }

      const gz::sim::Model m(model_entity);
      this->active_->pose = pose;
      this->active_->is_static = m.Static(ecm);
      this->active_->old_entity = model_entity;

      creator.RequestRemoveEntity(model_entity, true);
      this->active_->phase = Active::Phase::WaitRemoved;
      return;
    }

    if (this->active_->phase == Active::Phase::WaitRemoved) {
      if (this->active_->old_entity != gz::sim::kNullEntity && ecm.HasEntity(this->active_->old_entity)) {
        return;
      }

      sdf::Model model;
      model.SetName(this->active_->cmd.model_name);
      model.SetStatic(this->active_->is_static);
      model.SetRawPose(this->active_->pose);

      sdf::Link link;
      link.SetName("link");

      const sdf::Geometry geom = detail::makeGeometry(this->active_->cmd);

      sdf::Collision col;
      col.SetName("collision");
      col.SetGeom(geom);
      link.AddCollision(col);

      sdf::Visual vis;
      vis.SetName("visual");
      vis.SetGeom(geom);
      link.AddVisual(vis);

      model.AddLink(link);

      const auto new_entity = creator.CreateEntities(&model);
      if (new_entity == gz::sim::kNullEntity) {
        std::cerr << "[sclerp_gazebo] PrimitiveResizer: failed to spawn resized model: "
                  << this->active_->cmd.model_name << "\n";
        this->active_ = std::nullopt;
        return;
      }

      creator.SetParent(new_entity, this->world_entity_);

      std::cerr << "[sclerp_gazebo] PrimitiveResizer: resized model '" << this->active_->cmd.model_name << "'\n";
      this->active_ = std::nullopt;
      return;
    }
  }

private:
  struct Active {
    enum class Phase { RequestRemove = 0, WaitRemoved = 1 };
    Phase phase{Phase::RequestRemove};
    detail::ResizeCommand cmd;
    gz::sim::Entity old_entity{gz::sim::kNullEntity};
    gz::math::Pose3d pose{gz::math::Pose3d::Zero};
    bool is_static{true};
  };

  bool onResize(const gz::msgs::StringMsg& req, gz::msgs::Boolean& rep) {
    detail::ResizeCommand cmd;
    std::string err;
    if (!detail::parseCommand(req.data(), &cmd, &err)) {
      std::cerr << "[sclerp_gazebo] PrimitiveResizer: " << err << "\n";
      rep.set_data(false);
      return true;
    }

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->queue_.push_back(std::move(cmd));
    }

    rep.set_data(true);
    return true;
  }

private:
  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  gz::sim::EventManager* event_manager_{nullptr};
  std::string world_name_;

  gz::transport::Node node_;
  std::string service_;

  std::mutex mutex_;
  std::deque<detail::ResizeCommand> queue_;
  std::optional<Active> active_;
};

}  // namespace sclerp::gazebo

IGNITION_ADD_PLUGIN(sclerp::gazebo::PrimitiveResizer,
                    gz::sim::System,
                    gz::sim::ISystemConfigure,
                    gz::sim::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(sclerp::gazebo::PrimitiveResizer,
                          "sclerp::gazebo::PrimitiveResizer")
