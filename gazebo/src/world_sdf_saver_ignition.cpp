#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
// NOLINTNEXTLINE(modernize-deprecated-headers)
#include <gz/msgs/boolean.pb.h>
// NOLINTNEXTLINE(modernize-deprecated-headers)
#include <gz/msgs/sdf_generator_config.pb.h>
// NOLINTNEXTLINE(modernize-deprecated-headers)
#include <gz/msgs/stringmsg.pb.h>

#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

namespace sclerp::gazebo {

class WorldSdfSaver final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure {
public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager&) override {
    this->world_entity_ = entity;

    if (sdf) {
      if (sdf->HasElement("output_path")) {
        this->output_path_ = sdf->Get<std::string>("output_path");
      }
      if (sdf->HasElement("timeout_ms")) {
        const int t = sdf->Get<int>("timeout_ms");
        if (t > 0) this->timeout_ms_ = static_cast<unsigned int>(t);
      }

      auto* gen_cfg = this->generator_config_.mutable_global_entity_gen_config();
      if (sdf->HasElement("expand_include_tags")) {
        gen_cfg->mutable_expand_include_tags()->set_data(sdf->Get<bool>("expand_include_tags"));
      } else {
        gen_cfg->mutable_expand_include_tags()->set_data(true);
      }
      if (sdf->HasElement("resources_use_absolute_paths")) {
        gen_cfg->mutable_resources_use_absolute_paths()->set_data(sdf->Get<bool>("resources_use_absolute_paths"));
      } else {
        gen_cfg->mutable_resources_use_absolute_paths()->set_data(true);
      }
      if (sdf->HasElement("copy_model_resources")) {
        gen_cfg->mutable_copy_model_resources()->set_data(sdf->Get<bool>("copy_model_resources"));
      } else {
        gen_cfg->mutable_copy_model_resources()->set_data(false);
      }
    } else {
      auto* gen_cfg = this->generator_config_.mutable_global_entity_gen_config();
      gen_cfg->mutable_expand_include_tags()->set_data(true);
      gen_cfg->mutable_resources_use_absolute_paths()->set_data(true);
      gen_cfg->mutable_copy_model_resources()->set_data(false);
    }

    gz::sim::World world(this->world_entity_);
    const auto maybe_name = world.Name(ecm);
    if (!maybe_name || maybe_name->empty()) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: failed to resolve world name\n";
      return;
    }
    this->world_name_ = *maybe_name;

    this->generate_service_ = "/world/" + this->world_name_ + "/generate_world_sdf";
    this->save_service_ = "/world/" + this->world_name_ + "/sclerp/save_world_sdf";

    if (!this->node_.Advertise(this->save_service_, &WorldSdfSaver::onSave, this)) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: failed to advertise service: " << this->save_service_ << "\n";
      return;
    }

    std::cerr << "[sclerp_gazebo] WorldSdfSaver: ready; call service: " << this->save_service_ << "\n";
  }

private:
  bool onSave(const gz::msgs::StringMsg& req, gz::msgs::Boolean& rep) {
    const std::string path = req.data().empty() ? this->output_path_ : req.data();
    const bool ok = this->saveWorldSdf(path);
    rep.set_data(ok);
    return true;
  }

  bool saveWorldSdf(const std::string& path) {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->generate_service_.empty()) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: missing generate_world_sdf service name\n";
      return false;
    }
    if (path.empty()) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: output path is empty\n";
      return false;
    }

    gz::msgs::StringMsg sdf_out;
    bool result = false;
    const bool called = this->node_.Request(this->generate_service_,
                                           this->generator_config_,
                                           this->timeout_ms_,
                                           sdf_out,
                                           result);
    if (!called) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: request to '" << this->generate_service_
                << "' failed or timed out (" << this->timeout_ms_ << " ms)\n";
      return false;
    }
    if (!result) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: generate_world_sdf returned failure\n";
      return false;
    }
    if (sdf_out.data().empty()) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: generate_world_sdf returned empty SDF\n";
      return false;
    }

    std::ofstream out(path);
    if (!out) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: failed to open output file: " << path << "\n";
      return false;
    }
    out << sdf_out.data();
    out.close();
    if (!out) {
      std::cerr << "[sclerp_gazebo] WorldSdfSaver: failed to write output file: " << path << "\n";
      return false;
    }

    std::cerr << "[sclerp_gazebo] WorldSdfSaver: saved world SDF to: " << path << "\n";
    return true;
  }

private:
  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  std::string world_name_;

  std::string output_path_{"world_saved.sdf"};
  unsigned int timeout_ms_{5000};
  gz::msgs::SdfGeneratorConfig generator_config_;

  gz::transport::Node node_;
  std::string generate_service_;
  std::string save_service_;

  std::mutex mutex_;
};

}  // namespace sclerp::gazebo

IGNITION_ADD_PLUGIN(sclerp::gazebo::WorldSdfSaver,
                    gz::sim::System,
                    gz::sim::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(sclerp::gazebo::WorldSdfSaver,
                          "sclerp::gazebo::WorldSdfSaver")
