#include "sclerp/trajectory/interpolator.hpp"

namespace sclerp::trajectory {

std::vector<bool> continuousRevoluteMask(const sclerp::core::ManipulatorModel& model) {
  std::vector<bool> out;
  const int dof = model.dof();
  out.resize(static_cast<std::size_t>(dof), false);
  for (int i = 0; i < dof; ++i) {
    const auto& j = model.joint(i);
    const bool continuous = (j.type == sclerp::core::JointType::Revolute) && (!j.limit.enabled);
    out[static_cast<std::size_t>(i)] = continuous;
  }
  return out;
}

}  // namespace sclerp::trajectory

