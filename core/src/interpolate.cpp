#include "sclerp/core/interpolate.hpp"
#include <cmath>

namespace sclerp::core {

SE3d sclerp(const SE3d& T0, const SE3d& T1, double t) {
  const SE3d T_rel = T0.inverse() * T1;
  const Twist6d xi = logSE3(T_rel);

  Twist6d xi_t;
  xi_t.omega = xi.omega * t;
  xi_t.v     = xi.v * t;

  return T0 * expSE3(xi_t);
}

static Quat slerp_quat(const Quat& q0, const Quat& q1, double t) {
  Quat a = q0.normalized();
  Quat b = q1.normalized();
  return a.slerp(t, b).normalized();
}

SE3d interpolate(const SE3d& T0, const SE3d& T1, const Constraint& c, double t) {
  switch (c.type) {
    case ConstraintType::SE3:
      return sclerp(T0, T1, t);

    case ConstraintType::R3: {
      // translation only; keep rotation fixed at start
      const Vec3 p = (1.0 - t) * T0.p + t * T1.p;
      return SE3d(T0.q, p);
    }

    case ConstraintType::SO3: {
      // rotation only; keep translation fixed at start
      const Quat q = slerp_quat(T0.q, T1.q, t);
      return SE3d(q, T0.p);
    }

    case ConstraintType::SE2: {
      // Convention: use world Z as "up". Keep z, roll, pitch from start.
      // Implement by mixing translation x,y and yaw; keep z fixed.
      // For v0: approximate using yaw extraction from quaternions.
      const Vec3 p = (1.0 - t) * T0.p + t * T1.p;
      Vec3 p2 = p;
      p2.z() = T0.p.z();

      // Extract yaw from q (Z-Y-X convention)
      auto yaw_from_quat = [](const Quat& q) {
        const Mat3 R = q.toRotationMatrix();
        return std::atan2(R(1,0), R(0,0));
      };
      const double yaw0 = yaw_from_quat(T0.q);
      const double yaw1 = yaw_from_quat(T1.q);

      // shortest-angle interpolate
      double dy = yaw1 - yaw0;
      while (dy > M_PI) dy -= 2.0*M_PI;
      while (dy < -M_PI) dy += 2.0*M_PI;
      const double yaw = yaw0 + t * dy;

      // keep roll/pitch from start by constructing R = R_rp0 * Rz(yaw_delta)
      // v0: just build pure yaw and ignore rp (can refine later)
      Quat q = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
      return SE3d(q, p2);
    }

    case ConstraintType::Custom:
    default:
      // v0 fallback: full SE3
      return sclerp(T0, T1, t);
  }
}

}  // namespace sclerp::core
