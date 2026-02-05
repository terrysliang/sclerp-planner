#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "sclerp/collision/collision.hpp"
#include "sclerp/core/common/status.hpp"

using sclerp::collision::FclObject;
using sclerp::collision::DistanceQueryCache;
using sclerp::collision::createSphere;
using sclerp::collision::checkCollision;
using sclerp::core::Status;
using sclerp::core::ok;
using sclerp::core::Vec3;
using sclerp::core::Mat3;

static int parseIntArg(int argc, char** argv, const char* key, int def) {
  const std::string prefix = std::string(key) + "=";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0 && i + 1 < argc) {
      return std::stoi(argv[i + 1]);
    }
    if (std::strncmp(argv[i], prefix.c_str(), prefix.size()) == 0) {
      return std::stoi(std::string(argv[i] + prefix.size()));
    }
  }
  return def;
}

template <typename Fn>
static double benchMs(Fn&& fn) {
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

static std::vector<std::shared_ptr<FclObject>> makeSpheres(int count,
                                                           double radius,
                                                           const Vec3& start,
                                                           const Vec3& step) {
  std::vector<std::shared_ptr<FclObject>> out;
  out.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const Vec3 pos = start + step * static_cast<double>(i);
    std::shared_ptr<FclObject> obj;
    const Status st = createSphere(radius, pos, Mat3::Identity(), &obj);
    if (!ok(st)) {
      std::cerr << "createSphere failed at index " << i << "\n";
      std::exit(1);
    }
    out.push_back(std::move(obj));
  }
  return out;
}

int main(int argc, char** argv) {
  const int links = parseIntArg(argc, argv, "--links", 20);
  const int obstacles = parseIntArg(argc, argv, "--obstacles", 50);
  const int iters = parseIntArg(argc, argv, "--iters", 200);

  const double radius = 0.05;
  const Vec3 link_start(0.0, 0.0, 0.0);
  const Vec3 link_step(0.2, 0.0, 0.0);
  const Vec3 obs_start(1.0, 0.3, 0.0);
  const Vec3 obs_step(0.05, 0.05, 0.02);

  auto link_objs = makeSpheres(links, radius, link_start, link_step);
  auto obs_objs = makeSpheres(obstacles, radius, obs_start, obs_step);

  static volatile double sink = 0.0;

  auto run_checks = [&](bool cached) {
    double acc = 0.0;
    DistanceQueryCache cache;
    for (int it = 0; it < iters; ++it) {
      for (const auto& obs : obs_objs) {
        for (const auto& link : link_objs) {
          double min_d = 0.0;
          Vec3 p1 = Vec3::Zero();
          Vec3 p2 = Vec3::Zero();
          Status st = Status::Failure;
          if (cached) {
            st = checkCollision(*obs, *link, &min_d, &p1, &p2, &cache);
          } else {
            st = checkCollision(*obs, *link, &min_d, &p1, &p2);
          }
          if (!ok(st)) {
            std::cerr << "checkCollision failed\n";
            std::exit(1);
          }
          acc += min_d;
        }
      }
    }
    sink += acc;
  };

  // Warm-up
  run_checks(false);

  const double baseline_ms = benchMs([&]() { run_checks(false); });
  const double cached_ms = benchMs([&]() { run_checks(true); });

  std::cout << "sclerp_collision_benchmark\n";
  std::cout << "links=" << links
            << " obstacles=" << obstacles
            << " iters=" << iters << "\n";
  std::cout << "baseline_ms=" << baseline_ms << "\n";
  std::cout << "cached_ms=" << cached_ms << "\n";
  return 0;
}
