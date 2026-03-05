#include "sclerp/core/planning/failure_log.hpp"

#include "sclerp/core/common/logger.hpp"

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace sclerp::core {
namespace {

constexpr const char* kDefaultDir = "sclerp_failure_logs";

static std::atomic<std::uint64_t> g_seq{0};

const char* statusToString(Status s) {
  switch (s) {
    case Status::Success: return "Success";
    case Status::Failure: return "Failure";
    case Status::JointLimit: return "JointLimit";
    case Status::InvalidParameter: return "InvalidParameter";
  }
  return "Unknown";
}

LogLevel failureLogLevel(Status s) {
  return ok(s) ? LogLevel::Info : LogLevel::Error;
}

std::string timestampForFilename() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto t = system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif

  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_"
      << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

std::string nowIso8601Local() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto t = system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif

  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << "."
      << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}

bool isValidCsvName(const std::string& s) {
  // Keep the dump robust for trivial CSV readers.
  // Note: dump readers treat lines starting with '#' as comments, so don't allow a header token
  // to start with '#'.
  return !s.empty() && s[0] != '#' && s.find(',') == std::string::npos &&
         s.find('\n') == std::string::npos && s.find('\r') == std::string::npos;
}

void writeJointPathCsv(std::ostream& out,
                       const std::vector<std::string>& joint_names,
                       const std::vector<const Eigen::VectorXd*>& waypoints) {
  out << std::setprecision(17);

  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    if (j) out << ",";
    out << joint_names[j];
  }
  out << "\n";

  const int dof = static_cast<int>(joint_names.size());
  for (const Eigen::VectorXd* q_ptr : waypoints) {
    if (!q_ptr) continue;
    const auto& q = *q_ptr;
    if (q.size() != dof) continue;
    for (int j = 0; j < dof; ++j) {
      if (j) out << ",";
      out << q[j];
    }
    out << "\n";
  }
}

}  // namespace

void dumpMotionPlanFailure(const std::string& planner_name,
                           Status status,
                           int iters,
                           const JointPath& path,
                           const FailureLogOptions& opt,
                           const Eigen::VectorXd* q_init,
                           const std::vector<std::string>* joint_names) noexcept {
  try {
    if (ok(status)) return;
    if (!opt.enabled) return;

    std::vector<const Eigen::VectorXd*> waypoints;
    waypoints.reserve(path.positions.size() + 1);

    if (!path.positions.empty()) {
      for (const auto& q : path.positions) waypoints.push_back(&q);
    } else if (q_init && q_init->size() > 0) {
      waypoints.push_back(q_init);
    } else {
      log(LogLevel::Warn, planner_name + ": failure log skipped (empty path and missing q_init)");
      return;
    }

    const int dof = static_cast<int>(waypoints.front()->size());
    if (dof <= 0) {
      log(LogLevel::Warn, planner_name + ": failure log skipped (invalid dof)");
      return;
    }

    std::vector<std::string> header_names;
    header_names.reserve(static_cast<std::size_t>(dof));

    const auto tryUseNames = [&](const std::vector<std::string>& names) -> bool {
      if (names.size() != static_cast<std::size_t>(dof)) return false;
      for (const auto& n : names) {
        if (!isValidCsvName(n)) return false;
      }
      header_names = names;
      return true;
    };

    bool have_names = false;
    if (!path.joint_names.empty()) have_names = tryUseNames(path.joint_names);
    if (!have_names && joint_names) have_names = tryUseNames(*joint_names);
    if (!have_names) {
      header_names.clear();
      for (int j = 0; j < dof; ++j) header_names.push_back("q" + std::to_string(j));
    }

    const std::filesystem::path out_dir = opt.dir.empty() ? std::filesystem::path(kDefaultDir)
                                                          : std::filesystem::path(opt.dir);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
      log(LogLevel::Warn,
          planner_name + ": failed to create failure log dir '" + out_dir.string() + "': " + ec.message());
      return;
    }

    const std::string ts = timestampForFilename();
    const std::uint64_t seq = g_seq.fetch_add(1, std::memory_order_relaxed);
    const std::string fname = planner_name + "_" + ts + "_" + statusToString(status) + "_" + std::to_string(seq) + ".csv";
    const std::filesystem::path csv_path = out_dir / fname;

    std::ofstream f(csv_path.string());
    if (!f) {
      log(LogLevel::Warn, planner_name + ": failed to open failure dump file: " + csv_path.string());
      return;
    }

    f << "# sclerp_failure_log_v1\n";
    f << "# planner=" << planner_name << "\n";
    f << "# status=" << static_cast<int>(status) << " (" << statusToString(status) << ")\n";
    f << "# iters=" << iters << "\n";
    f << "# waypoints=" << waypoints.size() << "\n";
    f << "# timestamp_local=" << nowIso8601Local() << "\n";

    writeJointPathCsv(f, header_names, waypoints);

    const LogLevel lvl = failureLogLevel(status);
    std::ostringstream oss;
    oss << planner_name << ": status=" << statusToString(status)
        << ", iters=" << iters
        << ", dumped_joint_path=" << csv_path.string();
    log(lvl, oss.str());
  } catch (const std::exception& e) {
    log(LogLevel::Warn, planner_name + ": exception while writing failure log: " + std::string(e.what()));
  } catch (...) {
    log(LogLevel::Warn, planner_name + ": unknown exception while writing failure log");
  }
}

}  // namespace sclerp::core
