/**
 * @file freeze_judge.h
 * @author hang chen (chen@hang.plus)
 * @brief Is this session's geometry good enough to be called absolute truth forever.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_FREEZE_JUDGE_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_FREEZE_JUDGE_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <map>
#include <ostream>
#include <vector>

#include "lifelong/optimization/covariance_evaluator.h"
#include "lifelong/optimization/optimization.h"
#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

struct FreezeJudgeOption {
  // Simulation values, pending calibration on a real loop bag.
  double max_translation_stddev = 0.02;
  double max_rotation_stddev = 0.005;

  friend std::ostream& operator<<(std::ostream& os, const FreezeJudgeOption& option) {
    os << "FreezeJudgeOption:" << std::endl;
    os << "  max_translation_stddev: " << option.max_translation_stddev << std::endl;
    os << "  max_rotation_stddev: " << option.max_rotation_stddev << std::endl;
    return os;
  }
};

FreezeJudgeOption LoadFreezeJudgeOption(const YAML::Node& node);

enum class FreezeRejection {
  NONE,
  NOT_ACTIVE,
  NO_FINISHED_SUBMAP,
  NOT_ANCHORED,
  COVARIANCE_UNAVAILABLE,
  COVARIANCE_TOO_LARGE,
};

const char* ToString(FreezeRejection rejection);

struct SubmapUncertainty {
  SubmapId id;
  bool unfinished = false;
  bool covariance_available = false;
  // Zero when unavailable, and legitimately zero for the gauge datum.
  Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
  bool passed = false;
};

struct FreezeVerdict {
  bool eligible = false;
  // Nothing frozen yet and this session defines the global frame: eligible on structure alone.
  bool bootstrap = false;
  FreezeRejection rejection = FreezeRejection::NOT_ACTIVE;
  std::map<SubmapId, int> frozen_links;
  std::vector<SubmapUncertainty> submaps;

  friend std::ostream& operator<<(std::ostream& os, const FreezeVerdict& verdict) {
    int linked = 0;
    for (const auto& [id, count] : verdict.frozen_links) {
      linked += count > 0 ? 1 : 0;
    }
    int finished = 0;
    int passed = 0;
    Eigen::Vector3d worst = Eigen::Vector3d::Zero();
    for (const SubmapUncertainty& uncertainty : verdict.submaps) {
      if (uncertainty.unfinished) {
        continue;
      }
      ++finished;
      passed += uncertainty.passed ? 1 : 0;
      worst = worst.cwiseMax(uncertainty.stddev);
    }
    os << ToString(verdict.rejection) << (verdict.bootstrap ? " (bootstrap)" : "") << ", " << linked
       << " of " << verdict.frozen_links.size() << " submaps linked, " << passed << " of "
       << finished << " finished passed, worst stddev " << worst.x() << " " << worst.y() << " m "
       << worst.z() << " rad";
    return os;
  }
};

// Ask right after a solve: the covariances and the gauge assignment are only meaningful there.
class FreezeJudge {
 public:
  explicit FreezeJudge(const FreezeJudgeOption& option = FreezeJudgeOption());

  FreezeVerdict Judge(const PoseGraphData& graph, Optimization& optimization,
                      CovarianceEvaluator& covariance_evaluator, SessionId id) const;

  const FreezeJudgeOption& option() const { return option_; }

 private:
  FreezeJudgeOption option_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_FREEZE_JUDGE_H_
