/**
 * @file freeze_judge.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/freeze_judge.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <optional>

#include "lifelong/sessions/frozen_links.h"
#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {
namespace {

bool CarriesOwnDatum(const PoseGraphData& graph, const Optimization& optimization, SessionId id) {
  const SessionData& session = graph.session(id);
  for (const SubmapId& submap_id : session.submap_ids) {
    const VariableId variable = VariableId::Of(submap_id);
    if (optimization.HasVariable(variable) && optimization.IsVariableConstant(variable)) {
      return true;
    }
  }
  for (const NodeId& node_id : session.node_ids) {
    const VariableId variable = VariableId::Of(node_id);
    if (optimization.HasVariable(variable) && optimization.IsVariableConstant(variable)) {
      return true;
    }
  }
  return false;
}

bool AnySessionFrozen(const PoseGraphData& graph) {
  for (const auto& [id, session] : graph.sessions()) {
    if (session.frozen()) {
      return true;
    }
  }
  return false;
}

bool IsUnfinished(const PoseGraphData& graph, const SubmapId& id) {
  const SubmapRecord& record = graph.submap(id);
  return record.submap != nullptr && !record.submap->finished();
}

}  // namespace

FreezeJudgeOption LoadFreezeJudgeOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  FreezeJudgeOption option;
  option.max_translation_stddev =
      LoadOption(node, "max_translation_stddev", option.max_translation_stddev);
  option.max_rotation_stddev = LoadOption(node, "max_rotation_stddev", option.max_rotation_stddev);
  return option;
}

const char* ToString(FreezeRejection rejection) {
  switch (rejection) {
    case FreezeRejection::NONE:
      return "none";
    case FreezeRejection::NOT_ACTIVE:
      return "not active";
    case FreezeRejection::NO_FINISHED_SUBMAP:
      return "no finished submap";
    case FreezeRejection::NOT_ANCHORED:
      return "not anchored";
    case FreezeRejection::COVARIANCE_UNAVAILABLE:
      return "covariance unavailable";
    case FreezeRejection::COVARIANCE_TOO_LARGE:
      return "covariance too large";
  }
  return "unknown";
}

FreezeJudge::FreezeJudge(const FreezeJudgeOption& option) : option_(option) {
  CHECK_GT(option_.max_translation_stddev, 0.0);
  CHECK_GT(option_.max_rotation_stddev, 0.0);
}

FreezeVerdict FreezeJudge::Judge(const PoseGraphData& graph, Optimization& optimization,
                                 CovarianceEvaluator& covariance_evaluator, SessionId id) const {
  FreezeVerdict verdict;

  if (!graph.HasSession(id) || graph.session(id).frozen()) {
    verdict.rejection = FreezeRejection::NOT_ACTIVE;
    return verdict;
  }

  const SessionData& session = graph.session(id);
  std::vector<SubmapId> finished;
  for (const SubmapId& submap_id : session.submap_ids) {
    if (!IsUnfinished(graph, submap_id)) {
      finished.push_back(submap_id);
    }
  }
  if (finished.empty()) {
    verdict.rejection = FreezeRejection::NO_FINISHED_SUBMAP;
    return verdict;
  }

  if (!AnySessionFrozen(graph) && CarriesOwnDatum(graph, optimization, id)) {
    verdict.bootstrap = true;
    verdict.eligible = true;
    verdict.rejection = FreezeRejection::NONE;
    return verdict;
  }

  verdict.frozen_links = CountFrozenLinks(graph, id);
  // An unanchored session's covariance is relative to its own datum (measured ~0.006 m) and
  // says nothing.
  if (!HasFrozenLink(graph, id)) {
    verdict.rejection = FreezeRejection::NOT_ANCHORED;
    return verdict;
  }
  verdict.rejection = FreezeRejection::NONE;

  std::vector<VariableId> known_variables;
  known_variables.reserve(finished.size());
  for (const SubmapId& submap_id : finished) {
    const VariableId variable = VariableId::Of(submap_id);
    if (optimization.HasVariable(variable)) {
      known_variables.push_back(variable);
    }
  }
  const std::optional<std::map<VariableId, Eigen::Matrix3d>> covariances =
      covariance_evaluator.ComputeMarginalCovariancesInGlobal(known_variables);

  verdict.submaps.reserve(session.submap_ids.size());
  for (const SubmapId& submap_id : session.submap_ids) {
    SubmapUncertainty uncertainty;
    uncertainty.id = submap_id;
    if (IsUnfinished(graph, submap_id)) {
      uncertainty.unfinished = true;
      verdict.submaps.push_back(uncertainty);
      continue;
    }
    const VariableId variable = VariableId::Of(submap_id);
    const std::optional<Eigen::Matrix3d> covariance =
        covariances.has_value() && optimization.HasVariable(variable)
            ? std::make_optional(covariances->at(variable))
            : std::nullopt;
    if (!covariance.has_value()) {
      verdict.submaps.push_back(uncertainty);
      if (verdict.rejection == FreezeRejection::NONE) {
        verdict.rejection = FreezeRejection::COVARIANCE_UNAVAILABLE;
      }
      continue;
    }
    uncertainty.covariance_available = true;
    uncertainty.stddev = Eigen::Vector3d(std::sqrt(std::max(0.0, (*covariance)(0, 0))),
                                         std::sqrt(std::max(0.0, (*covariance)(1, 1))),
                                         std::sqrt(std::max(0.0, (*covariance)(2, 2))));
    uncertainty.passed = uncertainty.stddev.x() <= option_.max_translation_stddev &&
                         uncertainty.stddev.y() <= option_.max_translation_stddev &&
                         uncertainty.stddev.z() <= option_.max_rotation_stddev;
    if (!uncertainty.passed && verdict.rejection == FreezeRejection::NONE) {
      verdict.rejection = FreezeRejection::COVARIANCE_TOO_LARGE;
    }
    verdict.submaps.push_back(uncertainty);
  }

  verdict.eligible = verdict.rejection == FreezeRejection::NONE;
  return verdict;
}

}  // namespace evergreenslam::lifelong
