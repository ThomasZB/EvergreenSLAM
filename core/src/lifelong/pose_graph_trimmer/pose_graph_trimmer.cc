/**
 * @file pose_graph_trimmer.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/pose_graph_trimmer.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "lifelong/pose_graph_trimmer/chow_liu_tree.h"
#include "lifelong/pose_graph_trimmer/marginalizer.h"

namespace evergreenslam::lifelong {
namespace {

int NearestIndex(const std::vector<Eigen::Affine2d>& poses, const Eigen::Vector2d& reference) {
  CHECK(!poses.empty());
  int nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poses.size(); ++i) {
    const double distance = (poses[i].translation() - reference).norm();
    if (distance < best) {
      best = distance;
      nearest = static_cast<int>(i);
    }
  }
  return nearest;
}

bool IsSessionHead(const PoseGraphData& graph, const SubmapId& id) {
  for (const SubmapId& other : graph.session(SessionOf(id)).submap_ids) {
    if (other.submap_index < id.submap_index) {
      return false;
    }
  }
  return true;
}

// A trimmed head hands its frozen link to the chain's new head: the nearest survivor is often a
// revisit pass, which would carry the tie away from where it was measured. Anywhere else the
// nearest survivor keeps the tie local.
int AnchorSurvivorIndex(const std::vector<VariableId>& survivors,
                        const std::vector<Eigen::Affine2d>& poses, SessionId session, bool head,
                        const Eigen::Vector2d& reference) {
  std::optional<int> earliest;
  for (size_t i = 0; head && i < survivors.size(); ++i) {
    if (survivors[i].kind != VariableId::Kind::NODE || survivors[i].session() != session) {
      continue;
    }
    if (!earliest.has_value() ||
        survivors[i].node_id().node_index < survivors[*earliest].node_id().node_index) {
      earliest = static_cast<int>(i);
    }
  }
  return earliest.value_or(NearestIndex(poses, reference));
}

std::optional<Constraint> MakePriorConstraint(const VariableId& variable,
                                              const Eigen::Affine2d& global_pose,
                                              const Eigen::Matrix3d& covariance,
                                              double min_stddev) {
  const std::optional<Eigen::Matrix3d> sqrt_information =
      FlooredSqrtInformation(covariance, min_stddev);
  if (!sqrt_information.has_value()) {
    return std::nullopt;
  }
  Constraint prior;
  prior.type = Constraint::Type::PRIOR;
  prior.from = variable;
  prior.to = std::nullopt;
  prior.relative_pose = global_pose;
  prior.sqrt_information = *sqrt_information;
  return prior;
}

// The anchor is constant, so the survivor's covariance is the whole relative uncertainty.
std::optional<Constraint> MakeAnchorConstraint(const VariableId& anchor,
                                               const Eigen::Affine2d& anchor_pose,
                                               const VariableId& survivor,
                                               const Eigen::Affine2d& survivor_pose,
                                               const Eigen::Matrix3d& survivor_covariance,
                                               double min_stddev) {
  Eigen::MatrixXd pair = Eigen::MatrixXd::Zero(6, 6);
  pair.bottomRightCorner<3, 3>() = survivor_covariance;
  const std::optional<MarginalConstraint> marginal =
      MakeMarginalConstraint({anchor_pose, survivor_pose}, pair, 0, 1, min_stddev);
  if (!marginal.has_value()) {
    return std::nullopt;
  }
  CHECK_EQ(marginal->from, 0);
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.recovered = true;
  constraint.from = anchor;
  constraint.to = survivor;
  constraint.relative_pose = marginal->relative_pose;
  constraint.sqrt_information = marginal->sqrt_information;
  return constraint;
}

}  // namespace

PoseGraphTrimmer::PoseGraphTrimmer(TrimmingHandle& handle, const PoseGraphTrimmerOption& option)
    : handle_(handle),
      option_(option),
      selector_(option.selector),
      covariance_evaluator_(handle.optimization()) {
  CHECK_GT(option_.max_submaps_per_round, 0);
}

void PoseGraphTrimmer::TrimOnce() {
  handle_.Enqueue([this] { RunRound(); });
}

void PoseGraphTrimmer::TrimToCompletionOnTask() {
  RunRound();
  while (!pending_.empty()) {
    RunRound();
  }
}

void PoseGraphTrimmer::EnqueueForTrim(const SubmapId& id) { pending_.push_back(id); }

void PoseGraphTrimmer::RunRound() {
  if (pending_.empty()) {
    for (const SubmapId& id : selector_.Select(handle_.graph())) {
      pending_.push_back(id);
    }
    return;
  }
  int trimmed = 0;
  while (trimmed < option_.max_submaps_per_round && !pending_.empty()) {
    const SubmapId id = pending_.front();
    pending_.pop_front();
    if (TrimSubmap(id)) {
      ++trimmed;
    }
  }
}

bool PoseGraphTrimmer::TrimSubmap(const SubmapId& id) {
  const PoseGraphData& graph = handle_.graph();
  Optimization& optimization = handle_.optimization();
  if (!graph.HasSubmap(id) || graph.session(SessionOf(id)).frozen()) {
    return false;
  }

  std::set<VariableId> removed;
  removed.insert(VariableId::Of(id));
  for (const NodeId& node_id : graph.submap(id).node_ids) {
    const std::vector<SubmapId> containing = graph.ContainingSubmapIds(node_id);
    if (containing.size() == 1 && containing.front() == id) {
      removed.insert(VariableId::Of(node_id));
    }
  }
  for (const VariableId& variable : removed) {
    if (optimization.IsVariableConstant(variable)) {
      VLOG(1) << "skipping a trim whose variable is constant (gauge datum)";
      return false;
    }
  }

  // Constant survivors and priors carry the removed set's global information; both must be
  // re-attached below or the component drifts off the global frame.
  std::set<VariableId> blanket;
  std::set<VariableId> constant_anchors;
  bool carries_prior = false;
  for (const Constraint& constraint : graph.constraints()) {
    const bool from_removed = removed.count(constraint.from) > 0;
    if (!constraint.to.has_value()) {
      carries_prior = carries_prior || from_removed;
      continue;
    }
    const bool to_removed = removed.count(*constraint.to) > 0;
    if (from_removed == to_removed) {
      continue;
    }
    const VariableId& survivor = from_removed ? *constraint.to : constraint.from;
    if (optimization.IsVariableConstant(survivor)) {
      constant_anchors.insert(survivor);
    } else {
      blanket.insert(survivor);
    }
  }

  if (carries_prior && !optimization.IsSessionAnchoredToFrozen(SessionOf(id))) {
    LOG(WARNING) << "prior dropped: its session holds its own gauge datum";
    carries_prior = false;
  }

  std::vector<Constraint> added_constraints;
  const std::vector<VariableId> survivors(blanket.begin(), blanket.end());
  const bool needs_anchor = !survivors.empty() && (carries_prior || !constant_anchors.empty());
  if (survivors.size() >= 2 || needs_anchor) {
    // The removed variables must stay free, so the survivor joint marginalizes rather than
    // conditions on them.
    std::vector<VariableId> evaluation_order = survivors;
    evaluation_order.insert(evaluation_order.end(), removed.begin(), removed.end());
    const std::optional<Eigen::MatrixXd> joint =
        covariance_evaluator_.ComputeConditionalJointCovarianceInBlanket(evaluation_order);
    if (!joint.has_value()) {
      LOG(WARNING) << "trim skipped: the blanket did not factorize";
      return false;
    }
    const int survivor_dims = static_cast<int>(survivors.size()) * 3;
    const Eigen::MatrixXd survivor_joint = joint->topLeftCorner(survivor_dims, survivor_dims);

    std::vector<Eigen::Affine2d> poses;
    poses.reserve(survivors.size());
    for (const VariableId& variable : survivors) {
      poses.push_back(optimization.GetVariablePose(variable));
    }

    if (survivors.size() >= 2) {
      const Eigen::MatrixXd mutual_information = ComputeMutualInformationMatrix(survivor_joint);
      for (const TreeEdge& edge : ComputeMaximumSpanningTree(mutual_information)) {
        const std::optional<MarginalConstraint> marginal = MakeMarginalConstraint(
            poses, survivor_joint, edge.first, edge.second, option_.min_recovered_stddev);
        if (!marginal.has_value()) {
          LOG(WARNING) << "tree edge dropped: covariance not invertible";
          continue;
        }
        Constraint constraint;
        constraint.type = Constraint::Type::INTER_SUBMAP;
        constraint.recovered = true;
        constraint.from = survivors[marginal->from];
        constraint.to = survivors[marginal->to];
        constraint.relative_pose = marginal->relative_pose;
        constraint.sqrt_information = marginal->sqrt_information;
        if (constraint.relative_pose.translation().norm() > option_.max_binary_constraint_length) {
          const std::optional<Constraint> prior = MaybeConvertLongEdgeToPrior(constraint);
          if (prior.has_value()) {
            added_constraints.push_back(*prior);
            ++num_priors_added_;
            continue;
          }
        }
        added_constraints.push_back(constraint);
      }
    }

    if (needs_anchor) {
      const int anchor =
          AnchorSurvivorIndex(survivors, poses, SessionOf(id), IsSessionHead(graph, id),
                              optimization.GetVariablePose(VariableId::Of(id)).translation());
      const VariableId& survivor = survivors[anchor];
      const Eigen::Affine2d& survivor_pose = poses[anchor];
      const Eigen::Matrix3d survivor_covariance =
          survivor_joint.block<3, 3>(anchor * 3, anchor * 3);
      if (!constant_anchors.empty()) {
        std::vector<VariableId> anchors(constant_anchors.begin(), constant_anchors.end());
        std::vector<Eigen::Affine2d> anchor_poses;
        for (const VariableId& anchor : anchors) {
          anchor_poses.push_back(optimization.GetVariablePose(anchor));
        }
        const int closest = NearestIndex(anchor_poses, survivor_pose.translation());
        const std::optional<Constraint> constraint =
            MakeAnchorConstraint(anchors[closest], anchor_poses[closest], survivor, survivor_pose,
                                 survivor_covariance, option_.min_recovered_stddev);
        if (constraint.has_value()) {
          added_constraints.push_back(*constraint);
        } else {
          LOG(WARNING) << "anchor edge dropped: covariance not invertible";
        }
      }
      if (carries_prior) {
        const std::optional<Constraint> prior = MakePriorConstraint(
            survivor, survivor_pose, survivor_covariance, option_.min_recovered_stddev);
        if (prior.has_value()) {
          added_constraints.push_back(*prior);
          ++num_priors_added_;
        } else {
          LOG(WARNING) << "anchor prior dropped: covariance not invertible";
        }
      }
    }
  } else {
    VLOG(1) << "trim with fewer than two survivors: information dropped, no tree";
  }

  TrimRequest request;
  request.deleted_submap_ids.push_back(id);
  const std::optional<SubmapId> successor = PickSuccessor(id, removed);
  if (successor.has_value()) {
    request.successors.emplace(id, *successor);
  }
  request.added_constraints = std::move(added_constraints);

  reports_.push_back(handle_.ApplyTrim(request));
  ++num_submaps_trimmed_;
  return true;
}

std::optional<SubmapId> PoseGraphTrimmer::PickSuccessor(const SubmapId& deleted,
                                                        const std::set<VariableId>& removed) const {
  const PoseGraphData& graph = handle_.graph();
  const SubmapRecord& record = graph.submap(deleted);
  const std::vector<std::int64_t>& deleted_cells = selector_.Footprint(record);
  const std::unordered_set<std::int64_t> deleted_set(deleted_cells.begin(), deleted_cells.end());

  std::optional<SubmapId> best;
  size_t best_overlap = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (const auto& [candidate_id, candidate] : graph.submaps()) {
    if (candidate_id == deleted || removed.count(VariableId::Of(candidate_id)) > 0) {
      continue;
    }
    // An unfinished grid belongs to the frontend thread.
    if (candidate.submap != nullptr && !candidate.submap->finished()) {
      continue;
    }
    size_t overlap = 0;
    for (const std::int64_t cell : selector_.Footprint(candidate)) {
      if (deleted_set.count(cell) > 0) {
        ++overlap;
      }
    }
    const double distance =
        (candidate.global_pose.translation() - record.global_pose.translation()).norm();
    if (!best.has_value() || overlap > best_overlap ||
        (overlap == best_overlap && distance < best_distance)) {
      best = candidate_id;
      best_overlap = overlap;
      best_distance = distance;
    }
  }
  return best;
}

std::optional<Constraint> PoseGraphTrimmer::MaybeConvertLongEdgeToPrior(const Constraint& binary) {
  CHECK(binary.to.has_value());
  if (!handle_.optimization().IsSessionAnchoredToFrozen(binary.to->session())) {
    return std::nullopt;
  }
  // The marginal, not the conditional joint the tree was built from: a prior's weight must not
  // inherit the conditional's systematic understatement.
  const std::optional<Eigen::Matrix3d> marginal =
      covariance_evaluator_.ComputeMarginalCovarianceInGlobal(*binary.to);
  if (!marginal.has_value()) {
    return std::nullopt;
  }
  const double translation_stddev = std::sqrt(std::max((*marginal)(0, 0), (*marginal)(1, 1)));
  const double rotation_stddev = std::sqrt((*marginal)(2, 2));
  if (translation_stddev > option_.prior_max_translation_stddev ||
      rotation_stddev > option_.prior_max_rotation_stddev) {
    return std::nullopt;
  }
  return MakePriorConstraint(*binary.to, handle_.optimization().GetVariablePose(*binary.to),
                             *marginal, option_.min_recovered_stddev);
}

}  // namespace evergreenslam::lifelong
