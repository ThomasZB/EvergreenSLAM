/**
 * @file constraint_builder.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/constraint_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <utility>

#include "lifelong/optimization/optimization_option.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/grid_mapping/probability_values.h"
#include "utils/scan_matching/ceres_scan_matcher.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

// Unknown and out-of-grid read as minimum probability: the same scale every matcher reports.
double ScoreAtPose(const mapping::GridMapu8& grid, const sensor::PointCloud& cloud,
                   const Eigen::Affine2d& pose) {
  if (cloud.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto& point : cloud) {
    sum += mapping::ValueToProbability(grid.GetValueAtPoint(pose * point.point));
  }
  return sum / static_cast<double>(cloud.size());
}

// Finished grids only: an unfinished known_area() moves under the frontend's inserts.
Eigen::AlignedBox2d KnownAreaInGrid(const mapping::ProbabilityGrid& grid) {
  const Eigen::AlignedBox2i& cells = grid.known_area();
  const Eigen::Vector2d origin(grid.origin_x(), grid.origin_y());
  const double res = grid.resolution();
  return Eigen::AlignedBox2d(origin + cells.min().cast<double>() * res,
                             origin + (cells.max().array() + 1).cast<double>().matrix() * res);
}

bool SubmapReaches(const SubmapRecord& record, const Eigen::Vector2d& global_point, double margin) {
  const Eigen::Vector2d in_grid = record.global_pose.inverse() * global_point;
  return KnownAreaInGrid(record.submap->grid()).exteriorDistance(in_grid) <= margin;
}

Eigen::AlignedBox2d GlobalBox(const SubmapRecord& record) {
  const Eigen::AlignedBox2d box = KnownAreaInGrid(record.submap->grid());
  Eigen::AlignedBox2d global;
  for (int corner = 0; corner < 4; ++corner) {
    global.extend(record.global_pose *
                  box.corner(static_cast<Eigen::AlignedBox2d::CornerType>(corner)));
  }
  return global;
}

}  // namespace

ConstraintBuilder::ConstraintBuilder(ConstraintHandle& handle,
                                     const ConstraintBuilderOption& option,
                                     const ConstraintWeightOption& weight)
    : handle_(handle),
      option_(option),
      loop_closure_sqrt_information_(LoopClosureSqrtInformation(weight)),
      sampler_(option.sampler_option),
      coarse_matcher_(option.matcher_option),
      drift_(option) {
  if (option_.num_match_workers > 0) {
    pool_ = std::make_unique<MatchWorkerPool>(option_.num_match_workers);
  }
}

void ConstraintBuilder::SearchForNode(const NodeId& node_id, int budget) {
  handle_.Enqueue([this, node_id, budget] { SearchForNodeOnTask(node_id, budget); });
}

void ConstraintBuilder::SearchForSubmap(const SubmapId& submap_id, int budget) {
  handle_.Enqueue([this, submap_id, budget] { SearchForSubmapOnTask(submap_id, budget); });
}

void ConstraintBuilder::SearchForNodeOnTask(const NodeId& node_id, int budget) {
  const PoseGraphData& graph = handle_.graph();
  if (!graph.HasNode(node_id)) {
    return;
  }
  drift_.RecordNode(graph, node_id);
  // Trimming and session GC remove ids without telling the builder; without this sweep the maps
  // grow for the lifetime of the process.
  if (++searches_since_sweep_ >= 256) {
    searches_since_sweep_ = 0;
    SweepStaleState();
  }
  if (!sampler_.ShouldSampleRound()) {
    return;
  }
  RunRound(CandidatesForNode(node_id), budget);
}

void ConstraintBuilder::SearchForSubmapOnTask(const SubmapId& submap_id, int budget) {
  const PoseGraphData& graph = handle_.graph();
  if (!graph.HasSubmap(submap_id) || !graph.submap(submap_id).submap->finished() ||
      graph.submap(submap_id).submap->grid().empty()) {
    return;
  }
  RunRound(CandidatesForSubmap(submap_id), budget);
}

void ConstraintBuilder::SearchForNodeAroundOnTask(const NodeId& node_id,
                                                  const std::optional<Eigen::Affine2d>& prior,
                                                  int budget) {
  if (!handle_.graph().HasNode(node_id)) {
    return;
  }
  if (prior.has_value()) {
    RunRound(CandidatesAround(node_id, prior), budget, PriorSource::EXTERNAL, *prior);
    return;
  }
  RunRound(CandidatesAround(node_id, std::nullopt), std::numeric_limits<int>::max(),
           PriorSource::NONE);
}

void ConstraintBuilder::WaitForMatches() const {
  if (pool_ != nullptr) {
    pool_->WaitIdle();
  }
}

std::vector<LoopCandidate> ConstraintBuilder::CandidatesForNode(const NodeId& node_id) const {
  std::vector<LoopCandidate> candidates;
  for (const auto& [submap_id, record] : handle_.graph().submaps()) {
    if (const std::optional<LoopCandidate> candidate = CandidateFor(node_id, submap_id, record)) {
      candidates.push_back(*candidate);
    }
  }
  return candidates;
}

std::vector<LoopCandidate> ConstraintBuilder::CandidatesForSubmap(const SubmapId& submap_id) const {
  const PoseGraphData& graph = handle_.graph();
  const SubmapRecord& record = graph.submap(submap_id);
  const Eigen::AlignedBox2d footprint = GlobalBox(record);
  const Eigen::Vector2d slack = Eigen::Vector2d::Constant(option_.max_candidate_slack);
  const Eigen::AlignedBox2d reach(footprint.min() - slack, footprint.max() + slack);

  std::vector<LoopCandidate> candidates;
  std::set<NodeId> seen;
  seen.insert(record.node_ids.begin(), record.node_ids.end());
  for (const auto& [other_id, other] : graph.submaps()) {
    if (other_id == submap_id) {
      continue;
    }
    // With free space inserted a node sits inside its own submap's footprint, so a finished
    // submap out of reach holds no candidate; an unfinished one has no stable footprint to test.
    if (other.submap->finished() && !other.submap->grid().empty() &&
        !GlobalBox(other).intersects(reach)) {
      continue;
    }
    for (const NodeId& node_id : other.node_ids) {
      if (!seen.insert(node_id).second) {
        continue;
      }
      if (const std::optional<LoopCandidate> candidate = CandidateFor(node_id, submap_id, record)) {
        candidates.push_back(*candidate);
      }
    }
  }
  return candidates;
}

std::optional<LoopCandidate> ConstraintBuilder::CandidateFor(const NodeId& node_id,
                                                             const SubmapId& submap_id,
                                                             const SubmapRecord& record) const {
  if (!IsEligiblePair(node_id, submap_id, record)) {
    return std::nullopt;
  }
  const Node& node = handle_.graph().node(node_id);
  const double slack = drift_.Slack(handle_.graph(), node_id, submap_id);
  if (!SubmapReaches(record, node.global_pose.translation(), slack)) {
    return std::nullopt;
  }
  const double distance =
      (node.global_pose.translation() - record.global_pose.translation()).norm();
  if (distance > option_.max_constraint_distance) {
    return std::nullopt;
  }
  return LoopCandidate{node_id, submap_id, distance, slack,
                       !(SessionOf(submap_id) == SessionOf(node_id))};
}

std::vector<LoopCandidate> ConstraintBuilder::CandidatesAround(
    const NodeId& node_id, const std::optional<Eigen::Affine2d>& prior) const {
  const PoseGraphData& graph = handle_.graph();
  std::vector<LoopCandidate> candidates;
  for (const auto& [submap_id, record] : graph.submaps()) {
    if (SessionOf(submap_id) == SessionOf(node_id) || !record.submap->finished() ||
        record.submap->grid().empty()) {
      continue;
    }
    double distance = 0.0;
    if (prior.has_value()) {
      if (!SubmapReaches(record, prior->translation(),
                         option_.relocalization_linear_search_window)) {
        continue;
      }
      distance = (prior->translation() - record.global_pose.translation()).norm();
    }
    candidates.push_back(LoopCandidate{node_id, submap_id, distance, 0.0, true});
  }
  return candidates;
}

void ConstraintBuilder::SweepStaleState() {
  const PoseGraphData& graph = handle_.graph();
  drift_.Sweep(graph);
  for (auto it = precomputation_cache_.begin(); it != precomputation_cache_.end();) {
    if (graph.HasSubmap(it->first)) {
      ++it;
      continue;
    }
    precomputation_lru_.erase(it->second.lru);
    it = precomputation_cache_.erase(it);
  }
}

bool ConstraintBuilder::IsEligiblePair(const NodeId& node_id, const SubmapId& submap_id,
                                       const SubmapRecord& record) const {
  // Only finished submaps: an unfinished grid still changes under the matcher, which is a race.
  if (!record.submap->finished() || record.submap->grid().empty()) {
    return false;
  }
  if (record.node_ids.count(node_id) > 0) {
    return false;
  }
  // Two frozen endpoints are both absolute truth; a constraint between them moves nothing.
  const PoseGraphData& graph = handle_.graph();
  if (graph.session(SessionOf(node_id)).frozen() && graph.session(SessionOf(submap_id)).frozen()) {
    return false;
  }
  return attempted_pairs_.count({node_id, submap_id}) == 0;
}

void ConstraintBuilder::RunRound(std::vector<LoopCandidate> candidates, int budget,
                                 PriorSource source, const Eigen::Affine2d& external_prior) {
  const std::vector<LoopCandidate> selected = sampler_.Select(std::move(candidates), budget);
  for (const LoopCandidate& candidate : selected) {
    attempted_pairs_.insert({candidate.node_id, candidate.submap_id});
    num_matches_attempted_.fetch_add(1);
    std::optional<MatchInput> input = PrepareMatch(candidate, source, external_prior);
    if (!input.has_value()) {
      continue;
    }
    if (pool_ == nullptr) {
      ApplyMatchResult(RunMatch(*input));
      continue;
    }
    pool_->Submit([this, input = std::move(*input)] {
      const std::optional<Constraint> constraint = RunMatch(input);
      // Graph writes happen on the backend task, never on a worker.
      handle_.Enqueue([this, constraint] { ApplyMatchResult(constraint); });
    });
  }
}

std::optional<ConstraintBuilder::MatchInput> ConstraintBuilder::PrepareMatch(
    const LoopCandidate& candidate, PriorSource source, const Eigen::Affine2d& external_prior) {
  const PoseGraphData& graph = handle_.graph();
  const Node& node = graph.node(candidate.node_id);
  const SubmapRecord& record = graph.submap(candidate.submap_id);
  if (node.constant_data.point_cloud.empty()) {
    return std::nullopt;
  }
  MatchInput input;
  input.node_id = candidate.node_id;
  input.submap_id = candidate.submap_id;
  input.submap = record.submap;
  // Force the lazy snapshot here, on the backend task; workers then only read it.
  record.submap->Snapshot();
  input.grid_to_global = record.global_pose;
  switch (source) {
    case PriorSource::GRAPH:
      input.prior = node.global_pose;
      input.linear_search_window = option_.search_window_floor + candidate.search_slack;
      break;
    case PriorSource::EXTERNAL:
      input.prior = external_prior;
      input.linear_search_window = option_.relocalization_linear_search_window;
      input.angular_search_window = option_.relocalization_angular_search_window;
      break;
    case PriorSource::NONE:
      break;
  }
  input.gates =
      source != PriorSource::NONE && input.linear_search_window <= option_.tight_window_max
          ? option_.tight_gates
          : option_.wide_gates;
  input.cloud = node.constant_data.point_cloud;
  input.precomputation = PrecomputationFor(candidate.submap_id, record.submap);
  return input;
}

std::shared_ptr<const utils::scan_matching::PrecomputationGridStack>
ConstraintBuilder::PrecomputationFor(const SubmapId& submap_id,
                                     const std::shared_ptr<const mapping::Submap>& submap) {
  if (option_.precomputation_cache_size <= 0) {
    return nullptr;
  }
  DCHECK(submap->finished());
  const auto it = precomputation_cache_.find(submap_id);
  if (it != precomputation_cache_.end()) {
    if (it->second.submap.lock() == submap) {
      precomputation_lru_.splice(precomputation_lru_.begin(), precomputation_lru_, it->second.lru);
      return it->second.stack;
    }
    precomputation_lru_.erase(it->second.lru);
    precomputation_cache_.erase(it);
  }
  while (static_cast<int>(precomputation_cache_.size()) >= option_.precomputation_cache_size) {
    precomputation_cache_.erase(precomputation_lru_.back());
    precomputation_lru_.pop_back();
  }
  PrecomputationEntry entry;
  entry.submap = submap;
  entry.stack = coarse_matcher_.BuildPrecomputationStack(submap->Snapshot());
  precomputation_lru_.push_front(submap_id);
  entry.lru = precomputation_lru_.begin();
  const std::shared_ptr<const utils::scan_matching::PrecomputationGridStack> stack = entry.stack;
  precomputation_cache_.emplace(submap_id, std::move(entry));
  num_precomputation_builds_.fetch_add(1);
  return stack;
}

std::optional<Constraint> ConstraintBuilder::RunMatch(const MatchInput& input) const {
  const mapping::GridMapu8& grid = input.submap->Snapshot();
  const sensor::PointCloud& cloud = input.cloud;

  const std::vector<utils::scan_matching::CandidateSubmap> search_grids{
      utils::scan_matching::CandidateSubmap{input.grid_to_global, grid}};
  utils::scan_matching::FastCorrelativeScanMatcher::MatchParams params;
  params.linear_search_window = input.linear_search_window;
  params.min_score = input.gates.min_coarse_score;
  params.angular_search_window = input.angular_search_window;
  params.stack = input.precomputation;
  const std::optional<utils::scan_matching::GlobalMatchResult> coarse =
      coarse_matcher_.Match(cloud, search_grids, input.prior, params);
  if (!coarse.has_value()) {
    return std::nullopt;
  }

  const Eigen::Affine2d coarse_in_grid =
      Eigen::Affine2d(input.grid_to_global.inverse() * coarse->pose);
  Eigen::Affine2d refined_in_grid = coarse_in_grid;
  // Local, so the worker path holds no shared mutable state. No pose prior: one anchored at the
  // coarse result would only ratify whatever the coarse stage picked.
  utils::scan_matching::CeresScanMatcher refiner;
  refiner.Match(cloud, grid, refined_in_grid);

  const double refinement_translation =
      (refined_in_grid.translation() - coarse_in_grid.translation()).norm();
  const double refinement_rotation = std::abs(transform::NormalizeAngle(
      transform::GetYaw(refined_in_grid) - transform::GetYaw(coarse_in_grid)));
  if (refinement_translation > option_.max_refinement_translation ||
      refinement_rotation > option_.max_refinement_rotation) {
    return std::nullopt;
  }

  const double refined_score = ScoreAtPose(grid, cloud, refined_in_grid);
  if (refined_score < input.gates.min_refined_score) {
    return std::nullopt;
  }

  // A corridor alias is a genuine local optimum and sails through every other gate.
  double best_alias_score = 0.0;
  for (int k = 0; k < 8; ++k) {
    const double angle = static_cast<double>(k) * M_PI / 4.0;
    Eigen::Affine2d displaced = refined_in_grid;
    displaced.translation() +=
        option_.prominence_translation * Eigen::Vector2d(std::cos(angle), std::sin(angle));
    best_alias_score = std::max(best_alias_score, ScoreAtPose(grid, cloud, displaced));
  }
  for (const double sign : {-1.0, 1.0}) {
    const Eigen::Affine2d rotated = Eigen::Affine2d(
        refined_in_grid * transform::FromXYTheta(0.0, 0.0, sign * option_.prominence_rotation));
    best_alias_score = std::max(best_alias_score, ScoreAtPose(grid, cloud, rotated));
  }
  if (refined_score - best_alias_score < option_.min_score_prominence) {
    return std::nullopt;
  }

  // A mean score cannot tell "half the scan contradicts the map" from "everything matches
  // mediocrely". Unknown cells do not count; occlusion is normal.
  double free_points = 0.0;
  for (const auto& point : cloud) {
    const uint8_t value = grid.GetValueAtPoint(refined_in_grid * point.point);
    if (mapping::IsKnownValue(value) &&
        mapping::ValueToProbability(value) <= option_.free_space_probability) {
      free_points += 1.0;
    }
  }
  if (free_points / static_cast<double>(cloud.size()) > input.gates.max_free_space_fraction) {
    return std::nullopt;
  }

  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = VariableId::Of(input.submap_id);
  constraint.to = VariableId::Of(input.node_id);
  constraint.relative_pose = refined_in_grid;
  constraint.sqrt_information =
      LeverArmSqrtInformation(loop_closure_sqrt_information_, refined_in_grid.translation().norm());
  return constraint;
}

void ConstraintBuilder::ApplyMatchResult(const std::optional<Constraint>& constraint) {
  if (!constraint.has_value()) {
    return;
  }
  if (!handle_.AddConstraintIfEndpointsLive(*constraint)) {
    num_constraints_dropped_.fetch_add(1);
    return;
  }
  num_constraints_added_.fetch_add(1);
  drift_.NoteClosure(handle_.graph(), constraint->to->node_id(), constraint->from.submap_id());
}

}  // namespace evergreenslam::lifelong
