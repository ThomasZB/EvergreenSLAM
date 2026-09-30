/**
 * @file constraint_builder.h
 * @author hang chen (chen@hang.plus)
 * @brief Loop closure search: coarse match, refinement, two-gate acceptance, constraint out.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_H_
#define EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "lifelong/backend_handles.h"
#include "lifelong/constraints/constraint_builder_option.h"
#include "lifelong/constraints/constraint_sampler.h"
#include "lifelong/constraints/drift_tracker.h"
#include "lifelong/constraints/match_worker_pool.h"
#include "lifelong/optimization/optimization_option.h"
#include "utils/scan_matching/fast_correlative_scan_matcher.h"

namespace evergreenslam::lifelong {

// Matching is per submap pair on purpose: the multi-submap Match() takes a global max, so a
// submap with unbalanced evidence can win with an aliased pose.
//
// Candidate selection, sampling and result application stay on the backend task; only the
// stateless per-pair match chain fans out to workers.
class ConstraintBuilder {
 public:
  explicit ConstraintBuilder(ConstraintHandle& handle,
                             const ConstraintBuilderOption& option = ConstraintBuilderOption(),
                             const ConstraintWeightOption& weight = ConstraintWeightOption());

  void SearchForNode(const NodeId& node_id, int budget = -1);
  // Not round-gated: a submap finishing is rare.
  void SearchForSubmap(const SubmapId& submap_id, int budget = -1);
  void SearchForNodeOnTask(const NodeId& node_id, int budget = -1);
  void SearchForSubmapOnTask(const SubmapId& submap_id, int budget = -1);
  // Operator relocalization, so no attempted_pairs_ or drift gate: a resent pose retries pairs.
  // With a prior, finished submaps of other sessions whose footprint reaches it within the
  // relocalization window are matched from the prior; without one, whole grids over every heading.
  void SearchForNodeAroundOnTask(const NodeId& node_id, const std::optional<Eigen::Affine2d>& prior,
                                 int budget = -1);

  // Results already enqueued still need a queue drain; WaitUntilQuiescent alternates the two.
  void WaitForMatches() const;
  void SweepStaleStateOnTask();

  int num_matches_attempted() const { return num_matches_attempted_.load(); }
  int num_constraints_added() const { return num_constraints_added_.load(); }
  // Results whose endpoint was trimmed before they landed.
  int num_constraints_dropped() const { return num_constraints_dropped_.load(); }
  int num_precomputation_builds() const { return num_precomputation_builds_.load(); }
  // Backend task state; only meaningful to a caller that has drained the queue.
  int num_cached_precomputations() const { return static_cast<int>(precomputation_cache_.size()); }

 private:
  enum class PriorSource { GRAPH, EXTERNAL, NONE };

  // Resolved on the backend task: a worker must never touch graph state it cannot pin.
  struct MatchInput {
    NodeId node_id;
    SubmapId submap_id;
    std::shared_ptr<const mapping::Submap> submap;
    Eigen::Affine2d grid_to_global = Eigen::Affine2d::Identity();
    // Absent for the whole-grid, every-heading search.
    std::optional<Eigen::Affine2d> prior;
    double linear_search_window = 0.0;
    std::optional<double> angular_search_window;
    AcceptanceGates gates{};
    // Immutable and self-contained, so a worker holding this copy survives eviction.
    std::shared_ptr<const utils::scan_matching::PrecomputationGridStack> precomputation;
    // Copied: the node can be trimmed while the match runs.
    sensor::PointCloud cloud;
  };

  // Weak by design: the cache must never keep a trimmed grid alive.
  struct PrecomputationEntry {
    std::weak_ptr<const mapping::Submap> submap;
    std::shared_ptr<const utils::scan_matching::PrecomputationGridStack> stack;
    std::list<SubmapId>::iterator lru;
  };

  std::vector<LoopCandidate> CandidatesForNode(const NodeId& node_id) const;
  std::vector<LoopCandidate> CandidatesForSubmap(const SubmapId& submap_id) const;
  std::vector<LoopCandidate> CandidatesAround(const NodeId& node_id,
                                              const std::optional<Eigen::Affine2d>& prior) const;
  std::optional<LoopCandidate> CandidateFor(const NodeId& node_id, const SubmapId& submap_id,
                                            const SubmapRecord& record) const;
  bool IsEligiblePair(const NodeId& node_id, const SubmapId& submap_id,
                      const SubmapRecord& record) const;
  void RunRound(std::vector<LoopCandidate> candidates, int budget,
                PriorSource source = PriorSource::GRAPH,
                const Eigen::Affine2d& external_prior = Eigen::Affine2d::Identity());
  std::optional<MatchInput> PrepareMatch(const LoopCandidate& candidate, PriorSource source,
                                         const Eigen::Affine2d& external_prior);
  std::shared_ptr<const utils::scan_matching::PrecomputationGridStack> PrecomputationFor(
      const SubmapId& submap_id, const std::shared_ptr<const mapping::Submap>& submap);
  // Thread safe: reads only the input and the stateless coarse matcher.
  std::optional<Constraint> RunMatch(const MatchInput& input) const;
  void ApplyMatchResult(const std::optional<Constraint>& constraint);

  ConstraintHandle& handle_;
  ConstraintBuilderOption option_;
  Eigen::Matrix3d loop_closure_sqrt_information_;
  ConstraintSampler sampler_;
  utils::scan_matching::FastCorrelativeScanMatcher coarse_matcher_;
  // Null when num_match_workers == 0; matches then run inline on the backend task.
  std::unique_ptr<MatchWorkerPool> pool_;

  // Backend task only.
  DriftTracker drift_;
  // A pair is matched at most once. Backend task only.
  std::set<std::pair<NodeId, SubmapId>> attempted_pairs_;
  int searches_since_sweep_ = 0;
  // Backend task only. Front of the list is most recently used.
  std::map<SubmapId, PrecomputationEntry> precomputation_cache_;
  std::list<SubmapId> precomputation_lru_;
  std::atomic<int> num_matches_attempted_{0};
  std::atomic<int> num_constraints_added_{0};
  std::atomic<int> num_constraints_dropped_{0};
  std::atomic<int> num_precomputation_builds_{0};
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_H_
