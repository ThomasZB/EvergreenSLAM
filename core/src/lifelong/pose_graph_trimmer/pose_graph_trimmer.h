/**
 * @file pose_graph_trimmer.h
 * @author hang chen (chen@hang.plus)
 * @brief The trimming pipeline: blanket, joint covariance, Chow-Liu tree, replacement, report.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_H_

#include <atomic>
#include <deque>
#include <optional>
#include <set>
#include <vector>

#include "lifelong/backend_handles.h"
#include "lifelong/optimization/covariance_evaluator.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer_option.h"
#include "lifelong/pose_graph_trimmer/trim_selector.h"

namespace evergreenslam::lifelong {

class PoseGraphTrimmer {
 public:
  explicit PoseGraphTrimmer(TrimmingHandle& handle,
                            const PoseGraphTrimmerOption& option = PoseGraphTrimmerOption());

  void TrimOnce();
  void TrimToCompletionOnTask();
  void EnqueueForTrim(const SubmapId& id);

  const std::vector<TrimReport>& reports() const { return reports_; }
  size_t pending_count() const { return pending_.size(); }

  int num_submaps_trimmed() const { return num_submaps_trimmed_.load(); }
  int num_priors_added() const { return num_priors_added_.load(); }

 private:
  void RunRound();
  bool TrimSubmap(const SubmapId& id);
  std::optional<SubmapId> PickSuccessor(const SubmapId& deleted,
                                        const std::set<VariableId>& removed) const;
  // Weighted by the marginal, never the conditional one, which understates uncertainty.
  std::optional<Constraint> MaybeConvertLongEdgeToPrior(const Constraint& binary);

  TrimmingHandle& handle_;
  PoseGraphTrimmerOption option_;
  TrimSelector selector_;
  CovarianceEvaluator covariance_evaluator_;

  std::deque<SubmapId> pending_;
  std::vector<TrimReport> reports_;
  std::atomic<int> num_submaps_trimmed_{0};
  std::atomic<int> num_priors_added_{0};
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_H_
