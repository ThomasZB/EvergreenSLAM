/**
 * @file constraint_builder_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_OPTION_H_
#define EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

#include "lifelong/constraints/constraint_sampler.h"
#include "utils/scan_matching/fast_correlative_scan_matcher_option.h"

namespace evergreenslam::lifelong {

struct ConstraintBuilderOption {
  utils::scan_matching::FastCorrelativeScanMatcherOption matcher_option;
  ConstraintSamplerOption sampler_option;

  double candidate_slack_per_meter = 0.05;  // m per meter of path since the last valid closure
  double max_candidate_slack = 30.0;        // m
  // The graph-prior coarse window is this plus the pair's slack, so a session that closes often
  // searches well under tight_window_max.
  double search_window_floor = 0.5;  // m
  // A same-session closure resets a session's drift only when the node is at least this many
  // node indices away from the submap's node range; nearer pairs are odometry-scale and tie
  // nothing. Cross-session always.
  int min_loop_node_gap = 20;
  // Coarse windows around an operator hint, which replaces the graph's estimate as the prior.
  double relocalization_linear_search_window = 2.0;   // m
  double relocalization_angular_search_window = 0.5;  // rad
  // reseed threshold
  double relocalization_min_score = 0.2;
  double max_constraint_distance = 15.0;  // m

  // Per-pair search window at or below this uses tight_min_score: a window that cannot alias far
  // may accept a scene that changed since the map was built. Wider windows keep the strict score.
  double tight_window_max = 1.0;  // m
  double tight_min_score = 0.4;
  double wide_min_score = 0.55;
  double max_refinement_translation = 0.25;  // m
  double max_refinement_rotation = 0.1;      // rad
  // Corridor gate: an alias is a genuine local optimum, so no pointwise criterion sees it.
  double prominence_translation = 0.5;  // m
  double prominence_rotation = 0.3;     // rad
  double min_score_prominence = 0.1;

  int precomputation_cache_size = 32;

  // 0 runs matches inline on the backend task, PoseGraph replaces 0 with a hardware default.
  int num_match_workers = 0;

  friend std::ostream& operator<<(std::ostream& os, const ConstraintBuilderOption& option) {
    os << "ConstraintBuilderOption:" << std::endl;
    os << "  candidate_slack_per_meter: " << option.candidate_slack_per_meter << std::endl;
    os << "  max_candidate_slack: " << option.max_candidate_slack << std::endl;
    os << "  search_window_floor: " << option.search_window_floor << std::endl;
    os << "  min_loop_node_gap: " << option.min_loop_node_gap << std::endl;
    os << "  relocalization_linear_search_window: " << option.relocalization_linear_search_window
       << std::endl;
    os << "  relocalization_angular_search_window: " << option.relocalization_angular_search_window
       << std::endl;
    os << "  relocalization_min_score: " << option.relocalization_min_score << std::endl;
    os << "  max_constraint_distance: " << option.max_constraint_distance << std::endl;
    os << "  tight_window_max: " << option.tight_window_max << std::endl;
    os << "  tight_min_score: " << option.tight_min_score << std::endl;
    os << "  wide_min_score: " << option.wide_min_score << std::endl;
    os << "  max_refinement_translation: " << option.max_refinement_translation << std::endl;
    os << "  max_refinement_rotation: " << option.max_refinement_rotation << std::endl;
    os << "  prominence_translation: " << option.prominence_translation << std::endl;
    os << "  prominence_rotation: " << option.prominence_rotation << std::endl;
    os << "  min_score_prominence: " << option.min_score_prominence << std::endl;
    os << "  precomputation_cache_size: " << option.precomputation_cache_size << std::endl;
    os << "  num_match_workers: " << option.num_match_workers << std::endl;
    os << "  sampling_ratio: " << option.sampler_option.sampling_ratio << std::endl;
    os << "  max_matches_per_round: " << option.sampler_option.max_matches_per_round << std::endl;
    os << option.matcher_option;
    return os;
  }
};

ConstraintBuilderOption LoadConstraintBuilderOption(const YAML::Node& node);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_BUILDER_OPTION_H_
