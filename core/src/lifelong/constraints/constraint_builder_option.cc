/**
 * @file constraint_builder_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/constraint_builder_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

using utils::config::Child;
using utils::config::LoadOption;

ConstraintBuilderOption LoadConstraintBuilderOption(const YAML::Node& node) {
  ConstraintBuilderOption option;
  option.matcher_option = utils::scan_matching::LoadFastCorrelativeScanMatcherOption(
      Child(node, "fast_correlative_scan_matcher"));

  const YAML::Node sampler = Child(node, "constraint_sampler");
  option.sampler_option.sampling_ratio =
      LoadOption(sampler, "sampling_ratio", option.sampler_option.sampling_ratio);
  option.sampler_option.max_matches_per_round =
      LoadOption(sampler, "max_matches_per_round", option.sampler_option.max_matches_per_round);

  option.candidate_slack_per_meter =
      LoadOption(node, "candidate_slack_per_meter", option.candidate_slack_per_meter);
  option.max_candidate_slack = LoadOption(node, "max_candidate_slack", option.max_candidate_slack);
  option.search_window_floor = LoadOption(node, "search_window_floor", option.search_window_floor);
  option.min_loop_node_gap = LoadOption(node, "min_loop_node_gap", option.min_loop_node_gap);
  option.relocalization_linear_search_window = LoadOption(
      node, "relocalization_linear_search_window", option.relocalization_linear_search_window);
  option.relocalization_angular_search_window = LoadOption(
      node, "relocalization_angular_search_window", option.relocalization_angular_search_window);
  option.relocalization_min_score =
      LoadOption(node, "relocalization_min_score", option.relocalization_min_score);
  option.max_constraint_distance =
      LoadOption(node, "max_constraint_distance", option.max_constraint_distance);
  option.tight_window_max = LoadOption(node, "tight_window_max", option.tight_window_max);
  option.tight_min_score = LoadOption(node, "tight_min_score", option.tight_min_score);
  option.wide_min_score = LoadOption(node, "wide_min_score", option.wide_min_score);
  option.max_refinement_translation =
      LoadOption(node, "max_refinement_translation", option.max_refinement_translation);
  option.max_refinement_rotation =
      LoadOption(node, "max_refinement_rotation", option.max_refinement_rotation);
  option.prominence_translation =
      LoadOption(node, "prominence_translation", option.prominence_translation);
  option.prominence_rotation = LoadOption(node, "prominence_rotation", option.prominence_rotation);
  option.min_score_prominence =
      LoadOption(node, "min_score_prominence", option.min_score_prominence);
  option.precomputation_cache_size =
      LoadOption(node, "precomputation_cache_size", option.precomputation_cache_size);
  option.num_match_workers = LoadOption(node, "num_match_workers", option.num_match_workers);
  return option;
}

}  // namespace evergreenslam::lifelong
