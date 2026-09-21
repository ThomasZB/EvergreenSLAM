/**
 * @file pose_graph_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_option.h"

#include <glog/logging.h>

#include "utils/config/yaml_utils.h"
#include "utils/scan_matching/fast_correlative_scan_matcher_option.h"

namespace evergreenslam::lifelong {

using utils::config::Child;
using utils::config::LoadOption;

const utils::config::Schema& LifelongSchema() {
  static const utils::config::Schema schema = {
      {"",
       {"optimization", "constraint_weight", "session_manager", "constraint_builder", "trimmer",
        "trim", "trim_every_n_optimizations", "checkpoint_min_interval",
        "gc_unfrozen_after_n_boots"}},
      {"optimization", {"optimize_every_n_nodes", "max_num_iterations", "num_threads"}},
      {"constraint_weight",
       {"odometry_translation_stddev", "odometry_rotation_stddev",
        "loop_closure_translation_stddev", "loop_closure_rotation_stddev",
        "loop_closure_huber_distance"}},
      {"session_manager",
       {"auto_freeze", "min_new_area", "min_new_area_fraction", "growth_tolerance",
        "track_growth_tolerance", "submaps_without_growth", "coverage_resolution",
        "track_resolution", "frozen_covered_fraction", "freeze_judge"}},
      {"session_manager/freeze_judge", {"max_translation_stddev", "max_rotation_stddev"}},
      // The loop matcher's angular window and depth live in the top level global_scan_matcher
      // section; its linear window and score floor are set here per pair.
      {"constraint_builder",
       {"constraint_sampler", "candidate_slack_per_meter", "max_candidate_slack",
        "search_window_floor", "min_loop_node_gap", "relocalization_linear_search_window",
        "relocalization_angular_search_window", "max_constraint_distance", "tight_window_max",
        "tight_gates", "wide_gates", "max_refinement_translation", "max_refinement_rotation",
        "prominence_translation", "prominence_rotation", "min_score_prominence",
        "free_space_probability", "precomputation_cache_size", "num_match_workers"}},
      {"constraint_builder/constraint_sampler", {"sampling_ratio", "max_matches_per_round"}},
      {"constraint_builder/tight_gates",
       {"min_coarse_score", "min_refined_score", "max_free_space_fraction"}},
      {"constraint_builder/wide_gates",
       {"min_coarse_score", "min_refined_score", "max_free_space_fraction"}},
      {"trimmer",
       {"selector", "max_submaps_per_round", "min_recovered_stddev", "max_binary_constraint_length",
        "prior_max_translation_stddev", "prior_max_rotation_stddev"}},
      {"trimmer/selector",
       {"submap_coverage_resolution", "node_coverage_resolution", "min_covered_fraction",
        "min_covering_newer_submaps", "min_covered_node_fraction", "coverage_mode",
        "min_surviving_submaps", "keep_newest_submaps"}},
  };
  return schema;
}

const utils::config::Schema& GlobalScanMatcherSchema() {
  static const utils::config::Schema schema = {
      {"", {"fast_correlative"}},
      {"fast_correlative", {"angular_search_window", "branch_and_bound_depth"}},
  };
  return schema;
}

PoseGraphOption LoadPoseGraphOption(const YAML::Node& root) {
  const YAML::Node node = Child(root, "lifelong");
  PoseGraphOption option;
  option.optimization = LoadOptimizationOption(Child(node, "optimization"));
  option.constraint_weight = LoadConstraintWeightOption(Child(node, "constraint_weight"));
  option.session_manager = LoadSessionManagerOption(Child(node, "session_manager"));
  option.constraint_builder = LoadConstraintBuilderOption(Child(node, "constraint_builder"));
  option.constraint_builder.matcher_option =
      utils::scan_matching::LoadFastCorrelativeScanMatcherOption(
          Child(Child(root, "global_scan_matcher"), "fast_correlative"));
  option.trimmer = LoadPoseGraphTrimmerOption(Child(node, "trimmer"));
  option.trim = LoadOption(node, "trim", option.trim);
  option.trim_every_n_optimizations =
      LoadOption(node, "trim_every_n_optimizations", option.trim_every_n_optimizations);
  option.checkpoint_min_interval = common::FromSeconds(LoadOption(
      node, "checkpoint_min_interval", common::ToSeconds(option.checkpoint_min_interval)));
  option.gc_unfrozen_after_n_boots =
      LoadOption(node, "gc_unfrozen_after_n_boots", option.gc_unfrozen_after_n_boots);
  CHECK_GE(option.trimmer.min_recovered_stddev,
           option.constraint_weight.odometry_translation_stddev)
      << "a recovered constraint may not be tighter than the raw measurements it summarizes";
  return option;
}

std::vector<std::string> FindUnknownLifelongKeys(const YAML::Node& node) {
  return utils::config::FindUnknownKeys(node, LifelongSchema());
}

std::vector<std::string> FindUnknownGlobalScanMatcherKeys(const YAML::Node& node) {
  return utils::config::FindUnknownKeys(node, GlobalScanMatcherSchema());
}

PoseGraphOption LoadPoseGraphOptionFromFile(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  for (const std::string& key : FindUnknownLifelongKeys(Child(root, "lifelong"))) {
    LOG(WARNING) << "unknown option in " << path << ", ignored: lifelong/" << key;
  }
  for (const std::string& key :
       FindUnknownGlobalScanMatcherKeys(Child(root, "global_scan_matcher"))) {
    LOG(WARNING) << "unknown option in " << path << ", ignored: global_scan_matcher/" << key;
  }
  return LoadPoseGraphOption(root);
}

}  // namespace evergreenslam::lifelong
