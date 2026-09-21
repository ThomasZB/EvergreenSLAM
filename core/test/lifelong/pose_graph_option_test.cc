/**
 * @file pose_graph_option_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Config loading for the backend.
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_option.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "common/time.h"

namespace evergreenslam::lifelong {
namespace {

TEST(PoseGraphOptionTest, LoadsEveryModuleFromYaml) {
  const YAML::Node root = YAML::Load(R"(
global_scan_matcher:
  fast_correlative:
    angular_search_window: 0.35
    branch_and_bound_depth: 5
lifelong:
  optimization:
    optimize_every_n_nodes: 17
  constraint_weight:
    odometry_translation_stddev: 0.005
    loop_closure_huber_distance: 0.25
  session_manager:
    min_new_area_fraction: 0.25
    growth_tolerance: 7.5
    track_growth_tolerance: 2.5
    submaps_without_growth: 3
    track_resolution: 0.6
    freeze_judge:
      max_translation_stddev: 0.033
  constraint_builder:
    candidate_slack_per_meter: 0.02
    search_window_floor: 0.8
    tight_window_max: 1.5
    tight_gates:
      max_free_space_fraction: 0.35
    wide_gates:
      min_refined_score: 0.7
    min_loop_node_gap: 7
    num_match_workers: 3
    constraint_sampler:
      max_matches_per_round: 5
  trimmer:
    max_submaps_per_round: 4
    selector:
      min_surviving_submaps: 9
  trim: false
  checkpoint_min_interval: 2.5
)");
  const PoseGraphOption option = LoadPoseGraphOption(root);
  EXPECT_EQ(option.optimization.optimize_every_n_nodes, 17);
  EXPECT_DOUBLE_EQ(option.constraint_weight.odometry_translation_stddev, 0.005);
  EXPECT_DOUBLE_EQ(option.constraint_weight.loop_closure_huber_distance, 0.25);
  EXPECT_DOUBLE_EQ(option.session_manager.min_new_area_fraction, 0.25);
  EXPECT_DOUBLE_EQ(option.session_manager.growth_tolerance, 7.5);
  EXPECT_DOUBLE_EQ(option.session_manager.track_growth_tolerance, 2.5);
  EXPECT_EQ(option.session_manager.submaps_without_growth, 3);
  EXPECT_DOUBLE_EQ(option.session_manager.track_resolution, 0.6);
  EXPECT_DOUBLE_EQ(option.session_manager.freeze_judge.max_translation_stddev, 0.033);
  EXPECT_DOUBLE_EQ(option.constraint_builder.candidate_slack_per_meter, 0.02);
  EXPECT_DOUBLE_EQ(option.constraint_builder.search_window_floor, 0.8);
  EXPECT_DOUBLE_EQ(option.constraint_builder.tight_window_max, 1.5);
  EXPECT_DOUBLE_EQ(option.constraint_builder.tight_gates.max_free_space_fraction, 0.35);
  EXPECT_DOUBLE_EQ(option.constraint_builder.tight_gates.min_coarse_score,
                   PoseGraphOption().constraint_builder.tight_gates.min_coarse_score);
  EXPECT_DOUBLE_EQ(option.constraint_builder.wide_gates.min_refined_score, 0.7);
  EXPECT_EQ(option.constraint_builder.min_loop_node_gap, 7);
  EXPECT_EQ(option.constraint_builder.sampler_option.max_matches_per_round, 5);
  // The loop matcher comes from the shared global_scan_matcher section, not from lifelong.
  EXPECT_DOUBLE_EQ(option.constraint_builder.matcher_option.angular_search_window, 0.35);
  EXPECT_EQ(option.constraint_builder.matcher_option.branch_and_bound_depth, 5);
  EXPECT_EQ(option.trimmer.max_submaps_per_round, 4);
  EXPECT_EQ(option.trimmer.selector.min_surviving_submaps, 9);
  EXPECT_EQ(option.constraint_builder.num_match_workers, 3);
  EXPECT_FALSE(option.trim);
  EXPECT_EQ(option.checkpoint_min_interval, common::FromSeconds(2.5));
}

TEST(PoseGraphOptionTest, MissingSectionsKeepDefaults) {
  const PoseGraphOption loaded = LoadPoseGraphOption(YAML::Load("{}"));
  const PoseGraphOption defaults;
  EXPECT_EQ(loaded.optimization.optimize_every_n_nodes,
            defaults.optimization.optimize_every_n_nodes);
  EXPECT_DOUBLE_EQ(loaded.session_manager.freeze_judge.max_rotation_stddev,
                   defaults.session_manager.freeze_judge.max_rotation_stddev);
  EXPECT_DOUBLE_EQ(loaded.constraint_builder.min_score_prominence,
                   defaults.constraint_builder.min_score_prominence);
  EXPECT_EQ(loaded.trimmer.selector.keep_newest_submaps,
            defaults.trimmer.selector.keep_newest_submaps);
  EXPECT_DOUBLE_EQ(loaded.constraint_weight.loop_closure_huber_distance,
                   defaults.constraint_weight.loop_closure_huber_distance);
  EXPECT_EQ(loaded.trim, defaults.trim);
  EXPECT_EQ(loaded.checkpoint_min_interval, defaults.checkpoint_min_interval);
}

TEST(PoseGraphOptionTest, UnknownKeysAreReported) {
  const YAML::Node node = YAML::Load(R"(
optimization:
  optimize_every_n_node: 17
trimmer:
  selector:
    keep_newest_submap: 1
constraint_builder:
  wide_gates:
    min_refined_scor: 0.6
trimm: true
)");
  const std::vector<std::string> unknown = FindUnknownLifelongKeys(node);
  ASSERT_EQ(unknown.size(), 4u);
  EXPECT_NE(
      std::find(unknown.begin(), unknown.end(), "constraint_builder/wide_gates/min_refined_scor"),
      unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "optimization/optimize_every_n_node"),
            unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "trimmer/selector/keep_newest_submap"),
            unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "trimm"), unknown.end());
}

void ExpectSameGates(const AcceptanceGates& gates, const AcceptanceGates& expected) {
  EXPECT_DOUBLE_EQ(gates.min_coarse_score, expected.min_coarse_score);
  EXPECT_DOUBLE_EQ(gates.min_refined_score, expected.min_refined_score);
  EXPECT_DOUBLE_EQ(gates.max_free_space_fraction, expected.max_free_space_fraction);
}

void ExpectMatchesDefaults(const PoseGraphOption& option) {
  const PoseGraphOption defaults;
  EXPECT_EQ(option.optimization.optimize_every_n_nodes,
            defaults.optimization.optimize_every_n_nodes);
  EXPECT_EQ(option.optimization.max_num_iterations, defaults.optimization.max_num_iterations);
  EXPECT_EQ(option.optimization.num_threads, defaults.optimization.num_threads);
  EXPECT_DOUBLE_EQ(option.constraint_weight.odometry_translation_stddev,
                   defaults.constraint_weight.odometry_translation_stddev);
  EXPECT_DOUBLE_EQ(option.constraint_weight.odometry_rotation_stddev,
                   defaults.constraint_weight.odometry_rotation_stddev);
  EXPECT_DOUBLE_EQ(option.constraint_weight.loop_closure_translation_stddev,
                   defaults.constraint_weight.loop_closure_translation_stddev);
  EXPECT_DOUBLE_EQ(option.constraint_weight.loop_closure_rotation_stddev,
                   defaults.constraint_weight.loop_closure_rotation_stddev);
  EXPECT_DOUBLE_EQ(option.constraint_weight.loop_closure_huber_distance,
                   defaults.constraint_weight.loop_closure_huber_distance);
  EXPECT_EQ(option.session_manager.auto_freeze, defaults.session_manager.auto_freeze);
  EXPECT_DOUBLE_EQ(option.session_manager.min_new_area, defaults.session_manager.min_new_area);
  EXPECT_DOUBLE_EQ(option.session_manager.growth_tolerance,
                   defaults.session_manager.growth_tolerance);
  EXPECT_EQ(option.session_manager.submaps_without_growth,
            defaults.session_manager.submaps_without_growth);
  EXPECT_DOUBLE_EQ(option.session_manager.coverage_resolution,
                   defaults.session_manager.coverage_resolution);
  EXPECT_DOUBLE_EQ(option.session_manager.frozen_covered_fraction,
                   defaults.session_manager.frozen_covered_fraction);
  EXPECT_DOUBLE_EQ(option.session_manager.freeze_judge.max_translation_stddev,
                   defaults.session_manager.freeze_judge.max_translation_stddev);
  EXPECT_DOUBLE_EQ(option.session_manager.freeze_judge.max_rotation_stddev,
                   defaults.session_manager.freeze_judge.max_rotation_stddev);
  EXPECT_DOUBLE_EQ(option.constraint_builder.sampler_option.sampling_ratio,
                   defaults.constraint_builder.sampler_option.sampling_ratio);
  EXPECT_EQ(option.constraint_builder.sampler_option.max_matches_per_round,
            defaults.constraint_builder.sampler_option.max_matches_per_round);
  EXPECT_DOUBLE_EQ(option.constraint_builder.candidate_slack_per_meter,
                   defaults.constraint_builder.candidate_slack_per_meter);
  EXPECT_DOUBLE_EQ(option.constraint_builder.max_candidate_slack,
                   defaults.constraint_builder.max_candidate_slack);
  EXPECT_DOUBLE_EQ(option.constraint_builder.search_window_floor,
                   defaults.constraint_builder.search_window_floor);
  EXPECT_EQ(option.constraint_builder.min_loop_node_gap,
            defaults.constraint_builder.min_loop_node_gap);
  EXPECT_DOUBLE_EQ(option.constraint_builder.relocalization_linear_search_window,
                   defaults.constraint_builder.relocalization_linear_search_window);
  EXPECT_DOUBLE_EQ(option.constraint_builder.relocalization_angular_search_window,
                   defaults.constraint_builder.relocalization_angular_search_window);
  EXPECT_DOUBLE_EQ(option.constraint_builder.tight_window_max,
                   defaults.constraint_builder.tight_window_max);
  ExpectSameGates(option.constraint_builder.tight_gates, defaults.constraint_builder.tight_gates);
  ExpectSameGates(option.constraint_builder.wide_gates, defaults.constraint_builder.wide_gates);
  EXPECT_DOUBLE_EQ(option.constraint_builder.max_refinement_translation,
                   defaults.constraint_builder.max_refinement_translation);
  EXPECT_DOUBLE_EQ(option.constraint_builder.max_refinement_rotation,
                   defaults.constraint_builder.max_refinement_rotation);
  EXPECT_DOUBLE_EQ(option.constraint_builder.prominence_translation,
                   defaults.constraint_builder.prominence_translation);
  EXPECT_DOUBLE_EQ(option.constraint_builder.prominence_rotation,
                   defaults.constraint_builder.prominence_rotation);
  EXPECT_DOUBLE_EQ(option.constraint_builder.min_score_prominence,
                   defaults.constraint_builder.min_score_prominence);
  EXPECT_DOUBLE_EQ(option.constraint_builder.free_space_probability,
                   defaults.constraint_builder.free_space_probability);
  EXPECT_EQ(option.constraint_builder.precomputation_cache_size,
            defaults.constraint_builder.precomputation_cache_size);
  EXPECT_EQ(option.constraint_builder.num_match_workers,
            defaults.constraint_builder.num_match_workers);
  EXPECT_DOUBLE_EQ(option.trimmer.selector.submap_coverage_resolution,
                   defaults.trimmer.selector.submap_coverage_resolution);
  EXPECT_DOUBLE_EQ(option.trimmer.selector.node_coverage_resolution,
                   defaults.trimmer.selector.node_coverage_resolution);
  EXPECT_DOUBLE_EQ(option.trimmer.selector.min_covered_fraction,
                   defaults.trimmer.selector.min_covered_fraction);
  EXPECT_EQ(option.trimmer.selector.min_covering_newer_submaps,
            defaults.trimmer.selector.min_covering_newer_submaps);
  EXPECT_DOUBLE_EQ(option.trimmer.selector.min_covered_node_fraction,
                   defaults.trimmer.selector.min_covered_node_fraction);
  EXPECT_EQ(option.trimmer.selector.coverage_mode, defaults.trimmer.selector.coverage_mode);
  EXPECT_EQ(option.trimmer.selector.min_surviving_submaps,
            defaults.trimmer.selector.min_surviving_submaps);
  EXPECT_EQ(option.trimmer.selector.keep_newest_submaps,
            defaults.trimmer.selector.keep_newest_submaps);
  EXPECT_EQ(option.trimmer.max_submaps_per_round, defaults.trimmer.max_submaps_per_round);
  EXPECT_DOUBLE_EQ(option.trimmer.min_recovered_stddev, defaults.trimmer.min_recovered_stddev);
  EXPECT_DOUBLE_EQ(option.trimmer.max_binary_constraint_length,
                   defaults.trimmer.max_binary_constraint_length);
  EXPECT_DOUBLE_EQ(option.trimmer.prior_max_translation_stddev,
                   defaults.trimmer.prior_max_translation_stddev);
  EXPECT_DOUBLE_EQ(option.trimmer.prior_max_rotation_stddev,
                   defaults.trimmer.prior_max_rotation_stddev);
  EXPECT_EQ(option.trim, defaults.trim);
  EXPECT_EQ(option.trim_every_n_optimizations, defaults.trim_every_n_optimizations);
  EXPECT_EQ(option.checkpoint_min_interval, defaults.checkpoint_min_interval);
  EXPECT_EQ(option.gc_unfrozen_after_n_boots, defaults.gc_unfrozen_after_n_boots);
}

std::string ConfigPath(const std::string& name) {
  return std::string(EVERGREENSLAM_CONFIG_DIR) + "/" + name;
}

std::vector<std::string> UnknownKeysIn(const std::string& name) {
  const YAML::Node root = YAML::LoadFile(ConfigPath(name));
  EXPECT_TRUE(root["lifelong"]) << name;
  EXPECT_TRUE(root["global_scan_matcher"]) << name;
  std::vector<std::string> unknown = FindUnknownLifelongKeys(root["lifelong"]);
  for (const std::string& key : FindUnknownGlobalScanMatcherKeys(root["global_scan_matcher"])) {
    unknown.push_back("global_scan_matcher/" + key);
  }
  return unknown;
}

TEST(PoseGraphOptionTest, UnknownGlobalScanMatcherKeysAreReported) {
  const YAML::Node node = YAML::Load(R"(
fast_correlative:
  angular_search_window: 0.52
  angular_search_windwo: 0.52
fast_correlativ:
  branch_and_bound_depth: 7
)");
  const std::vector<std::string> unknown = FindUnknownGlobalScanMatcherKeys(node);
  ASSERT_EQ(unknown.size(), 2u);
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "fast_correlative/angular_search_windwo"),
            unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "fast_correlativ"), unknown.end());
}

// The two shipped configs carry defaults only, so both must reproduce them and neither may carry
// a key nothing reads.
TEST(PoseGraphOptionTest, ShippedConfigMatchesTheDefaults) {
  const std::vector<std::string> unknown = UnknownKeysIn("evergreenslam.yaml");
  EXPECT_TRUE(unknown.empty()) << "first unknown key: " << (unknown.empty() ? "" : unknown.front());
  ExpectMatchesDefaults(LoadPoseGraphOptionFromFile(ConfigPath("evergreenslam.yaml")));
}

TEST(PoseGraphOptionTest, ReferenceConfigMatchesTheDefaults) {
  const std::vector<std::string> unknown = UnknownKeysIn("reference.yaml");
  EXPECT_TRUE(unknown.empty()) << "first unknown key: " << (unknown.empty() ? "" : unknown.front());
  ExpectMatchesDefaults(LoadPoseGraphOptionFromFile(ConfigPath("reference.yaml")));
}

// Assigning to a YAML::Node writes through to the document, so a section is reached by
// descending, never by reassigning the cursor.
YAML::Node Descend(const YAML::Node& node, const std::vector<std::string>& segments, size_t index) {
  if (index == segments.size() || !node) {
    return node;
  }
  return Descend(node[segments[index]], segments, index + 1);
}

std::vector<std::string> Split(const std::string& path) {
  std::vector<std::string> segments;
  std::stringstream stream(path);
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    segments.push_back(segment);
  }
  return segments;
}

// Equal-to-defaults cannot catch a key the reference file forgot, since a missing key loads the
// default too. The schema is the list of everything the loader reads, so walk it.
void ExpectListsEveryKey(const YAML::Node& top, const std::string& top_name,
                         const utils::config::Schema& schema) {
  ASSERT_TRUE(top) << "reference.yaml is missing section " << top_name;
  for (const auto& [section, keys] : schema) {
    const YAML::Node node = Descend(top, Split(section), 0);
    ASSERT_TRUE(node) << "reference.yaml is missing section " << top_name << "/" << section;
    for (const std::string& key : keys) {
      EXPECT_TRUE(node[key]) << "reference.yaml is missing " << top_name << "/" << section << "/"
                             << key;
    }
  }
}

TEST(PoseGraphOptionTest, ReferenceConfigListsEveryKey) {
  const YAML::Node root = YAML::LoadFile(ConfigPath("reference.yaml"));
  ExpectListsEveryKey(root["lifelong"], "lifelong", LifelongSchema());
  ExpectListsEveryKey(root["global_scan_matcher"], "global_scan_matcher",
                      GlobalScanMatcherSchema());
}

// A recovered constraint that outweighs the raw measurements it summarizes would let trimming
// tighten the graph, so the floor is refused at load rather than silently honoured.
TEST(PoseGraphOptionTest, RecoveredStddevBelowTheOdometryStddevIsFatal) {
  const YAML::Node root = YAML::Load(R"(
lifelong:
  constraint_weight:
    odometry_translation_stddev: 0.05
  trimmer:
    min_recovered_stddev: 0.01
)");
  EXPECT_DEATH(LoadPoseGraphOption(root), "min_recovered_stddev");
}

}  // namespace
}  // namespace evergreenslam::lifelong
