/**
 * @file local_trajectory_builder_option_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/local_trajectory_builder_option.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

namespace evergreenslam::mapping {
namespace {

TEST(LocalTrajectoryBuilderOptionTest, OverridesApplyThroughNestedSections) {
  const YAML::Node node = YAML::Load(R"(
min_range: 0.3
adaptive_voxel_filter:
  min_num_points: 120
generic_tracking_filter:
  trans_sigma: 2.5
pose_optimization:
  linear_search_window: 0.25
motion_filter:
  max_distance_meters: 0.5
active_map:
  num_scans_per_submap: 12
  castrays_mapping:
    insert_free_space: false
)");
  const LocalTrajectoryBuilderOption option = LoadLocalTrajectoryBuilderOption(node);

  EXPECT_DOUBLE_EQ(option.min_range, 0.3);
  EXPECT_EQ(option.adaptive_voxel_filter_option.min_num_points, 120);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.trans_sigma, 2.5);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.linear_search_window, 0.25);
  EXPECT_DOUBLE_EQ(option.motion_filter_option.max_distance_meters, 0.5);
  EXPECT_EQ(option.active_map_option.num_scans_per_submap, 12);
  EXPECT_FALSE(option.active_map_option.inserter_option.insert_free_space);
}

TEST(LocalTrajectoryBuilderOptionTest, AbsentKeysAndSectionsKeepDefaults) {
  const LocalTrajectoryBuilderOption defaults;

  const YAML::Node partial = YAML::Load("min_range: 0.3");
  const LocalTrajectoryBuilderOption option = LoadLocalTrajectoryBuilderOption(partial);

  EXPECT_DOUBLE_EQ(option.max_range, defaults.max_range);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.rotation_weight,
                   defaults.pose_optimization_option.rotation_weight);
  EXPECT_EQ(option.active_map_option.num_scans_per_submap,
            defaults.active_map_option.num_scans_per_submap);
  EXPECT_DOUBLE_EQ(option.active_map_option.inserter_option.hit_probability,
                   defaults.active_map_option.inserter_option.hit_probability);

  // An empty node must not throw either; every section is optional.
  const LocalTrajectoryBuilderOption from_empty = LoadLocalTrajectoryBuilderOption(YAML::Node());
  EXPECT_DOUBLE_EQ(from_empty.min_range, defaults.min_range);
}

TEST(LocalTrajectoryBuilderOptionTest, UnknownKeysAreReported) {
  const YAML::Node node = YAML::Load(R"(
min_range: 0.3
min_rnage: 0.4
pose_optimization:
  linear_search_window: 0.25
  linear_search_windwo: 0.35
active_map:
  castrays_mapping:
    hit_probabilty: 0.6
)");
  const std::vector<std::string> unknown = FindUnknownLocalTrajectoryBuilderKeys(node);
  EXPECT_EQ(unknown.size(), 3u);
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "min_rnage"), unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "pose_optimization/linear_search_windwo"),
            unknown.end());
  EXPECT_NE(std::find(unknown.begin(), unknown.end(), "active_map/castrays_mapping/hit_probabilty"),
            unknown.end());
}

// configs/evergreenslam.yaml documents the defaults, so it must reproduce them
// exactly, and must not carry a key nothing reads.
TEST(LocalTrajectoryBuilderOptionTest, ShippedConfigMatchesTheDefaults) {
  const std::string path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml";
  const LocalTrajectoryBuilderOption option = LoadLocalTrajectoryBuilderOptionFromFile(path);
  const LocalTrajectoryBuilderOption defaults;

  const YAML::Node root = YAML::LoadFile(path);
  const std::vector<std::string> unknown =
      FindUnknownLocalTrajectoryBuilderKeys(root["local_trajectory_builder"]);
  EXPECT_TRUE(unknown.empty()) << "first unknown key: " << (unknown.empty() ? "" : unknown.front());

  EXPECT_DOUBLE_EQ(option.min_range, defaults.min_range);
  EXPECT_DOUBLE_EQ(option.max_range, defaults.max_range);
  EXPECT_DOUBLE_EQ(option.voxel_size, defaults.voxel_size);
  EXPECT_DOUBLE_EQ(option.adaptive_voxel_filter_option.max_length,
                   defaults.adaptive_voxel_filter_option.max_length);
  EXPECT_EQ(option.adaptive_voxel_filter_option.min_num_points,
            defaults.adaptive_voxel_filter_option.min_num_points);

  EXPECT_DOUBLE_EQ(option.tracking_filter_option.trans_alpha,
                   defaults.tracking_filter_option.trans_alpha);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.rot_alpha,
                   defaults.tracking_filter_option.rot_alpha);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.trans_sigma,
                   defaults.tracking_filter_option.trans_sigma);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.rot_sigma,
                   defaults.tracking_filter_option.rot_sigma);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.measurement_translation_sigma,
                   defaults.tracking_filter_option.measurement_translation_sigma);
  EXPECT_DOUBLE_EQ(option.tracking_filter_option.measurement_rotation_sigma,
                   defaults.tracking_filter_option.measurement_rotation_sigma);

  EXPECT_DOUBLE_EQ(option.pose_optimization_option.linear_search_window,
                   defaults.pose_optimization_option.linear_search_window);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.angular_search_window,
                   defaults.pose_optimization_option.angular_search_window);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.grid_match_weight,
                   defaults.pose_optimization_option.grid_match_weight);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.translation_weight,
                   defaults.pose_optimization_option.translation_weight);
  EXPECT_DOUBLE_EQ(option.pose_optimization_option.rotation_weight,
                   defaults.pose_optimization_option.rotation_weight);

  EXPECT_DOUBLE_EQ(option.motion_filter_option.max_time_seconds,
                   defaults.motion_filter_option.max_time_seconds);
  EXPECT_DOUBLE_EQ(option.motion_filter_option.max_distance_meters,
                   defaults.motion_filter_option.max_distance_meters);
  EXPECT_DOUBLE_EQ(option.motion_filter_option.max_angle_radians,
                   defaults.motion_filter_option.max_angle_radians);

  EXPECT_DOUBLE_EQ(option.active_map_option.resolution, defaults.active_map_option.resolution);
  EXPECT_EQ(option.active_map_option.num_scans_per_submap,
            defaults.active_map_option.num_scans_per_submap);
  EXPECT_DOUBLE_EQ(option.active_map_option.inserter_option.hit_probability,
                   defaults.active_map_option.inserter_option.hit_probability);
  EXPECT_DOUBLE_EQ(option.active_map_option.inserter_option.miss_probability,
                   defaults.active_map_option.inserter_option.miss_probability);
  EXPECT_EQ(option.active_map_option.inserter_option.insert_free_space,
            defaults.active_map_option.inserter_option.insert_free_space);
}

}  // namespace
}  // namespace evergreenslam::mapping
