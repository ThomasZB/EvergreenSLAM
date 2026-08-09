/**
 * @file local_trajectory_builder_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/local_trajectory_builder_option.h"

#include <glog/logging.h>

#include "utils/config/yaml_utils.h"

namespace evergreenslam::mapping {
namespace {

using utils::config::Child;
using utils::config::LoadOption;

// Kept next to the loaders below; adding an option here as well is what keeps
// FindUnknownKeys() from reporting it as a typo.
const utils::config::Schema& LocalTrajectoryBuilderSchema() {
  static const utils::config::Schema schema = {
      {"",
       {"min_range", "max_range", "voxel_size", "adaptive_voxel_filter", "generic_tracking_filter",
        "pose_optimization", "motion_filter", "active_map"}},
      {"adaptive_voxel_filter", {"max_length", "min_num_points"}},
      {"generic_tracking_filter",
       {"trans_alpha", "rot_alpha", "trans_sigma", "rot_sigma", "measurement_translation_sigma",
        "measurement_rotation_sigma"}},
      {"pose_optimization",
       {"linear_search_window", "angular_search_window", "grid_match_weight", "translation_weight",
        "rotation_weight"}},
      {"motion_filter", {"max_time_seconds", "max_distance_meters", "max_angle_radians"}},
      {"active_map", {"resolution", "num_scans_per_submap", "castrays_mapping"}},
      {"active_map/castrays_mapping", {"hit_probability", "miss_probability", "insert_free_space"}},
  };
  return schema;
}

}  // namespace

LocalTrajectoryBuilderOption LoadLocalTrajectoryBuilderOption(const YAML::Node& node) {
  LocalTrajectoryBuilderOption option;
  option.min_range = LoadOption(node, "min_range", option.min_range);
  option.max_range = LoadOption(node, "max_range", option.max_range);
  option.voxel_size = LoadOption(node, "voxel_size", option.voxel_size);
  option.adaptive_voxel_filter_option =
      sensor::LoadAdaptiveVoxelFilterOption(Child(node, "adaptive_voxel_filter"));
  option.tracking_filter_option =
      utils::filters::LoadGenericTrackingFilterOption(Child(node, "generic_tracking_filter"));
  option.pose_optimization_option = LoadPoseOptimizationOption(Child(node, "pose_optimization"));
  option.motion_filter_option =
      utils::filters::LoadMotionFilterOption(Child(node, "motion_filter"));
  option.active_map_option = LoadActiveMapOption(Child(node, "active_map"));
  return option;
}

std::vector<std::string> FindUnknownLocalTrajectoryBuilderKeys(const YAML::Node& node) {
  return utils::config::FindUnknownKeys(node, LocalTrajectoryBuilderSchema());
}

LocalTrajectoryBuilderOption LoadLocalTrajectoryBuilderOptionFromFile(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node node = Child(root, "local_trajectory_builder");
  for (const std::string& key : FindUnknownLocalTrajectoryBuilderKeys(node)) {
    LOG(WARNING) << "unknown option in " << path << ", ignored: local_trajectory_builder/" << key;
  }
  return LoadLocalTrajectoryBuilderOption(node);
}

}  // namespace evergreenslam::mapping
