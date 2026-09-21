/**
 * @file pose_graph_option.h
 * @author hang chen (chen@hang.plus)
 * @brief Every backend module's options in one place, loaded from one config file.
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_OPTION_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/constraints/constraint_builder_option.h"
#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer_option.h"
#include "lifelong/sessions/session_manager.h"
#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

struct PoseGraphOption {
  OptimizationOption optimization;
  ConstraintWeightOption constraint_weight;
  SessionManagerOption session_manager;
  ConstraintBuilderOption constraint_builder;
  PoseGraphTrimmerOption trimmer;
  bool trim = true;
  int trim_every_n_optimizations = 1;
  // Sensor time; Finish, freeze and rotation write regardless.
  common::Duration checkpoint_min_interval = common::FromSeconds(30.0);
  int gc_unfrozen_after_n_boots = 5;

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphOption& option) {
    os << "PoseGraphOption:" << std::endl;
    os << "  trim: " << option.trim << std::endl;
    os << "  trim_every_n_optimizations: " << option.trim_every_n_optimizations << std::endl;
    os << "  checkpoint_min_interval: " << common::ToSeconds(option.checkpoint_min_interval)
       << std::endl;
    os << "  gc_unfrozen_after_n_boots: " << option.gc_unfrozen_after_n_boots << std::endl;
    os << option.optimization;
    os << option.constraint_weight;
    os << option.session_manager;
    os << option.constraint_builder;
    os << option.trimmer;
    return os;
  }
};

PoseGraphOption LoadPoseGraphOption(const YAML::Node& root);
// Exposed so a test can check that configs/reference.yaml lists every key.
const utils::config::Schema& LifelongSchema();
std::vector<std::string> FindUnknownLifelongKeys(const YAML::Node& node);
// The loop matcher reads the top-level global_scan_matcher section, so it is validated here.
const utils::config::Schema& GlobalScanMatcherSchema();
std::vector<std::string> FindUnknownGlobalScanMatcherKeys(const YAML::Node& node);
PoseGraphOption LoadPoseGraphOptionFromFile(const std::string& path);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_OPTION_H_
