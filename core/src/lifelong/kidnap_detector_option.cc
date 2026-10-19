/**
 * @file kidnap_detector_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/kidnap_detector_option.h"

#include <glog/logging.h>

#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

KidnapDetectorOption LoadKidnapDetectorOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  KidnapDetectorOption option;
  option.enabled = LoadOption(node, "enabled", option.enabled);
  option.max_match_score = LoadOption(node, "max_match_score", option.max_match_score);
  option.max_base_score = LoadOption(node, "max_base_score", option.max_base_score);
  option.min_base_coverage = LoadOption(node, "min_base_coverage", option.min_base_coverage);
  option.min_distance_without_base_match =
      LoadOption(node, "min_distance_without_base_match", option.min_distance_without_base_match);
  option.min_suspect_keyframes =
      LoadOption(node, "min_suspect_keyframes", option.min_suspect_keyframes);
  option.cooldown_keyframes = LoadOption(node, "cooldown_keyframes", option.cooldown_keyframes);
  option.max_cooldown_keyframes =
      LoadOption(node, "max_cooldown_keyframes", option.max_cooldown_keyframes);
  CHECK_GE(option.min_suspect_keyframes, 1) << "kidnap_detector/min_suspect_keyframes";
  CHECK_GE(option.max_cooldown_keyframes, option.cooldown_keyframes)
      << "kidnap_detector/max_cooldown_keyframes below cooldown_keyframes";
  return option;
}

}  // namespace evergreenslam::lifelong
