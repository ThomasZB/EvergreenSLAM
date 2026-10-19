/**
 * @file kidnap_detector_option.h
 * @author hang chen (chen@hang.plus)
 * @brief When the backend decides the robot was carried off and searches the whole base for it.
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_OPTION_H_
#define EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::lifelong {

struct KidnapDetectorOption {
  bool enabled = true;
  // A keyframe the frontend matched at or below this breaks continuity until the next closure
  // onto the frozen base. Correct tracking on the museum_b2 replay: 5th percentile 0.58.
  double max_match_score = 0.45;
  // The newest keyframe disagrees with the frozen base: score below this over at least
  // min_base_coverage of its points. Correct poses at coverage >= 0.5: 5th percentile 0.41.
  double max_base_score = 0.4;
  double min_base_coverage = 0.5;
  // Driven since the last closure onto the frozen base, or since the session began without one.
  double min_distance_without_base_match = 3.0;
  // Consecutive keyframes all three must hold for.
  int min_suspect_keyframes = 8;
  // Keyframes after a search settles before the next may start; doubled after each no_match up
  // to max_cooldown_keyframes, back to cooldown_keyframes after a match.
  int cooldown_keyframes = 50;
  int max_cooldown_keyframes = 800;

  friend std::ostream& operator<<(std::ostream& os, const KidnapDetectorOption& option) {
    os << "KidnapDetectorOption:" << std::endl;
    os << "  enabled: " << option.enabled << std::endl;
    os << "  max_match_score: " << option.max_match_score << std::endl;
    os << "  max_base_score: " << option.max_base_score << std::endl;
    os << "  min_base_coverage: " << option.min_base_coverage << std::endl;
    os << "  min_distance_without_base_match: " << option.min_distance_without_base_match
       << std::endl;
    os << "  min_suspect_keyframes: " << option.min_suspect_keyframes << std::endl;
    os << "  cooldown_keyframes: " << option.cooldown_keyframes << std::endl;
    os << "  max_cooldown_keyframes: " << option.max_cooldown_keyframes << std::endl;
    return os;
  }
};

KidnapDetectorOption LoadKidnapDetectorOption(const YAML::Node& node);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_OPTION_H_
