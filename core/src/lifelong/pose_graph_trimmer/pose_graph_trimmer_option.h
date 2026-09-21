/**
 * @file pose_graph_trimmer_option.h
 * @author hang chen (chen@hang.plus)
 * @brief Options for the trim selector and the trimmer pipeline.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_OPTION_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::lifelong {

enum class CoverageMode { kAny, kAll };

inline const char* ToString(CoverageMode mode) {
  return mode == CoverageMode::kAll ? "all" : "any";
}

struct TrimSelectorOption {
  double submap_coverage_resolution = 0.15;
  double node_coverage_resolution = 0.3;
  double min_covered_fraction = 0.9;
  int min_covering_newer_submaps = 2;
  // A submap without nodes never satisfies this under kAny, so emptied submaps cannot cascade.
  double min_covered_node_fraction = 0.8;
  CoverageMode coverage_mode = CoverageMode::kAny;
  // Freezing is gated on minted submaps, not survivors, so this floor is free to be small.
  int min_surviving_submaps = 2;
  // Finished submaps per session that neither get trimmed nor count as coverers. A node is matched
  // against the submaps around it; if those just finished could cover an older one, it could be
  // deleted while the match result is still on a worker.
  int keep_newest_submaps = 2;

  friend std::ostream& operator<<(std::ostream& os, const TrimSelectorOption& option) {
    os << "TrimSelectorOption:" << std::endl;
    os << "  submap_coverage_resolution: " << option.submap_coverage_resolution << std::endl;
    os << "  node_coverage_resolution: " << option.node_coverage_resolution << std::endl;
    os << "  min_covered_fraction: " << option.min_covered_fraction << std::endl;
    os << "  min_covering_newer_submaps: " << option.min_covering_newer_submaps << std::endl;
    os << "  min_covered_node_fraction: " << option.min_covered_node_fraction << std::endl;
    os << "  coverage_mode: " << ToString(option.coverage_mode) << std::endl;
    os << "  min_surviving_submaps: " << option.min_surviving_submaps << std::endl;
    os << "  keep_newest_submaps: " << option.keep_newest_submaps << std::endl;
    return os;
  }
};

TrimSelectorOption LoadTrimSelectorOption(const YAML::Node& node);

struct PoseGraphTrimmerOption {
  TrimSelectorOption selector;
  int max_submaps_per_round = 2;
  // A summary may not claim more certainty than the measurements it replaces.
  double min_recovered_stddev = 1e-2;
  // Gated on freeze-grade marginals, because an absolute prior resists future loop corrections.
  double max_binary_constraint_length = 10.0;
  double prior_max_translation_stddev = 0.02;
  double prior_max_rotation_stddev = 0.005;

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphTrimmerOption& option) {
    os << "PoseGraphTrimmerOption:" << std::endl;
    os << "  max_submaps_per_round: " << option.max_submaps_per_round << std::endl;
    os << "  min_recovered_stddev: " << option.min_recovered_stddev << std::endl;
    os << "  max_binary_constraint_length: " << option.max_binary_constraint_length << std::endl;
    os << "  prior_max_translation_stddev: " << option.prior_max_translation_stddev << std::endl;
    os << "  prior_max_rotation_stddev: " << option.prior_max_rotation_stddev << std::endl;
    os << option.selector;
    return os;
  }
};

PoseGraphTrimmerOption LoadPoseGraphTrimmerOption(const YAML::Node& node);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_POSE_GRAPH_TRIMMER_OPTION_H_
