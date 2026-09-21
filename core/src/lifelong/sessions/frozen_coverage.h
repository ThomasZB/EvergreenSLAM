/**
 * @file frozen_coverage.h
 * @author hang chen (chen@hang.plus)
 * @brief The coarse cell sets of the frozen layer, area and track: what a new session's submaps
 * and nodes are compared against.
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_COVERAGE_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_COVERAGE_H_

#include <cstddef>
#include <cstdint>
#include <unordered_set>

#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

class FrozenCoverage {
 public:
  FrozenCoverage(double resolution, double track_resolution);

  void Rebuild(const PoseGraphData& graph);
  void AddSession(const PoseGraphData& graph, SessionId id);

  struct Overlap {
    int cells = 0;
    int covered = 0;

    // An empty footprint has nothing novel.
    double covered_fraction() const;
    double novel_area(double resolution) const;
  };

  Overlap Query(const SubmapRecord& record) const;
  bool Contains(std::int64_t cell) const { return cells_.count(cell) != 0; }
  bool ContainsTrack(std::int64_t cell) const { return track_.count(cell) != 0; }

  double resolution() const { return resolution_; }
  double track_resolution() const { return track_resolution_; }
  size_t size() const { return cells_.size(); }
  size_t track_size() const { return track_.size(); }
  double area() const;

 private:
  double resolution_;
  double track_resolution_;
  std::unordered_set<std::int64_t> cells_;
  // Cells the frozen sessions' nodes stand in.
  std::unordered_set<std::int64_t> track_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_COVERAGE_H_
