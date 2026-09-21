/**
 * @file frozen_coverage.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_coverage.h"

#include <glog/logging.h>

#include "lifelong/coarse_footprint.h"

namespace evergreenslam::lifelong {

FrozenCoverage::FrozenCoverage(double resolution, double track_resolution)
    : resolution_(resolution), track_resolution_(track_resolution) {
  CHECK_GT(resolution_, 0.0);
  CHECK_GT(track_resolution_, 0.0);
}

void FrozenCoverage::Rebuild(const PoseGraphData& graph) {
  cells_.clear();
  track_.clear();
  for (const auto& [id, session] : graph.sessions()) {
    if (session.frozen()) {
      AddSession(graph, id);
    }
  }
}

void FrozenCoverage::AddSession(const PoseGraphData& graph, SessionId id) {
  CHECK(graph.session(id).frozen()) << "session " << id.session_index << " is not frozen";
  for (const SubmapId& submap_id : graph.session(id).submap_ids) {
    const SubmapRecord& record = graph.submap(submap_id);
    // Graph-only tests carry no grids.
    if (record.submap == nullptr) {
      continue;
    }
    CHECK(record.submap->finished()) << "a frozen session has no unfinished submaps";
    for (const std::int64_t cell : ComputeCoarseFootprint(record, resolution_)) {
      cells_.insert(cell);
    }
  }
  for (const NodeId& node_id : graph.session(id).node_ids) {
    track_.insert(CoarseCellOf(graph.node(node_id).global_pose.translation(), track_resolution_));
  }
}

double FrozenCoverage::area() const {
  return static_cast<double>(cells_.size()) * resolution_ * resolution_;
}

FrozenCoverage::Overlap FrozenCoverage::Query(const SubmapRecord& record) const {
  Overlap overlap;
  for (const std::int64_t cell : ComputeCoarseFootprint(record, resolution_)) {
    ++overlap.cells;
    if (Contains(cell)) {
      ++overlap.covered;
    }
  }
  return overlap;
}

double FrozenCoverage::Overlap::covered_fraction() const {
  return cells == 0 ? 1.0 : static_cast<double>(covered) / static_cast<double>(cells);
}

double FrozenCoverage::Overlap::novel_area(double resolution) const {
  return static_cast<double>(cells - covered) * resolution * resolution;
}

}  // namespace evergreenslam::lifelong
