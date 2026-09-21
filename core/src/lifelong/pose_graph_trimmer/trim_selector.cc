/**
 * @file trim_selector.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/trim_selector.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "lifelong/coarse_footprint.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

double FarthestKnownCellDistance(const mapping::ProbabilityGrid& grid) {
  const Eigen::AlignedBox2i& known = grid.known_area();
  if (known.isEmpty()) {
    return 0.0;
  }
  double radius = 0.0;
  for (const int x : {known.min().x(), known.max().x()}) {
    for (const int y : {known.min().y(), known.max().y()}) {
      radius = std::max(radius, grid.ToCenter(Eigen::Array2i(x, y)).norm());
    }
  }
  return radius;
}

}  // namespace

TrimSelector::TrimSelector(const TrimSelectorOption& option) : option_(option) {
  CHECK_GT(option_.submap_coverage_resolution, 0.0);
  CHECK_GT(option_.node_coverage_resolution, 0.0);
  CHECK_GE(option_.min_surviving_submaps, 0);
}

const std::vector<std::int64_t>& TrimSelector::Footprint(const SubmapRecord& record) const {
  const auto it = footprint_cache_.find(record.id);
  if (it != footprint_cache_.end()) {
    const CachedFootprint& cached = it->second;
    const double rotation = std::abs(utils::transform::NormalizeAngle(
        utils::transform::GetYaw(record.global_pose) - utils::transform::GetYaw(cached.pose)));
    const double shift = (cached.pose.translation() - record.global_pose.translation()).norm() +
                         rotation * cached.radius;
    if (shift < 0.5 * option_.submap_coverage_resolution) {
      return cached.cells;
    }
  }
  CachedFootprint& entry = footprint_cache_[record.id];
  entry.pose = record.global_pose;
  entry.radius = FarthestKnownCellDistance(record.submap->grid());
  entry.cells = ComputeCoarseFootprint(record, option_.submap_coverage_resolution);
  return entry.cells;
}

std::vector<SubmapId> TrimSelector::Select(const PoseGraphData& graph) const {
  // Ids are never reused, so entries for trimmed submaps would otherwise accumulate forever.
  for (auto it = footprint_cache_.begin(); it != footprint_cache_.end();) {
    it = graph.HasSubmap(it->first) ? std::next(it) : footprint_cache_.erase(it);
  }

  std::vector<SubmapId> trimmable = graph.TrimmableSubmapIds();
  std::sort(trimmable.begin(), trimmable.end());

  // Read once: finished() flips on the frontend thread, and the loops below must agree on it.
  std::set<SubmapId> finished;
  for (const auto& [id, record] : graph.submaps()) {
    if (record.submap != nullptr && record.submap->finished()) {
      finished.insert(id);
    }
  }

  std::unordered_set<int> frozen_sessions;
  std::set<SubmapId> newest;
  for (const auto& [session_id, session] : graph.sessions()) {
    if (session.frozen()) {
      frozen_sessions.insert(session_id.session_index);
      continue;
    }
    int kept = 0;
    for (auto it = session.submap_ids.rbegin();
         it != session.submap_ids.rend() && kept < option_.keep_newest_submaps; ++it) {
      if (finished.count(*it) != 0) {
        newest.insert(*it);
        ++kept;
      }
    }
  }

  // A frozen session can carry a higher index than a floating one, so its submaps would count as
  // "newer". Unfinished grids belong to the frontend thread, so they are excluded as coverers.
  std::vector<SubmapId> all_ids;
  for (const SubmapId& id : finished) {
    if (frozen_sessions.count(id.session_id) != 0 || newest.count(id) != 0) {
      continue;
    }
    all_ids.push_back(id);
  }
  std::sort(all_ids.begin(), all_ids.end());
  std::map<SubmapId, int> ordinal_of;
  for (size_t i = 0; i < all_ids.size(); ++i) {
    ordinal_of.emplace(all_ids[i], static_cast<int>(i));
  }

  std::unordered_map<std::int64_t, std::vector<int>> submap_cover;
  for (const SubmapId& id : all_ids) {
    const int ordinal = ordinal_of.at(id);
    for (const std::int64_t cell : Footprint(graph.submap(id))) {
      submap_cover[cell].push_back(ordinal);  // ascending: ids are visited in order
    }
  }

  // Only a coverer's nodes cover, for the same reason its area does.
  std::set<NodeId> covering_nodes;
  for (const SubmapId& id : all_ids) {
    const std::set<NodeId>& node_ids = graph.submap(id).node_ids;
    covering_nodes.insert(node_ids.begin(), node_ids.end());
  }
  std::unordered_map<std::int64_t, std::vector<NodeId>> node_cover;
  for (const NodeId& id : covering_nodes) {
    node_cover[CoarseCellOf(graph.node(id).global_pose.translation(),
                            option_.node_coverage_resolution)]
        .push_back(id);  // ascending: a set iterates in id order
  }

  std::map<SessionId, std::vector<SubmapId>> selected_by_session;
  for (const SubmapId& id : trimmable) {
    if (finished.count(id) == 0 || newest.count(id) != 0) {
      continue;
    }
    const SubmapRecord& record = graph.submap(id);

    const std::vector<std::int64_t>& footprint = Footprint(record);
    bool submap_covered = false;
    if (!footprint.empty()) {
      const int ordinal = ordinal_of.at(id);
      int covered_cells = 0;
      for (const std::int64_t cell : footprint) {
        const std::vector<int>& covering = submap_cover.at(cell);
        const int newer = static_cast<int>(
            covering.end() - std::upper_bound(covering.begin(), covering.end(), ordinal));
        if (newer >= option_.min_covering_newer_submaps) {
          ++covered_cells;
        }
      }
      submap_covered = static_cast<double>(covered_cells) >=
                       option_.min_covered_fraction * static_cast<double>(footprint.size());
    }

    bool node_covered = false;
    if (!record.node_ids.empty()) {
      int covered_nodes = 0;
      for (const NodeId& node_id : record.node_ids) {
        const std::int64_t cell = CoarseCellOf(graph.node(node_id).global_pose.translation(),
                                               option_.node_coverage_resolution);
        const std::vector<NodeId>& residents = node_cover.at(cell);
        // Filtered here, not out of node_cover: a transferred submap keeps node_ids left behind
        // in a now-frozen session, and node_cover.at() above would miss their cells.
        const bool covered =
            std::any_of(std::upper_bound(residents.begin(), residents.end(), node_id),
                        residents.end(), [&record, &frozen_sessions](const NodeId& newer) {
                          return record.node_ids.count(newer) == 0 &&
                                 frozen_sessions.count(newer.session_id) == 0;
                        });
        if (covered) {
          ++covered_nodes;
        }
      }
      node_covered =
          static_cast<double>(covered_nodes) >=
          option_.min_covered_node_fraction * static_cast<double>(record.node_ids.size());
    }

    const bool redundant = option_.coverage_mode == CoverageMode::kAll
                               ? submap_covered && (record.node_ids.empty() || node_covered)
                               : submap_covered || node_covered;
    if (!redundant) {
      continue;
    }

    selected_by_session[SessionOf(id)].push_back(id);
  }

  std::vector<SubmapId> selected;
  for (auto& [session_id, candidates] : selected_by_session) {
    const int total = static_cast<int>(graph.session(session_id).submap_ids.size());
    const int allowed = std::max(0, total - option_.min_surviving_submaps);
    const int take = std::min(allowed, static_cast<int>(candidates.size()));
    selected.insert(selected.end(), candidates.begin(), candidates.begin() + take);
  }
  std::sort(selected.begin(), selected.end());
  return selected;
}

}  // namespace evergreenslam::lifelong
