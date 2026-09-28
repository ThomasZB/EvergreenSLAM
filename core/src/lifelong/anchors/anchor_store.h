/**
 * @file anchor_store.h
 * @author hang chen (chen@hang.plus)
 * @brief Anchors: remembered robot poses bound to a submap, rebound as trim and rotation rename or
 * delete what they hang off.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_ANCHORS_ANCHOR_STORE_H_
#define EVERGREENSLAM_LIFELONG_ANCHORS_ANCHOR_STORE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "common/time.h"
#include "lifelong/ids.h"
#include "lifelong/pose_graph_data.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::lifelong {

using AnchorId = uint64_t;
enum class AnchorState { BOUND, REBOUND, ORPHAN };
enum class OrphanReason { NONE, TRIMMED_NO_SUCCESSOR, SESSION_REMOVED, SUBMAP_MISSING };

const char* ToString(AnchorState state);
const char* ToString(OrphanReason reason);

struct Anchor {
  AnchorId id = 0;
  SubmapId submap_id;
  // T_rel = graph.submap(s).global_pose.inverse() * graph.node(n).global_pose, optimized poses.
  Eigen::Affine2d submap_from_anchor = Eigen::Affine2d::Identity();
  // Evidence only, never used to resolve: nodes are trimmed while their submap survives.
  NodeId node_id;
  // The saved keyframe's time, not the wall clock at save.
  common::Time saved_at;
  AnchorState state = AnchorState::BOUND;
  OrphanReason orphan_reason = OrphanReason::NONE;
  // The keyframe's cloud in the anchor frame (= the node frame); raw material for a later align.
  std::optional<sensor::PointCloud> scan;
};

struct ResolvedAnchor {
  AnchorId id = 0;
  AnchorState state = AnchorState::BOUND;
  OrphanReason orphan_reason = OrphanReason::NONE;
  // graph.session(SessionOf(submap_id)).frozen() at Resolve time; false when ORPHAN.
  bool frozen = false;
  SubmapId submap_id;
  common::Time saved_at;
  // Empty exactly when ORPHAN.
  std::optional<Eigen::Affine2d> global_pose;
};

struct AnchorTable {
  AnchorId next_id = 1;
  std::vector<Anchor> anchors;
};

// Backend-task state: every method runs on the PoseGraph task queue; no I/O, no names. Where
// PoseGraph calls each hook and when MapManager persists the table: adapters/agent/API.md,
// "Anchor hooks".
class AnchorStore {
 public:
  AnchorStore() = default;

  std::optional<Anchor> Save(const PoseGraphData& graph, const NodeId& node, bool keep_scan);
  std::optional<Anchor> Rebind(const PoseGraphData& graph, AnchorId id, const NodeId& node,
                               bool keep_scan);

  // Rollback of a Save or Rebind that never reached disk. Both bump revision(); next_id stays a
  // high-water mark, so an erased id is never issued again.
  void Replace(const Anchor& anchor);
  void Erase(AnchorId id);

  std::optional<Anchor> Get(AnchorId id) const;
  // Empty for an unknown id. A live anchor whose submap is gone breaks the hooks' invariant:
  // logged and resolved without a pose, not CHECKed, since the caller is a server.
  std::optional<ResolvedAnchor> Resolve(const PoseGraphData& graph, AnchorId id) const;
  std::vector<ResolvedAnchor> ResolveAll(const PoseGraphData& graph) const;
  std::vector<Anchor> All() const;

  // T_rel <- successor_from_deleted * T_rel and REBOUND; no successor ->
  // ORPHAN(TRIMMED_NO_SUCCESSOR).
  void OnTrim(const TrimReport& report);
  // Rename only: TransferSubmap keeps the submap's poses, so T_rel and state are unchanged.
  void OnSubmapsTransferred(const std::map<SubmapId, SubmapId>& old_to_new);
  // Every non-orphan anchor bound in `id` becomes ORPHAN(SESSION_REMOVED).
  void OnSessionRemoved(SessionId id);
  // Load-time only. Missing submap -> ORPHAN(SUBMAP_MISSING), or SESSION_REMOVED when the whole
  // session is gone. ORPHAN is final except through Rebind.
  void Reconcile(const PoseGraphData& graph);

  AnchorTable Table() const;
  // Resets revision() to 0, the baseline for "unchanged since read". CHECKs next_id > every id.
  void Restore(AnchorTable table);

  // Bumped by every change, so an unchanged table is not rewritten on each checkpoint.
  int64_t revision() const { return revision_; }
  int size() const { return static_cast<int>(anchors_.size()); }

 private:
  AnchorId next_id_ = 1;
  std::map<AnchorId, Anchor> anchors_;
  int64_t revision_ = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_ANCHORS_ANCHOR_STORE_H_
