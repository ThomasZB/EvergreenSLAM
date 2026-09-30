/**
 * @file anchor_store.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/anchors/anchor_store.h"

#include <glog/logging.h>

#include <utility>

namespace evergreenslam::lifelong {
namespace {

std::optional<Anchor> Bind(const PoseGraphData& graph, const NodeId& node_id, bool keep_scan,
                           const Eigen::Affine2d& node_from_anchor) {
  const std::optional<SubmapId> submap_id = AnchorStore::BindingSubmap(graph, node_id);
  if (!submap_id.has_value()) {
    return std::nullopt;
  }
  const Node& node = graph.node(node_id);
  Anchor anchor;
  anchor.submap_id = *submap_id;
  anchor.submap_from_anchor = Eigen::Affine2d(graph.submap(*submap_id).global_pose.inverse() *
                                              node.global_pose * node_from_anchor);
  anchor.node_id = node_id;
  anchor.saved_at = node.constant_data.time;
  if (keep_scan) {
    anchor.scan = node_from_anchor.matrix().isIdentity(0.0)
                      ? node.constant_data.point_cloud
                      : sensor::TransformPointCloud(node.constant_data.point_cloud,
                                                    node_from_anchor.inverse());
  }
  return anchor;
}

void MarkOrphan(Anchor& anchor, OrphanReason reason) {
  anchor.state = AnchorState::ORPHAN;
  anchor.orphan_reason = reason;
}

}  // namespace

const char* ToString(AnchorState state) {
  switch (state) {
    case AnchorState::BOUND:
      return "bound";
    case AnchorState::REBOUND:
      return "rebound";
    case AnchorState::ORPHAN:
      return "orphan";
  }
  return "unknown";
}

const char* ToString(OrphanReason reason) {
  switch (reason) {
    case OrphanReason::NONE:
      return "none";
    case OrphanReason::TRIMMED_NO_SUCCESSOR:
      return "trimmed_no_successor";
    case OrphanReason::SESSION_REMOVED:
      return "session_removed";
    case OrphanReason::SUBMAP_MISSING:
      return "submap_missing";
  }
  return "unknown";
}

std::optional<SubmapId> AnchorStore::BindingSubmap(const PoseGraphData& graph, const NodeId& node) {
  if (!graph.HasNode(node)) {
    return std::nullopt;
  }
  const std::vector<SubmapId> containing = graph.ContainingSubmapIds(node);
  if (containing.empty()) {
    return std::nullopt;
  }
  return containing.front();
}

std::optional<Anchor> AnchorStore::Save(const PoseGraphData& graph, const NodeId& node,
                                        bool keep_scan, const Eigen::Affine2d& node_from_anchor) {
  std::optional<Anchor> anchor = Bind(graph, node, keep_scan, node_from_anchor);
  if (!anchor.has_value()) {
    return std::nullopt;
  }
  anchor->id = next_id_++;
  anchors_.emplace(anchor->id, *anchor);
  ++revision_;
  return anchor;
}

void AnchorStore::Replace(const Anchor& anchor) {
  const auto it = anchors_.find(anchor.id);
  CHECK(it != anchors_.end()) << "anchor " << anchor.id << " is unknown";
  it->second = anchor;
  ++revision_;
}

void AnchorStore::Erase(AnchorId id) {
  CHECK_EQ(anchors_.erase(id), 1u) << "anchor " << id << " is unknown";
  ++revision_;
}

std::optional<Anchor> AnchorStore::Rebind(const PoseGraphData& graph, AnchorId id,
                                          const NodeId& node, bool keep_scan,
                                          const Eigen::Affine2d& node_from_anchor) {
  const auto it = anchors_.find(id);
  if (it == anchors_.end()) {
    return std::nullopt;
  }
  std::optional<Anchor> anchor = Bind(graph, node, keep_scan, node_from_anchor);
  if (!anchor.has_value()) {
    return std::nullopt;
  }
  anchor->id = id;
  it->second = *anchor;
  ++revision_;
  return anchor;
}

std::optional<Anchor> AnchorStore::Get(AnchorId id) const {
  const auto it = anchors_.find(id);
  if (it == anchors_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<ResolvedAnchor> AnchorStore::Resolve(const PoseGraphData& graph, AnchorId id) const {
  const auto it = anchors_.find(id);
  if (it == anchors_.end()) {
    return std::nullopt;
  }
  const Anchor& anchor = it->second;
  ResolvedAnchor resolved;
  resolved.id = anchor.id;
  resolved.state = anchor.state;
  resolved.orphan_reason = anchor.orphan_reason;
  resolved.submap_id = anchor.submap_id;
  resolved.saved_at = anchor.saved_at;
  if (anchor.state == AnchorState::ORPHAN) {
    return resolved;
  }
  if (!graph.HasSubmap(anchor.submap_id)) {
    LOG(ERROR) << "anchor " << anchor.id << " is live but its submap "
               << anchor.submap_id.session_id << "." << anchor.submap_id.submap_index
               << " is gone: a graph change skipped its hook";
    // Reported as an orphan so a caller never sees a live state without a pose.
    resolved.state = AnchorState::ORPHAN;
    resolved.orphan_reason = OrphanReason::SUBMAP_MISSING;
    return resolved;
  }
  resolved.frozen = graph.session(SessionOf(anchor.submap_id)).frozen();
  resolved.global_pose =
      Eigen::Affine2d(graph.submap(anchor.submap_id).global_pose * anchor.submap_from_anchor);
  return resolved;
}

std::vector<ResolvedAnchor> AnchorStore::ResolveAll(const PoseGraphData& graph) const {
  std::vector<ResolvedAnchor> resolved;
  resolved.reserve(anchors_.size());
  for (const auto& [id, anchor] : anchors_) {
    resolved.push_back(*Resolve(graph, id));
  }
  return resolved;
}

std::vector<Anchor> AnchorStore::All() const {
  std::vector<Anchor> all;
  all.reserve(anchors_.size());
  for (const auto& [id, anchor] : anchors_) {
    all.push_back(anchor);
  }
  return all;
}

void AnchorStore::OnTrim(const TrimReport& report) {
  bool changed = false;
  for (const TrimReport::SubmapSuccession& succession : report.submap_successions) {
    for (auto& [id, anchor] : anchors_) {
      if (anchor.state == AnchorState::ORPHAN ||
          !(anchor.submap_id == succession.deleted_submap_id)) {
        continue;
      }
      changed = true;
      if (!succession.successor_submap_id.has_value()) {
        MarkOrphan(anchor, OrphanReason::TRIMMED_NO_SUCCESSOR);
        continue;
      }
      anchor.submap_id = *succession.successor_submap_id;
      anchor.submap_from_anchor =
          Eigen::Affine2d(succession.successor_from_deleted * anchor.submap_from_anchor);
      anchor.state = AnchorState::REBOUND;
    }
  }
  if (changed) {
    ++revision_;
  }
}

void AnchorStore::OnSubmapsTransferred(const std::map<SubmapId, SubmapId>& old_to_new) {
  bool changed = false;
  for (auto& [id, anchor] : anchors_) {
    if (anchor.state == AnchorState::ORPHAN) {
      continue;
    }
    const auto renamed = old_to_new.find(anchor.submap_id);
    if (renamed == old_to_new.end()) {
      continue;
    }
    anchor.submap_id = renamed->second;
    changed = true;
  }
  if (changed) {
    ++revision_;
  }
}

void AnchorStore::OnSessionRemoved(SessionId session) {
  bool changed = false;
  for (auto& [id, anchor] : anchors_) {
    if (anchor.state == AnchorState::ORPHAN || SessionOf(anchor.submap_id) != session) {
      continue;
    }
    MarkOrphan(anchor, OrphanReason::SESSION_REMOVED);
    changed = true;
  }
  if (changed) {
    ++revision_;
  }
}

void AnchorStore::Reconcile(const PoseGraphData& graph) {
  bool changed = false;
  for (auto& [id, anchor] : anchors_) {
    if (anchor.state == AnchorState::ORPHAN) {
      continue;
    }
    if (!graph.HasSession(SessionOf(anchor.submap_id))) {
      MarkOrphan(anchor, OrphanReason::SESSION_REMOVED);
      changed = true;
    } else if (!graph.HasSubmap(anchor.submap_id)) {
      MarkOrphan(anchor, OrphanReason::SUBMAP_MISSING);
      changed = true;
    }
  }
  if (changed) {
    ++revision_;
  }
}

AnchorTable AnchorStore::Table() const {
  AnchorTable table;
  table.next_id = next_id_;
  table.anchors = All();
  return table;
}

void AnchorStore::Restore(AnchorTable table) {
  CHECK_GE(table.next_id, 1u) << "anchor id 0 is never issued";
  std::map<AnchorId, Anchor> anchors;
  for (Anchor& anchor : table.anchors) {
    CHECK_NE(anchor.id, 0u) << "anchor id 0 is never issued";
    CHECK_LT(anchor.id, table.next_id) << "next_id must exceed every issued id";
    const AnchorId id = anchor.id;
    CHECK(anchors.emplace(id, std::move(anchor)).second) << "anchor " << id << " appears twice";
  }
  anchors_ = std::move(anchors);
  next_id_ = table.next_id;
  revision_ = 0;
}

}  // namespace evergreenslam::lifelong
