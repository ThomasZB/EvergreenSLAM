/**
 * @file pose_graph_data.h
 * @author hang chen (chen@hang.plus)
 * @brief Identifiers, records and constraints the pose graph is made of, and the data container
 * that owns them.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_DATA_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_DATA_H_

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include "common/time.h"
#include "lifelong/ids.h"
#include "lifelong/sessions/session_data.h"
#include "mapping/submap.h"
#include "mapping/trajectory_data.h"

namespace evergreenslam::lifelong {

// Ids are monotone per session and never reused, including after a removal and across a reload.
class IdAllocator {
 public:
  SessionId AllocateSession() { return SessionId{next_session_index_++}; }

  NodeId AllocateNode(SessionId session) {
    Observe(session);
    return NodeId{session.session_index, next_node_index_[session.session_index]++};
  }

  SubmapId AllocateSubmap(SessionId session) {
    Observe(session);
    return SubmapId{session.session_index, next_submap_index_[session.session_index]++};
  }

  void Observe(SessionId id) {
    next_session_index_ = std::max(next_session_index_, id.session_index + 1);
  }

  void Observe(const NodeId& id) {
    Observe(SessionOf(id));
    int& next = next_node_index_[id.session_id];
    next = std::max(next, id.node_index + 1);
  }

  void Observe(const SubmapId& id) {
    Observe(SessionOf(id));
    int& next = next_submap_index_[id.session_id];
    next = std::max(next, id.submap_index + 1);
  }

  // Deserialization must replay these: the surviving records say nothing about trimmed indices.
  void ObserveNextIndices(SessionId session, int next_submap_index, int next_node_index) {
    Observe(session);
    int& next_submap = next_submap_index_[session.session_index];
    next_submap = std::max(next_submap, next_submap_index);
    int& next_node = next_node_index_[session.session_index];
    next_node = std::max(next_node, next_node_index);
  }

  int next_node_index(SessionId session) const {
    const auto it = next_node_index_.find(session.session_index);
    return it == next_node_index_.end() ? 0 : it->second;
  }

  int next_submap_index(SessionId session) const {
    const auto it = next_submap_index_.find(session.session_index);
    return it == next_submap_index_.end() ? 0 : it->second;
  }

 private:
  int next_session_index_ = 0;
  std::map<int, int> next_node_index_;
  std::map<int, int> next_submap_index_;
};

// Trimming marginalizes over nodes and submaps alike, so an endpoint cannot be typed as either.
struct VariableId {
  enum class Kind { NODE, SUBMAP };

  Kind kind = Kind::NODE;
  int session_id = 0;
  int index = 0;

  static VariableId Of(const NodeId& id) { return {Kind::NODE, id.session_id, id.node_index}; }
  static VariableId Of(const SubmapId& id) {
    return {Kind::SUBMAP, id.session_id, id.submap_index};
  }

  NodeId node_id() const {
    CHECK(kind == Kind::NODE);
    return NodeId{session_id, index};
  }
  SubmapId submap_id() const {
    CHECK(kind == Kind::SUBMAP);
    return SubmapId{session_id, index};
  }
  SessionId session() const { return SessionId{session_id}; }

  bool operator==(const VariableId& other) const {
    return kind == other.kind && session_id == other.session_id && index == other.index;
  }
  bool operator<(const VariableId& other) const {
    return std::tie(kind, session_id, index) < std::tie(other.kind, other.session_id, other.index);
  }
};

struct Constraint {
  enum class Type { INTRA_SUBMAP, INTER_SUBMAP, PRIOR };

  Type type = Type::INTRA_SUBMAP;
  VariableId from;
  // Empty for a PRIOR: the residual is unary and `relative_pose` is absolute, not T_from_to.
  std::optional<VariableId> to;
  Eigen::Affine2d relative_pose = Eigen::Affine2d::Identity();
  Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
  // Marginalization summaries: Huber-lossed, their information goes stale as the graph moves.
  bool recovered = false;
};

struct Node {
  NodeId id;
  // Never changes once inserted: serialization and anchor replay read it.
  mapping::TrajectoryNode constant_data;
  Eigen::Affine2d global_pose = Eigen::Affine2d::Identity();
};

struct SubmapRecord {
  SubmapId id;
  std::shared_ptr<const mapping::Submap> submap;
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
  Eigen::Affine2d global_pose = Eigen::Affine2d::Identity();
  std::set<NodeId> node_ids;
};

struct TrimRequest {
  std::vector<SubmapId> deleted_submap_ids;
  std::map<SubmapId, SubmapId> successors;
  std::vector<Constraint> added_constraints;
};

struct TrimReport {
  struct SubmapSuccession {
    SubmapId deleted_submap_id;
    // Empty when the trimmer named no survivor to carry what hung off the deleted submap.
    std::optional<SubmapId> successor_submap_id;
    // Deleted submap in the successor frame: T_rel <- successor_from_deleted * T_rel.
    Eigen::Affine2d successor_from_deleted = Eigen::Affine2d::Identity();
  };

  std::vector<SubmapSuccession> submap_successions;
  std::vector<NodeId> deleted_node_ids;
  std::vector<Constraint> added_constraints;
};

class PoseGraphData {
 public:
  SessionId StartNewSession(common::Time time,
                            const Eigen::Affine2d& local_to_global = Eigen::Affine2d::Identity());
  void FreezeSession(SessionId id);

  void FinishSubmaps(SessionId id);
  void RestoreSession(const SessionData& session, int next_submap_index, int next_node_index);

  NodeId AllocateNodeId(SessionId session);
  SubmapId AllocateSubmapId(SessionId session);

  void AddSubmap(const SubmapRecord& submap);
  void AddNode(const Node& node, const std::vector<SubmapId>& containing_submap_ids);
  void AddNodeMembership(const NodeId& node_id, const SubmapId& submap_id);
  void AddConstraint(const Constraint& constraint);

  // Nodes stay put, so their intra constraints become cross-session edges. The caller must
  // re-key the ingest translation table in the same step.
  void TransferSubmap(const SubmapId& old_id, const SubmapId& new_id);

  void SetNodeGlobalPose(const NodeId& id, const Eigen::Affine2d& global_pose);
  void SetSubmapGlobalPose(const SubmapId& id, const Eigen::Affine2d& global_pose);
  void SetSessionLocalToGlobal(SessionId id, const Eigen::Affine2d& local_to_global);

  bool HasSession(SessionId id) const;
  bool HasNode(const NodeId& id) const;
  bool HasSubmap(const SubmapId& id) const;
  bool HasVariable(const VariableId& id) const;

  const SessionData& session(SessionId id) const;
  const Node& node(const NodeId& id) const;
  const SubmapRecord& submap(const SubmapId& id) const;

  const std::map<SessionId, SessionData>& sessions() const { return sessions_; }
  const std::map<NodeId, Node>& nodes() const { return nodes_; }
  const std::map<SubmapId, SubmapRecord>& submaps() const { return submaps_; }
  const std::vector<Constraint>& constraints() const { return constraints_; }
  // Serialization must read the high water marks from here, not from the surviving records.
  const IdAllocator& id_allocator() const { return id_allocator_; }

  std::vector<SubmapId> ContainingSubmapIds(const NodeId& id) const;
  std::optional<Eigen::Affine2d> ComputeSessionToGlobal(SessionId id) const;

  std::vector<SubmapId> TrimmableSubmapIds() const;
  TrimReport ApplyTrim(const TrimRequest& request);

  void RemoveSession(SessionId id);

 private:
  SessionData& MutableSession(SessionId id);
  void RemoveSubmap(const SubmapId& id);
  void RemoveNode(const NodeId& id);
  void RemoveConstraintsTouching(const std::set<VariableId>& removed);

  IdAllocator id_allocator_;
  std::map<SessionId, SessionData> sessions_;
  std::map<NodeId, Node> nodes_;
  std::map<SubmapId, SubmapRecord> submaps_;
  std::vector<Constraint> constraints_;
  std::map<NodeId, std::set<SubmapId>> node_to_submaps_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_DATA_H_
