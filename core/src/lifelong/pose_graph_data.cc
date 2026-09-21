/**
 * @file pose_graph_data.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_data.h"

#include <glog/logging.h>

#include <algorithm>
#include <memory>

namespace evergreenslam::lifelong {

SessionId PoseGraphData::StartNewSession(common::Time time,
                                         const Eigen::Affine2d& local_to_global) {
  SessionData session;
  session.id = id_allocator_.AllocateSession();
  session.state = SessionState::ACTIVE;
  session.start_time = time;
  session.last_node_time = time;
  session.local_to_global = local_to_global;
  sessions_.emplace(session.id, session);
  return session.id;
}

void PoseGraphData::RestoreSession(const SessionData& session, int next_submap_index,
                                   int next_node_index) {
  CHECK(!HasSession(session.id)) << "session " << session.id.session_index << " is already here";
  id_allocator_.ObserveNextIndices(session.id, next_submap_index, next_node_index);
  SessionData restored = session;
  // AddSubmap/AddNode refuse a frozen session, so the session is filled before it is frozen.
  restored.state = SessionState::ACTIVE;
  restored.submap_ids.clear();
  restored.node_ids.clear();
  sessions_.emplace(restored.id, restored);
}

void PoseGraphData::FreezeSession(SessionId id) {
  FinishSubmaps(id);
  MutableSession(id).state = SessionState::FROZEN;
}

void PoseGraphData::FinishSubmaps(SessionId id) {
  for (const SubmapId& submap_id : MutableSession(id).submap_ids) {
    SubmapRecord& record = submaps_.at(submap_id);
    // Graph-only tests carry no grids.
    if (record.submap == nullptr || record.submap->finished()) {
      continue;
    }
    // A copy, not a const_cast: the frontend owns this object and may still be inserting.
    record.submap = record.submap->FinishedCopy();
  }
}

NodeId PoseGraphData::AllocateNodeId(SessionId session) {
  return id_allocator_.AllocateNode(session);
}

SubmapId PoseGraphData::AllocateSubmapId(SessionId session) {
  return id_allocator_.AllocateSubmap(session);
}

void PoseGraphData::AddSubmap(const SubmapRecord& submap) {
  CHECK(!HasSubmap(submap.id)) << "submap ids are never reused";
  SessionData& session = MutableSession(SessionOf(submap.id));
  CHECK(!session.frozen()) << "a frozen session never grows";
  id_allocator_.Observe(submap.id);
  session.submap_ids.push_back(submap.id);
  submaps_.emplace(submap.id, submap);
}

void PoseGraphData::AddNode(const Node& node, const std::vector<SubmapId>& containing_submap_ids) {
  CHECK(!HasNode(node.id)) << "node ids are never reused";
  SessionData& session = MutableSession(SessionOf(node.id));
  CHECK(!session.frozen()) << "a frozen session never grows";
  id_allocator_.Observe(node.id);
  session.node_ids.push_back(node.id);
  session.last_node_time = std::max(session.last_node_time, node.constant_data.time);
  nodes_.emplace(node.id, node);

  std::set<SubmapId>& membership = node_to_submaps_[node.id];
  for (const SubmapId& submap_id : containing_submap_ids) {
    CHECK(HasSubmap(submap_id));
    membership.insert(submap_id);
    submaps_.at(submap_id).node_ids.insert(node.id);
  }
}

void PoseGraphData::AddNodeMembership(const NodeId& node_id, const SubmapId& submap_id) {
  CHECK(HasNode(node_id));
  CHECK(HasSubmap(submap_id));
  node_to_submaps_[node_id].insert(submap_id);
  submaps_.at(submap_id).node_ids.insert(node_id);
}

void PoseGraphData::TransferSubmap(const SubmapId& old_id, const SubmapId& new_id) {
  CHECK(HasSubmap(old_id));
  CHECK(!HasSubmap(new_id)) << "submap ids are never reused";
  CHECK(!session(SessionOf(old_id)).frozen()) << "a frozen session's submaps are immutable";
  SessionData& to_session = MutableSession(SessionOf(new_id));
  CHECK(!to_session.frozen()) << "a frozen session never grows";

  auto record = submaps_.extract(old_id);
  record.key() = new_id;
  record.mapped().id = new_id;
  const SubmapRecord& transferred = submaps_.insert(std::move(record)).position->second;
  id_allocator_.Observe(new_id);

  std::vector<SubmapId>& from_ids = MutableSession(SessionOf(old_id)).submap_ids;
  from_ids.erase(std::remove(from_ids.begin(), from_ids.end(), old_id), from_ids.end());
  to_session.submap_ids.push_back(new_id);

  for (const NodeId& node_id : transferred.node_ids) {
    std::set<SubmapId>& membership = node_to_submaps_.at(node_id);
    membership.erase(old_id);
    membership.insert(new_id);
  }

  const VariableId old_variable = VariableId::Of(old_id);
  const VariableId new_variable = VariableId::Of(new_id);
  for (Constraint& constraint : constraints_) {
    if (constraint.from == old_variable) {
      constraint.from = new_variable;
    }
    if (constraint.to.has_value() && *constraint.to == old_variable) {
      constraint.to = new_variable;
    }
  }
}

void PoseGraphData::AddConstraint(const Constraint& constraint) {
  CHECK_EQ(constraint.type == Constraint::Type::PRIOR, !constraint.to.has_value())
      << "a unary residual is exactly a prior";
  CHECK(HasVariable(constraint.from));
  if (constraint.to.has_value()) {
    CHECK(HasVariable(*constraint.to));
    CHECK(!(*constraint.to == constraint.from));
  }
  constraints_.push_back(constraint);
}

void PoseGraphData::SetNodeGlobalPose(const NodeId& id, const Eigen::Affine2d& global_pose) {
  CHECK(HasNode(id));
  nodes_.at(id).global_pose = global_pose;
}

void PoseGraphData::SetSubmapGlobalPose(const SubmapId& id, const Eigen::Affine2d& global_pose) {
  CHECK(HasSubmap(id));
  submaps_.at(id).global_pose = global_pose;
}

void PoseGraphData::SetSessionLocalToGlobal(SessionId id, const Eigen::Affine2d& local_to_global) {
  SessionData& session = MutableSession(id);
  CHECK(!session.frozen()) << "a frozen session's alignment is truth";
  session.local_to_global = local_to_global;
}

bool PoseGraphData::HasSession(SessionId id) const { return sessions_.count(id) > 0; }
bool PoseGraphData::HasNode(const NodeId& id) const { return nodes_.count(id) > 0; }
bool PoseGraphData::HasSubmap(const SubmapId& id) const { return submaps_.count(id) > 0; }

bool PoseGraphData::HasVariable(const VariableId& id) const {
  return id.kind == VariableId::Kind::NODE ? HasNode(id.node_id()) : HasSubmap(id.submap_id());
}

const SessionData& PoseGraphData::session(SessionId id) const {
  const auto it = sessions_.find(id);
  CHECK(it != sessions_.end()) << "unknown session " << id.session_index;
  return it->second;
}

const Node& PoseGraphData::node(const NodeId& id) const {
  const auto it = nodes_.find(id);
  CHECK(it != nodes_.end()) << "unknown node " << id.session_id << "." << id.node_index;
  return it->second;
}

const SubmapRecord& PoseGraphData::submap(const SubmapId& id) const {
  const auto it = submaps_.find(id);
  CHECK(it != submaps_.end()) << "unknown submap " << id.session_id << "." << id.submap_index;
  return it->second;
}

std::vector<SubmapId> PoseGraphData::ContainingSubmapIds(const NodeId& id) const {
  const auto it = node_to_submaps_.find(id);
  if (it == node_to_submaps_.end()) {
    return {};
  }
  return {it->second.begin(), it->second.end()};
}

std::optional<Eigen::Affine2d> PoseGraphData::ComputeSessionToGlobal(SessionId id) const {
  const SessionData& data = session(id);
  if (data.submap_ids.empty()) {
    return std::nullopt;
  }
  const SubmapRecord& last = submap(data.submap_ids.back());
  return Eigen::Affine2d(last.global_pose * last.local_pose.inverse());
}

std::vector<SubmapId> PoseGraphData::TrimmableSubmapIds() const {
  std::vector<SubmapId> ids;
  for (const auto& [id, record] : submaps_) {
    if (!session(SessionOf(id)).frozen()) {
      ids.push_back(id);
    }
  }
  return ids;
}

TrimReport PoseGraphData::ApplyTrim(const TrimRequest& request) {
  for (const SubmapId& id : request.deleted_submap_ids) {
    CHECK(HasSubmap(id));
    CHECK(!session(SessionOf(id)).frozen()) << "trimming never touches a frozen session";
  }

  TrimReport report;
  // Successions read the global poses, so they have to be taken before anything is deleted.
  for (const SubmapId& id : request.deleted_submap_ids) {
    TrimReport::SubmapSuccession succession;
    succession.deleted_submap_id = id;
    const auto it = request.successors.find(id);
    if (it != request.successors.end()) {
      CHECK(HasSubmap(it->second));
      CHECK(std::find(request.deleted_submap_ids.begin(), request.deleted_submap_ids.end(),
                      it->second) == request.deleted_submap_ids.end())
          << "a successor cannot itself be trimmed in the same request";
      succession.successor_submap_id = it->second;
      succession.successor_from_deleted =
          submap(it->second).global_pose.inverse() * submap(id).global_pose;
    }
    report.submap_successions.push_back(succession);
  }

  std::set<NodeId> orphan_candidates;
  std::set<VariableId> removed;
  for (const SubmapId& id : request.deleted_submap_ids) {
    const std::set<NodeId>& node_ids = submap(id).node_ids;
    orphan_candidates.insert(node_ids.begin(), node_ids.end());
    removed.insert(VariableId::Of(id));
    RemoveSubmap(id);
  }

  // A node lives as long as one submap still holds it.
  for (const NodeId& id : orphan_candidates) {
    if (node_to_submaps_.at(id).empty()) {
      removed.insert(VariableId::Of(id));
      report.deleted_node_ids.push_back(id);
      RemoveNode(id);
    }
  }

  RemoveConstraintsTouching(removed);

  for (const Constraint& constraint : request.added_constraints) {
    AddConstraint(constraint);
  }
  report.added_constraints = request.added_constraints;
  return report;
}

void PoseGraphData::RemoveSession(SessionId id) {
  CHECK(!session(id).frozen()) << "a frozen session is immutable and cannot be removed";

  std::set<VariableId> removed;
  const std::vector<SubmapId> submap_ids = session(id).submap_ids;
  for (const SubmapId& submap_id : submap_ids) {
    removed.insert(VariableId::Of(submap_id));
    RemoveSubmap(submap_id);
  }
  const std::vector<NodeId> node_ids = session(id).node_ids;
  for (const NodeId& node_id : node_ids) {
    // Membership left in another session's submap would dangle once the node is gone.
    for (const SubmapId& submap_id : node_to_submaps_.at(node_id)) {
      submaps_.at(submap_id).node_ids.erase(node_id);
    }
    removed.insert(VariableId::Of(node_id));
    RemoveNode(node_id);
  }
  RemoveConstraintsTouching(removed);
  sessions_.erase(id);
}

SessionData& PoseGraphData::MutableSession(SessionId id) {
  const auto it = sessions_.find(id);
  CHECK(it != sessions_.end()) << "unknown session " << id.session_index;
  return it->second;
}

void PoseGraphData::RemoveSubmap(const SubmapId& id) {
  for (const NodeId& node_id : submaps_.at(id).node_ids) {
    node_to_submaps_.at(node_id).erase(id);
  }
  std::vector<SubmapId>& session_submaps = MutableSession(SessionOf(id)).submap_ids;
  session_submaps.erase(std::remove(session_submaps.begin(), session_submaps.end(), id),
                        session_submaps.end());
  submaps_.erase(id);
}

void PoseGraphData::RemoveNode(const NodeId& id) {
  std::vector<NodeId>& session_nodes = MutableSession(SessionOf(id)).node_ids;
  session_nodes.erase(std::remove(session_nodes.begin(), session_nodes.end(), id),
                      session_nodes.end());
  node_to_submaps_.erase(id);
  nodes_.erase(id);
}

void PoseGraphData::RemoveConstraintsTouching(const std::set<VariableId>& removed) {
  constraints_.erase(
      std::remove_if(constraints_.begin(), constraints_.end(),
                     [&removed](const Constraint& constraint) {
                       return removed.count(constraint.from) > 0 ||
                              (constraint.to.has_value() && removed.count(*constraint.to) > 0);
                     }),
      constraints_.end());
}

}  // namespace evergreenslam::lifelong
