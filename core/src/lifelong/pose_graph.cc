/**
 * @file pose_graph.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph.h"

#include <glog/logging.h>

#include <future>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "lifelong/constraints/match_worker_pool.h"
#include "lifelong/global_map.h"
#include "lifelong/sessions/frozen_links.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::lifelong {
namespace {

PoseGraphOption ResolveMatchWorkers(PoseGraphOption option) {
  if (option.constraint_builder.num_match_workers <= 0) {
    option.constraint_builder.num_match_workers = MatchWorkerPool::DefaultNumWorkers();
  }
  return option;
}

bool IsKnownFree(const PoseGraphData& graph, const SubmapId& submap_id,
                 const Eigen::Vector2d& point_in_submap) {
  const std::shared_ptr<const mapping::Submap>& submap = graph.submap(submap_id).submap;
  if (submap == nullptr) {
    return false;
  }
  // Read from a snapshot to avoid racing with frontend updates.
  const uint8_t value = submap->SnapshotCopy().GetValueAtPoint(point_in_submap);
  return mapping::IsKnownValue(value) && mapping::ValueToProbability(value) < 0.5;
}

}  // namespace

// Republished right after rotation, so the caller's next map->odom read sees the successor's frame.
class PoseGraph::SessionAlignmentObserver : public SessionObserver {
 public:
  explicit SessionAlignmentObserver(PoseGraph& backend) : backend_(backend) {}

  void OnSessionFrozen(const PoseGraphData& graph, SessionId id) override {}
  void OnSessionStarted(const PoseGraphData& graph, SessionId id) override {
    backend_.RepublishActiveSessionToGlobal();
  }

 private:
  PoseGraph& backend_;
};

PoseGraph::PoseGraph(const PoseGraphOption& option, const std::string& map_directory)
    : option_(ResolveMatchWorkers(option)),
      optimization_(option_.optimization, option_.constraint_weight),
      session_manager_(*this, option_.session_manager),
      constraint_builder_(*this, option_.constraint_builder, option_.constraint_weight),
      trimmer_(*this, option_.trimmer) {
  if (!map_directory.empty()) {
    map_manager_ = std::make_shared<MapManager>(map_directory, anchor_store_);
    session_manager_.AddObserver(map_manager_);
  }
  session_manager_.AddObserver(std::make_shared<SessionAlignmentObserver>(*this));
}

PoseGraph::~PoseGraph() {
  // Members declared after task_queue_ die first, and ~TaskQueue still runs what is queued.
  WaitUntilQuiescent();
}

void PoseGraph::Start(common::Time time, std::optional<Eigen::Affine2d> initial_global_pose,
                      bool seed_from_previous_boot) {
  CHECK(!started_) << "Start boots the backend once";
  started_ = true;
  submap_translation_.clear();
  retired_local_indices_.clear();
  watched_submaps_.clear();
  last_ingested_node_index_ = -1;
  nodes_since_cadence_ = 0;

  std::optional<Eigen::Affine2d> alignment = initial_global_pose;
  if (map_manager_ != nullptr) {
    const MapManager::LoadOutcome outcome = map_manager_->Load(data_);
    if (std::holds_alternative<MapManager::LoadFailure>(outcome)) {
      const MapManager::LoadFailure& failure = std::get<MapManager::LoadFailure>(outcome);
      LOG(FATAL) << "refusing to boot on a damaged map directory: " << map_manager_->directory()
                 << ": " << ToString(failure.reason) << ": " << failure.detail;
    }
    std::optional<MapManager::LoadResult> loaded;
    if (std::holds_alternative<MapManager::LoadResult>(outcome)) {
      loaded = std::get<MapManager::LoadResult>(outcome);
    }
    map_manager_->RemoveUnreferencedFiles();
    if (!map_manager_->RecordBoot().has_value()) {
      LOG(FATAL) << "map directory is not writable: " << map_manager_->directory();
    }
    if (loaded.has_value()) {
      LOG(INFO) << "loaded " << loaded->num_sessions << " sessions (" << loaded->num_frozen_sessions
                << " frozen) from " << map_manager_->directory() << ", boot "
                << map_manager_->boot_count();
      for (const MapManager::LoadResult::UnfrozenSession& unfrozen : loaded->unfrozen_sessions) {
        data_.FinishSubmaps(unfrozen.id);
      }
      optimization_.BuildFrom(data_);
    }
    std::optional<AnchorTable> anchors = map_manager_->ReadAnchors();
    CHECK(anchors.has_value()) << "refusing to boot on an unreadable "
                               << map_manager_->anchors_file_name().value_or("anchor file")
                               << " in " << map_manager_->directory();
    anchor_store_.Restore(std::move(*anchors));
    if (loaded.has_value()) {
      GcFloatingSessions(*loaded);
    }
    anchor_store_.Reconcile(data_);
    map_manager_->Commit(data_);
    if (loaded.has_value()) {
      std::optional<common::Time> checkpoint_time;
      if (loaded->last_checkpoint_node.has_value() &&
          data_.HasNode(*loaded->last_checkpoint_node)) {
        // Covariances and the gauge assignment mean nothing before the first solve.
        Optimize();
        Drain();
        const Node& checkpoint_node = data_.node(*loaded->last_checkpoint_node);
        checkpoint_time = checkpoint_node.constant_data.time;
        if (seed_from_previous_boot && !alignment.has_value()) {
          // Re-read after the solve, so the alignment and the graph agree.
          alignment = checkpoint_node.global_pose;
        }
      }
      // The per-keyframe last pose beats the checkpoint only when newer and still in the map.
      const std::optional<MapManager::LastPose> last_pose = map_manager_->ReadLastPose();
      if (seed_from_previous_boot && !initial_global_pose.has_value() && last_pose.has_value() &&
          data_.HasSession(SessionOf(last_pose->node_id)) && !data_.HasNode(last_pose->node_id) &&
          (!checkpoint_time.has_value() || last_pose->time > *checkpoint_time)) {
        alignment = last_pose->global_pose;
      }
    }
  }

  last_checkpoint_time_ = time;
  boot_first_session_ =
      session_manager_.Start(time, alignment.value_or(Eigen::Affine2d::Identity()));
  Drain();
  RepublishActiveSessionToGlobal();
}

void PoseGraph::AddInsertionResult(const mapping::LocalTrajectoryBuilder::InsertionResult& result) {
  CHECK(started_) << "boot through Start first";

  EnqueueInsertionResult(result);
  EnqueueNodeSearch();
  SearchForFinishedSubmaps(result);
  task_queue_.Enqueue([this] { FinishBatch(); });
}

void PoseGraph::Finish() {
  CHECK(started_);
  WaitUntilQuiescent();
  Optimize();
  task_queue_.Enqueue([this] { session_manager_.OnOptimizedOnTask(); });
  WaitUntilQuiescent();
  if (map_manager_ != nullptr) {
    CheckpointFedSession();
  }
  RepublishActiveSessionToGlobal();
}

void PoseGraph::SetInitialPose(const Eigen::Affine2d& global_pose) {
  CHECK(started_) << "boot through Start first";
  task_queue_.Enqueue([this, global_pose] { RelocalizeOnTask(global_pose); });
}

void PoseGraph::RelocalizeGlobally() {
  CHECK(started_) << "boot through Start first";
  task_queue_.Enqueue([this] { RelocalizeOnTask(std::nullopt); });
}

void PoseGraph::RelocalizeOnTask(const std::optional<Eigen::Affine2d>& prior) {
  const std::vector<NodeId>& node_ids = data_.session(*session_manager_.fed_session()).node_ids;
  if (node_ids.empty()) {
    pending_relocalization_ = PendingRelocalization{prior};
    return;
  }
  constraint_builder_.SearchForNodeAroundOnTask(node_ids.back(), prior);
}

// A floating session seeded at identity can sit tens of metres and a right angle from where its
// first closure says it is; one Huber-lossed edge is a poor way to move it there.
bool PoseGraph::ReseedOnFirstAnchoring(const Constraint& constraint) {
  const std::optional<SessionId> fed = session_manager_.fed_session();
  if (!fed.has_value() || !constraint.to.has_value()) {
    return false;
  }
  const bool fed_from = constraint.from.session() == *fed;
  const bool fed_to = constraint.to->session() == *fed;
  if (fed_from == fed_to) {
    return false;
  }
  const SessionId other = fed_from ? constraint.to->session() : constraint.from.session();
  if (!data_.session(other).frozen() || HasFrozenLink(data_, *fed)) {
    return false;
  }
  CHECK(constraint.from.kind == VariableId::Kind::SUBMAP &&
        constraint.to->kind == VariableId::Kind::NODE);
  Eigen::Affine2d local_to_global = Eigen::Affine2d::Identity();
  if (fed_to) {
    const Node& node = data_.node(constraint.to->node_id());
    local_to_global = data_.submap(constraint.from.submap_id()).global_pose *
                      constraint.relative_pose * node.constant_data.local_pose.inverse();
  } else {
    const SubmapRecord& submap = data_.submap(constraint.from.submap_id());
    local_to_global = data_.node(constraint.to->node_id()).global_pose *
                      constraint.relative_pose.inverse() * submap.local_pose.inverse();
  }
  ReseedSession(*fed, local_to_global);
  return true;
}

void PoseGraph::ReseedSession(SessionId id, const Eigen::Affine2d& local_to_global) {
  data_.SetSessionLocalToGlobal(id, local_to_global);
  const SessionData& session = data_.session(id);
  for (const SubmapId& submap_id : session.submap_ids) {
    const Eigen::Affine2d pose = local_to_global * data_.submap(submap_id).local_pose;
    data_.SetSubmapGlobalPose(submap_id, pose);
    optimization_.SetVariablePose(VariableId::Of(submap_id), pose);
  }
  for (const NodeId& node_id : session.node_ids) {
    const Eigen::Affine2d pose = local_to_global * data_.node(node_id).constant_data.local_pose;
    data_.SetNodeGlobalPose(node_id, pose);
    optimization_.SetVariablePose(VariableId::Of(node_id), pose);
  }
}

bool PoseGraph::SolveAtBatchEnd() {
  if (nodes_since_cadence_ < option_.optimization.optimize_every_n_nodes) {
    return false;
  }
  nodes_since_cadence_ = 0;
  OptimizeOnTask();
  return true;
}

void PoseGraph::FinishBatch() {
  if (SolveAtBatchEnd()) {
    if (option_.trim && ++optimizations_since_trim_ >= option_.trim_every_n_optimizations) {
      optimizations_since_trim_ = 0;
      const int trimmed_before = trimmer_.num_submaps_trimmed();
      trimmer_.TrimToCompletionOnTask();
      if (trimmer_.num_submaps_trimmed() > trimmed_before) {
        // Inline, not enqueued: the checkpoint below must persist the re-solved state.
        OptimizeOnTask();
      }
    }
    session_manager_.OnOptimizedOnTask();
    if (map_manager_ != nullptr && CheckpointDue()) {
      CheckpointFedSession();
    }
  }
  RepublishActiveSessionToGlobal();
}

bool PoseGraph::CheckpointDue() const {
  const std::optional<SessionId> fed = session_manager_.fed_session();
  CHECK(fed.has_value());
  return data_.session(*fed).last_node_time - last_checkpoint_time_ >=
         option_.checkpoint_min_interval;
}

bool PoseGraph::CheckpointFedSession() {
  const std::optional<SessionId> fed = session_manager_.fed_session();
  CHECK(fed.has_value()) << "checkpoints only happen after Start opened the fed session";
  if (!map_manager_->Checkpoint(data_, *fed)) {
    return false;
  }
  last_checkpoint_time_ = data_.session(*fed).last_node_time;
  return true;
}

bool PoseGraph::CheckpointOnTask() { return map_manager_ == nullptr || CheckpointFedSession(); }

std::optional<NodeId> PoseGraph::last_ingested_node() const {
  if (last_ingested_node_index_ < 0) {
    return std::nullopt;
  }
  return last_ingested_node_;
}

PoseGraph::SaveAnchorResult PoseGraph::SaveAnchorOnTask(bool keep_scan,
                                                        std::optional<AnchorId> rebind,
                                                        const Eigen::Affine2d& node_from_anchor) {
  using Refusal = SaveAnchorResult::Refusal;
  const std::optional<NodeId> node = last_ingested_node();
  if (!node.has_value()) {
    return SaveAnchorResult{std::nullopt, Refusal::NO_KEYFRAME};
  }
  const std::optional<SubmapId> binding = AnchorStore::BindingSubmap(data_, *node);
  if (binding.has_value() && !node_from_anchor.translation().isZero(0.0)) {
    const Eigen::Vector2d target = data_.submap(*binding).global_pose.inverse() *
                                   data_.node(*node).global_pose * node_from_anchor.translation();
    if (!IsKnownFree(data_, *binding, target)) {
      return SaveAnchorResult{std::nullopt, Refusal::OFFSET_NOT_FREE};
    }
  }
  const std::optional<Anchor> previous =
      rebind.has_value() ? anchor_store_.Get(*rebind) : std::nullopt;
  std::optional<Anchor> anchor =
      rebind.has_value() ? anchor_store_.Rebind(data_, *rebind, *node, keep_scan, node_from_anchor)
                         : anchor_store_.Save(data_, *node, keep_scan, node_from_anchor);
  if (!anchor.has_value()) {
    return SaveAnchorResult{std::nullopt, Refusal::NODE_GONE};
  }
  // A refused save leaves the table as the disk has it, or the next commit would land it anyway.
  if (map_manager_ != nullptr && !CheckpointFedSession()) {
    if (previous.has_value()) {
      anchor_store_.Replace(*previous);
    } else {
      anchor_store_.Erase(anchor->id);
    }
    return SaveAnchorResult{std::nullopt, Refusal::NOT_PERSISTED};
  }
  return SaveAnchorResult{std::move(anchor), Refusal::NONE};
}

void PoseGraph::WaitUntilQuiescent() {
  do {
    Drain();
    constraint_builder_.WaitForMatches();
  } while (task_queue_.pending_count() > 0);
}

mapping::GridMapu8 PoseGraph::AssembleGlobalMap() {
  std::promise<mapping::GridMapu8> promise;
  std::future<mapping::GridMapu8> future = promise.get_future();
  task_queue_.Enqueue([this, &promise] {
    promise.set_value(lifelong::AssembleGlobalMap(data_, 0.0, /*only_finished=*/true));
  });
  return future.get();
}

void PoseGraph::AssembleGlobalMapAsync(std::function<void(mapping::GridMapu8)> callback) {
  task_queue_.Enqueue([this, cb = std::move(callback)] {
    cb(lifelong::AssembleGlobalMap(data_, 0.0, /*only_finished=*/true));
  });
}

std::optional<Eigen::Affine2d> PoseGraph::ActiveSessionToGlobal() const {
  std::lock_guard<std::mutex> lock(active_session_to_global_mutex_);
  return active_session_to_global_;
}

SessionId PoseGraph::StartNewSession(common::Time time, const Eigen::Affine2d& local_to_global) {
  return data_.StartNewSession(time, local_to_global);
}

void PoseGraph::EnqueueInsertionResult(
    const mapping::LocalTrajectoryBuilder::InsertionResult& result) {
  task_queue_.Enqueue([this, result] { HandleInsertionResult(result); });
}

void PoseGraph::EnqueueNodeSearch() {
  task_queue_.Enqueue([this] { constraint_builder_.SearchForNodeOnTask(last_ingested_node_); });
}

void PoseGraph::AddConstraint(const Constraint& constraint) {
  task_queue_.Enqueue([this, constraint] { HandleConstraint(constraint); });
}

bool PoseGraph::AddConstraintIfEndpointsLive(const Constraint& constraint) {
  if (!data_.HasVariable(constraint.from) ||
      (constraint.to.has_value() && !data_.HasVariable(*constraint.to))) {
    return false;
  }
  HandleConstraint(constraint);
  return true;
}

TrimReport PoseGraph::ApplyTrim(const TrimRequest& request) {
  // The graph's report is what tells the Problem which blocks to drop.
  TrimReport report = data_.ApplyTrim(request);
  anchor_store_.OnTrim(report);
  for (const SubmapId& id : request.deleted_submap_ids) {
    optimization_.RemoveVariable(VariableId::Of(id));
  }
  for (const NodeId& id : report.deleted_node_ids) {
    optimization_.RemoveVariable(VariableId::Of(id));
  }
  // Recovered edges only re-attach what the trim removed: the judge's link count scans the
  // graph, so they are not accounted here and deliberately do not re-judge the session.
  for (const Constraint& constraint : request.added_constraints) {
    optimization_.AddConstraint(constraint);
  }
  return report;
}

void PoseGraph::RemoveSession(SessionId id) {
  // Collected before the graph erases the SessionData that lists them.
  const SessionData& session = data_.session(id);
  std::vector<VariableId> variables;
  variables.reserve(session.submap_ids.size() + session.node_ids.size());
  for (const SubmapId& submap_id : session.submap_ids) {
    variables.push_back(VariableId::Of(submap_id));
  }
  for (const NodeId& node_id : session.node_ids) {
    variables.push_back(VariableId::Of(node_id));
  }
  data_.RemoveSession(id);
  anchor_store_.OnSessionRemoved(id);
  for (const VariableId& variable : variables) {
    optimization_.RemoveVariable(variable);
  }
  if (map_manager_ != nullptr) {
    map_manager_->RemoveSession(data_, id);
  }
}

void PoseGraph::Optimize() {
  task_queue_.Enqueue([this] { OptimizeOnTask(); });
}

void PoseGraph::OptimizeOnTask() { optimization_.Optimize(data_); }

void PoseGraph::FreezeSession(SessionId id) {
  task_queue_.Enqueue([this, id] {
    optimization_.Optimize(data_);
    data_.FreezeSession(id);
    optimization_.FreezeSession(id);
  });
}

SessionId PoseGraph::RotateAndFreezeFedSessionOnTask(SessionId id) {
  optimization_.Optimize(data_);
  // Read before the handover, off the submap the successor's own chain will start from, so the
  // frontend's local frame maps to global continuously across the boundary.
  const std::optional<Eigen::Affine2d> session_to_global = data_.ComputeSessionToGlobal(id);
  const SessionId next =
      data_.StartNewSession(data_.session(id).last_node_time,
                            session_to_global.value_or(data_.session(id).local_to_global));
  const std::map<SubmapId, SubmapId> adoption = TransferUnfinishedSubmaps(id, next);
  for (auto it = submap_translation_.begin(); it != submap_translation_.end();) {
    const auto renamed = adoption.find(it->second);
    if (renamed != adoption.end()) {
      it->second = renamed->second;
      ++it;
    } else {
      retired_local_indices_.insert(it->first);
      it = submap_translation_.erase(it);
    }
  }
  anchor_store_.OnSubmapsTransferred(adoption);
  data_.FreezeSession(id);
  optimization_.FreezeSession(id);
  return next;
}

void PoseGraph::TrimSubmapsOnTask(const std::vector<SubmapId>& ids) {
  if (!option_.trim) {
    return;
  }
  for (const SubmapId& id : ids) {
    trimmer_.EnqueueForTrim(id);
  }
  trimmer_.TrimToCompletionOnTask();
}

void PoseGraph::FreezeFedSession(std::optional<SessionId> expected) {
  CHECK(started_) << "boot through Start first";
  session_manager_.FreezeFedSession(expected);
}

PoseGraph::DropFedSessionResult PoseGraph::DropFedSession(SessionId id) {
  CHECK(started_) << "boot through Start first";
  std::promise<DropFedSessionResult> promise;
  std::future<DropFedSessionResult> future = promise.get_future();
  task_queue_.Enqueue([this, id, &promise] { promise.set_value(DropFedSessionOnTask(id)); });
  return future.get();
}

PoseGraph::DropFedSessionResult PoseGraph::DropFedSessionOnTask(SessionId id) {
  using Refusal = DropFedSessionResult::Refusal;
  if (session_manager_.fed_session() != id) {
    return DropFedSessionResult{std::nullopt, Refusal::NOT_FED};
  }
  if (session_manager_.freezing()) {
    return DropFedSessionResult{std::nullopt, Refusal::FREEZING};
  }
  const common::Time time = data_.session(id).last_node_time;
  // The successor first, so a fed session exists throughout and a kill between the two commits
  // leaves the old session on disk.
  const SessionId next = session_manager_.ReplaceFedSessionOnTask(time);
  RemoveSession(id);
  constraint_builder_.SweepStaleStateOnTask();

  submap_translation_.clear();
  retired_local_indices_.clear();
  watched_submaps_.clear();
  last_ingested_node_index_ = -1;
  nodes_since_cadence_ = 0;
  pending_relocalization_.reset();
  last_checkpoint_time_ = time;
  RepublishActiveSessionToGlobal();
  LOG(INFO) << "fed session " << id.session_index << " dropped; session " << next.session_index
            << " is fed now";
  return DropFedSessionResult{next, Refusal::NONE};
}

std::map<SubmapId, SubmapId> PoseGraph::TransferUnfinishedSubmaps(SessionId from, SessionId to) {
  std::map<SubmapId, SubmapId> adoption;
  // A copy: TransferSubmap edits the session's submap list while we walk it.
  const std::vector<SubmapId> submap_ids = data_.session(from).submap_ids;
  for (const SubmapId& old_id : submap_ids) {
    const SubmapRecord& record = data_.submap(old_id);
    if (record.submap == nullptr || record.submap->finished()) {
      continue;
    }
    const SubmapId new_id = data_.AllocateSubmapId(to);
    data_.TransferSubmap(old_id, new_id);
    optimization_.RenameVariable(VariableId::Of(old_id), VariableId::Of(new_id));
    adoption.emplace(old_id, new_id);
  }
  return adoption;
}

void PoseGraph::HandleInsertionResult(
    const mapping::LocalTrajectoryBuilder::InsertionResult& result) {
  const std::optional<SessionId> fed = session_manager_.fed_session();
  CHECK(fed.has_value()) << "start the backend before feeding it";
  const SessionId session_id = *fed;
  CHECK(!result.insertion_submaps.empty());
  CHECK_GT(result.node_index, last_ingested_node_index_)
      << "keyframes must arrive in builder order, exactly once";
  last_ingested_node_index_ = result.node_index;

  std::vector<SubmapId> submap_ids;
  submap_ids.reserve(result.insertion_submaps.size());
  for (const auto& submap : result.insertion_submaps) {
    CHECK(submap != nullptr);
    const auto entry = submap_translation_.find(submap->local_index());
    if (entry != submap_translation_.end()) {
      if (data_.HasSubmap(entry->second)) {
        submap_ids.push_back(entry->second);
        continue;
      }
      // Trimmed under the frontend's feet: the window still names it, the graph no longer does.
      retired_local_indices_.insert(entry->first);
      submap_translation_.erase(entry);
    }
    if (retired_local_indices_.count(submap->local_index()) > 0) {
      continue;
    }
    const SubmapId submap_id = data_.AllocateSubmapId(session_id);
    submap_translation_.emplace(submap->local_index(), submap_id);
    SubmapRecord record;
    record.id = submap_id;
    record.submap = submap;
    record.local_pose = submap->local_pose();
    const std::optional<Eigen::Affine2d> session_to_global =
        data_.ComputeSessionToGlobal(session_id);
    const Eigen::Affine2d alignment = session_to_global.has_value()
                                          ? *session_to_global
                                          : data_.session(session_id).local_to_global;
    record.global_pose = alignment * record.local_pose;
    data_.AddSubmap(record);
    optimization_.AddVariable(VariableId::Of(record.id), record.global_pose);
    submap_ids.push_back(submap_id);
  }
  CHECK(!submap_ids.empty()) << "a keyframe with only pruned submaps cannot exist: the window "
                                "always carries an unfinished submap";

  const std::optional<Eigen::Affine2d> session_to_global = data_.ComputeSessionToGlobal(session_id);
  CHECK(session_to_global.has_value());

  Node node;
  node.id = data_.AllocateNodeId(session_id);
  node.constant_data = result.node;
  node.global_pose = *session_to_global * node.constant_data.local_pose;
  data_.AddNode(node, submap_ids);
  optimization_.AddVariable(VariableId::Of(node.id), node.global_pose);
  last_ingested_node_ = node.id;
  if (map_manager_ != nullptr) {
    map_manager_->WriteLastPose(node);
  }

  for (const SubmapId& submap_id : submap_ids) {
    Constraint constraint;
    constraint.type = Constraint::Type::INTRA_SUBMAP;
    constraint.from = VariableId::Of(submap_id);
    constraint.to = VariableId::Of(node.id);
    constraint.relative_pose =
        data_.submap(submap_id).local_pose.inverse() * node.constant_data.local_pose;
    constraint.sqrt_information = OdometrySqrtInformation(option_.constraint_weight);
    HandleConstraint(constraint);
  }

  if (pending_relocalization_.has_value()) {
    const std::optional<Eigen::Affine2d> prior = pending_relocalization_->prior;
    pending_relocalization_.reset();
    constraint_builder_.SearchForNodeAroundOnTask(node.id, prior);
  }
  ++nodes_since_cadence_;
}

void PoseGraph::HandleConstraint(const Constraint& constraint) {
  const bool closure = constraint.type == Constraint::Type::INTER_SUBMAP && !constraint.recovered;
  const bool first_anchoring = closure && ReseedOnFirstAnchoring(constraint);
  data_.AddConstraint(constraint);
  optimization_.AddConstraint(constraint);
  session_manager_.OnConstraintAddedOnTask(constraint);
  // Once per session, not per closure: the rigid reseed off one edge leaves the drift the
  // tracker already tightened its windows for, and a stationary robot would wait a cadence.
  if (first_anchoring) {
    OptimizeOnTask();
    RepublishActiveSessionToGlobal();
  }
}

void PoseGraph::GcFloatingSessions(const MapManager::LoadResult& loaded) {
  if (option_.gc_unfrozen_after_n_boots <= 0) {
    return;
  }
  // A long-unanchored floating session is a dead run: freezing it would call its poses truth.
  const int boot_count = map_manager_->boot_count();
  for (const MapManager::LoadResult::UnfrozenSession& unfrozen : loaded.unfrozen_sessions) {
    const int boots_behind = boot_count - unfrozen.last_fed_boot;
    if (boots_behind <= option_.gc_unfrozen_after_n_boots) {
      continue;
    }
    LOG(INFO) << "garbage collecting floating session " << unfrozen.id.session_index
              << ", last fed " << boots_behind << " boots ago";
    RemoveSession(unfrozen.id);
  }
}

void PoseGraph::RepublishActiveSessionToGlobal() {
  const std::optional<SessionId> fed = session_manager_.fed_session();
  CHECK(fed.has_value());
  const std::optional<Eigen::Affine2d> session_to_global = data_.ComputeSessionToGlobal(*fed);
  std::lock_guard<std::mutex> lock(active_session_to_global_mutex_);
  // The alignment moves with every solve even when the session does not.
  active_session_to_global_ = session_to_global;
}

void PoseGraph::SearchForFinishedSubmaps(
    const mapping::LocalTrajectoryBuilder::InsertionResult& result) {
  for (const std::shared_ptr<const mapping::Submap>& submap : result.insertion_submaps) {
    if (!submap->finished()) {
      watched_submaps_.emplace(submap->local_index(), submap);
    }
  }
  for (auto it = watched_submaps_.begin(); it != watched_submaps_.end();) {
    if (!it->second->finished()) {
      ++it;
      continue;
    }
    const int local_index = it->first;
    task_queue_.Enqueue([this, local_index] {
      const auto entry = submap_translation_.find(local_index);
      if (entry == submap_translation_.end()) {
        return;
      }
      constraint_builder_.SearchForSubmapOnTask(entry->second);
      session_manager_.OnSubmapFinishedOnTask(entry->second);
    });
    it = watched_submaps_.erase(it);
  }
}

}  // namespace evergreenslam::lifelong
