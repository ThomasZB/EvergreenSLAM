/**
 * @file pose_graph.h
 * @author hang chen (chen@hang.plus)
 * @brief The whole lifelong backend behind two calls: boot and per keyframe.
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/backend_handles.h"
#include "lifelong/constraints/constraint_builder.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/optimization/optimization.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/pose_graph_option.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer.h"
#include "lifelong/sessions/session_manager.h"
#include "lifelong/task_queue.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/local_trajectory_builder.h"

namespace evergreenslam::lifelong {

class PoseGraph : public TrimmingHandle, public ConstraintHandle, public SessionHandle {
 public:
  explicit PoseGraph(const PoseGraphOption& option = PoseGraphOption(),
                     const std::string& map_directory = "");
  ~PoseGraph();

  void Start(common::Time time, std::optional<Eigen::Affine2d> initial_global_pose = std::nullopt,
             bool seed_from_previous_boot = true);
  void AddInsertionResult(const mapping::LocalTrajectoryBuilder::InsertionResult& result);
  void Finish();

  // Both return at once: the search is an ordinary backend loop-closure job, the pose only its
  // prior. A hint sent before the first keyframe waits for it; a newer hint replaces it.
  void SetInitialPose(const Eigen::Affine2d& global_pose);
  void RelocalizeGlobally();

  void WaitUntilQuiescent();

  // Runs on the backend task and skips unfinished submaps, still written by the frontend thread.
  mapping::GridMapu8 AssembleGlobalMap();
  // Non-blocking variant: enqueues the assembly as a task and delivers the result via callback.
  void AssembleGlobalMapAsync(std::function<void(mapping::GridMapu8)> callback);

  std::optional<Eigen::Affine2d> ActiveSessionToGlobal() const;
  // Writes the graph on the caller's thread: call it before the first task or with the queue
  // drained; the task-side path is RotateAndFreezeFedSessionOnTask.
  SessionId StartNewSession(common::Time time, const Eigen::Affine2d& local_to_global =
                                                   Eigen::Affine2d::Identity()) override;

  void EnqueueInsertionResult(const mapping::LocalTrajectoryBuilder::InsertionResult& result);

  void AddConstraint(const Constraint& constraint);
  bool AddConstraintIfEndpointsLive(const Constraint& constraint) override;
  TrimReport ApplyTrim(const TrimRequest& request) override;
  void RemoveSession(SessionId id);
  void RepublishActiveSessionToGlobal();

  void Optimize();
  void OptimizeOnTask();
  // Solves when the node cadence is due and returns whether it did, which is also whether trim
  // and checkpoint are due. Closures never solve here, or a loop-rich stretch would solve and
  // trim on every keyframe. Backend task only.
  bool SolveAtBatchEnd();

  void FreezeSession(SessionId id) override;
  SessionId RotateAndFreezeFedSessionOnTask(SessionId id) override;
  void TrimSubmapsOnTask(const std::vector<SubmapId>& ids) override;
  // The freeze sequence on demand, verdict or not; returns at once.
  void FreezeFedSession(std::optional<SessionId> expected = std::nullopt);

  // Backend task only, like every anchor read: the hooks rebind anchors on the queue.
  const AnchorStore& anchors() const { return anchor_store_; }
  std::optional<NodeId> last_ingested_node() const;
  struct SaveAnchorResult {
    enum class Refusal { NONE, NO_KEYFRAME, NODE_GONE, NOT_PERSISTED };
    // Empty exactly when refused: an anchor that did not reach disk may be reissued after a kill.
    std::optional<Anchor> anchor;
    Refusal refusal = Refusal::NONE;
  };
  SaveAnchorResult SaveAnchorOnTask(bool keep_scan, std::optional<AnchorId> rebind = std::nullopt);
  // Whether the checkpoint landed; true without a map directory.
  bool CheckpointOnTask();
  SessionId boot_first_session() const { return boot_first_session_; }

  // Graph and Problem must be re-keyed together, on the queue only. Returns old id -> new id.
  std::map<SubmapId, SubmapId> TransferUnfinishedSubmaps(SessionId from, SessionId to);

  void Enqueue(TaskQueue::Task task) override { task_queue_.Enqueue(std::move(task)); }
  void Drain() { task_queue_.Drain(); }

  const PoseGraphData& graph() const override { return data_; }
  PoseGraphData& mutable_graph() { return data_; }
  const Optimization& optimization() const { return optimization_; }
  Optimization& optimization() override { return optimization_; }
  Optimization& mutable_optimization() { return optimization_; }
  TaskQueue& task_queue() { return task_queue_; }
  const SessionManager& session_manager() const { return session_manager_; }
  SessionManager& mutable_session_manager() { return session_manager_; }
  const ConstraintBuilder& constraint_builder() const { return constraint_builder_; }
  const PoseGraphTrimmer& trimmer() const { return trimmer_; }
  const std::shared_ptr<MapManager>& map_manager() const { return map_manager_; }

 private:
  class SessionAlignmentObserver;

  void HandleInsertionResult(const mapping::LocalTrajectoryBuilder::InsertionResult& result);
  void HandleConstraint(const Constraint& constraint);
  // Must run after BuildFrom: RemoveSession needs the Problem to still hold the blocks.
  void GcFloatingSessions(const MapManager::LoadResult& loaded);

  void FinishBatch();
  bool CheckpointDue() const;
  bool CheckpointFedSession();
  void RelocalizeOnTask(const std::optional<Eigen::Affine2d>& prior);
  bool ReseedOnFirstAnchoring(const Constraint& constraint);
  void ReseedSession(SessionId id, const Eigen::Affine2d& local_to_global);
  void EnqueueNodeSearch();
  void SearchForFinishedSubmaps(const mapping::LocalTrajectoryBuilder::InsertionResult& result);

  PoseGraphOption option_;
  PoseGraphData data_;
  Optimization optimization_;
  TaskQueue task_queue_;

  SessionManager session_manager_;
  ConstraintBuilder constraint_builder_;
  PoseGraphTrimmer trimmer_;
  // Before map_manager_, which holds a reference to it.
  AnchorStore anchor_store_;
  std::shared_ptr<MapManager> map_manager_;

  mutable std::mutex active_session_to_global_mutex_;
  std::optional<Eigen::Affine2d> active_session_to_global_;

  bool started_ = false;
  SessionId boot_first_session_;
  std::map<int, SubmapId> submap_translation_;
  std::set<int> retired_local_indices_;
  NodeId last_ingested_node_;
  int last_ingested_node_index_ = -1;
  // Caller-thread state, keyed by local_index.
  std::map<int, std::shared_ptr<const mapping::Submap>> watched_submaps_;
  // Backend-task state.
  struct PendingRelocalization {
    std::optional<Eigen::Affine2d> prior;
  };
  std::optional<PendingRelocalization> pending_relocalization_;
  // The only solve trigger; freeze and first-anchoring solves neither reset nor shift it.
  int nodes_since_cadence_ = 0;
  int optimizations_since_trim_ = 0;
  common::Time last_checkpoint_time_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_H_
