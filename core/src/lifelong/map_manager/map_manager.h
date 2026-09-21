/**
 * @file map_manager.h
 * @author hang chen (chen@hang.plus)
 * @brief Persistence: one file per session plus a manifest, checkpoints, load at boot.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_MANAGER_H_
#define EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_MANAGER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/sessions/session_data.h"
#include "lifelong/sessions/session_manager.h"

namespace evergreenslam::lifelong {

// 3: submap grids are in the submap frame. Older files are refused, not migrated.
constexpr int kMapFormatVersion = 3;

// Checkpoints go through a temp file and a rename, so a kill at any moment leaves the previous
// checkpoint intact. A frozen session's file is written once more and then never again.
class MapManager : public SessionObserver {
 public:
  struct LoadResult {
    struct UnfrozenSession {
      SessionId id;
      int next_submap_index = 0;
      int next_node_index = 0;
      int last_fed_boot = 0;
    };

    int num_sessions = 0;
    int num_frozen_sessions = 0;
    std::vector<UnfrozenSession> unfrozen_sessions;
    std::optional<NodeId> last_checkpoint_node;
    std::optional<Eigen::Affine2d> last_checkpoint_global_pose;
  };

  explicit MapManager(std::string directory);

  void OnSessionFrozen(const PoseGraphData& graph, SessionId id) override;
  void OnSessionStarted(const PoseGraphData& graph, SessionId id) override;

  void Checkpoint(const PoseGraphData& graph, SessionId id);

  struct LastPose {
    NodeId node_id;
    common::Time time;
    Eigen::Affine2d global_pose = Eigen::Affine2d::Identity();
  };
  void WriteLastPose(const Node& node);
  std::optional<LastPose> ReadLastPose() const;

  // Once per Start.
  int RecordBoot();

  void RemoveSession(const PoseGraphData& graph, SessionId id);
  std::optional<LoadResult> Load(PoseGraphData& graph);

  struct FileSummary {
    int version = 0;
    bool frozen = false;
    int num_submaps = 0;
    int num_nodes = 0;
    int num_constraints = 0;
  };
  std::optional<FileSummary> InspectSessionFile(SessionId id) const;

  static std::string SessionFileName(SessionId id);
  static std::string LastPoseFileName();

  const std::string& directory() const { return directory_; }
  int boot_count() const { return boot_count_; }
  int num_checkpoints_written() const { return num_checkpoints_written_; }
  int num_frozen_files_written() const { return num_frozen_files_written_; }

 private:
  struct Entry {
    SessionState state = SessionState::ACTIVE;
    std::string file_name;
    int last_fed_boot = 0;
  };

  void WriteSessionFile(const PoseGraphData& graph, SessionId id, bool frozen, bool fed);
  void RewriteUnfrozenSessionFiles(const PoseGraphData& graph);
  void WriteManifest() const;
  std::string PathOf(const std::string& file_name) const;

  std::string directory_;
  std::map<SessionId, Entry> manifest_;
  int boot_count_ = 0;
  int num_checkpoints_written_ = 0;
  int num_frozen_files_written_ = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_MANAGER_H_
