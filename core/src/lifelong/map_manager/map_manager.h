/**
 * @file map_manager.h
 * @author hang chen (chen@hang.plus)
 * @brief Persistence: one file per session plus anchors behind a generation manifest.
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
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/map_manager/map_directory.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/sessions/session_data.h"
#include "lifelong/sessions/session_manager.h"

namespace evergreenslam::lifelong {

// 4: generation-named files behind a manifest. Older maps are refused, not migrated.
constexpr int kMapFormatVersion = 4;
constexpr int kAnchorFormatVersion = 1;

// Every write is one commit published by the manifest, so a kill leaves exactly one generation;
// each commit also heals what an earlier failed one left behind. Frozen files are write-once.
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
  // No manifest and no session or anchor file: a new map.
  struct FreshDirectory {};
  struct LoadFailure {
    enum class Reason {
      LOST_MANIFEST,
      UNREADABLE_MANIFEST,
      UNSUPPORTED_VERSION,
      MISSING_SESSION_FILE,
      CORRUPT_SESSION_FILE,
    };
    Reason reason = Reason::UNREADABLE_MANIFEST;
    std::string detail;
  };
  struct LastPose {
    NodeId node_id;
    common::Time time;
    Eigen::Affine2d global_pose = Eigen::Affine2d::Identity();
  };
  using LoadOutcome = std::variant<FreshDirectory, LoadResult, LoadFailure>;

  explicit MapManager(std::string directory);
  MapManager(std::string directory, const AnchorStore& anchors);

  void OnSessionFrozen(const PoseGraphData& graph, SessionId id) override;
  void OnSessionStarted(const PoseGraphData& graph, SessionId id) override;

  // Each returns whether its commit landed; a failed one changes nothing and a later one heals it.
  bool Checkpoint(const PoseGraphData& graph, SessionId id);
  // Anchors if changed, and the manifest, after reconciling it with the graph.
  bool Commit(const PoseGraphData& graph);

  void WriteLastPose(const Node& node);
  std::optional<LastPose> ReadLastPose() const;

  // Once per Start; writes the manifest only. The new boot count, or empty when it did not land.
  std::optional<int> RecordBoot();

  // The file the manifest read by Load names; call after Load.
  std::optional<AnchorTable> ReadAnchors();

  bool RemoveSession(const PoseGraphData& graph, SessionId id);
  LoadOutcome Load(PoseGraphData& graph);
  // After a Load that did not fail: removes `session_*.pb`, `anchors*.pb` and `*.tmp` the manifest
  // does not name, so never against a directory another process is committing to.
  void RemoveUnreferencedFiles();

  struct FileSummary {
    int version = 0;
    bool frozen = false;
    int num_submaps = 0;
    int num_nodes = 0;
    int num_constraints = 0;
  };
  std::optional<FileSummary> InspectSessionFile(SessionId id) const;

  std::optional<std::string> FileNameOf(SessionId id) const;
  std::optional<std::string> anchors_file_name() const;
  static std::string LastPoseFileName();

  void set_before_manifest_write_hook(std::function<void()> hook) {
    files_.set_before_manifest_write_hook(std::move(hook));
  }

  const std::string& directory() const { return files_.directory(); }
  int64_t generation() const { return files_.generation(); }
  int boot_count() const { return boot_count_; }
  int num_checkpoints_written() const { return num_checkpoints_written_; }
  int num_frozen_files_written() const { return num_frozen_files_written_; }
  int num_anchor_writes() const { return num_anchor_writes_; }

 private:
  struct Entry {
    SessionState state = SessionState::ACTIVE;
    std::string file_name;
    int last_fed_boot = 0;
  };
  struct SessionWrite {
    SessionId id;
    bool frozen = false;
    bool fed = false;
  };
  struct Staged {
    MapDirectory::Commit files;
    int boot_count = 0;
    int num_frozen_files = 0;
    std::map<SessionId, Entry> manifest;
    std::string anchors_file_name;
    std::optional<int64_t> anchors_revision;
  };

  Staged BeginCommit() const;
  bool StageSessionFile(const PoseGraphData& graph, const SessionWrite& write, Staged& staged);
  bool StageAnchorsFile(Staged& staged);
  // Either publishes every staged file through the manifest or removes them all.
  bool EndCommit(Staged& staged);
  bool CommitSessions(const PoseGraphData& graph, const std::vector<SessionWrite>& writes,
                      Staged staged);
  std::vector<SessionWrite> UnfrozenSessionWrites(const PoseGraphData& graph) const;
  std::string SerializeManifest(const Staged& staged) const;

  MapDirectory files_;
  const AnchorStore& anchors_;
  int64_t anchors_written_revision_ = 0;
  std::map<SessionId, Entry> manifest_;
  std::string anchors_file_name_;
  int boot_count_ = 0;
  int num_checkpoints_written_ = 0;
  int num_frozen_files_written_ = 0;
  int num_anchor_writes_ = 0;
};

const char* ToString(MapManager::LoadFailure::Reason reason);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_MANAGER_H_
