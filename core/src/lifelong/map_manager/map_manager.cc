/**
 * @file map_manager.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/map_manager.h"

#include <glog/logging.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "common/file.h"
#include "lifelong/map_manager/proto_conversion.h"

namespace evergreenslam::lifelong {
namespace {

constexpr char kManifestFileName[] = "manifest.pb";
constexpr char kLastPoseFileName[] = "last_pose.pb";

bool VersionSupported(int version) { return version == kMapFormatVersion; }

bool EndpointInSession(const Constraint& constraint, SessionId id) {
  return constraint.from.session() == id ||
         (constraint.to.has_value() && constraint.to->session() == id);
}

// The highest-indexed unfrozen endpoint session; empty when every endpoint is frozen.
std::optional<SessionId> UnfrozenOwnerOf(const PoseGraphData& graph, const Constraint& constraint) {
  std::optional<SessionId> owner;
  const auto consider = [&graph, &owner](SessionId session) {
    if (graph.session(session).frozen()) {
      return;
    }
    if (!owner.has_value() || *owner < session) {
      owner = session;
    }
  };
  consider(constraint.from.session());
  if (constraint.to.has_value()) {
    consider(constraint.to->session());
  }
  return owner;
}

std::vector<Constraint> ConstraintsOwnedBy(const PoseGraphData& graph, SessionId id, bool frozen) {
  std::vector<Constraint> owned;
  for (const Constraint& constraint : graph.constraints()) {
    const std::optional<SessionId> owner = UnfrozenOwnerOf(graph, constraint);
    const bool keep = frozen ? !owner.has_value() && EndpointInSession(constraint, id)
                             : owner.has_value() && *owner == id;
    if (keep) {
      owned.push_back(constraint);
    }
  }
  return owned;
}

}  // namespace

MapManager::MapManager(std::string directory) : directory_(std::move(directory)) {
  std::error_code error;
  std::filesystem::create_directories(directory_, error);
  LOG_IF(ERROR, error) << "cannot create map directory " << directory_ << ": " << error.message();
}

std::string MapManager::SessionFileName(SessionId id) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "session_%06d.pb", id.session_index);
  return std::string(buffer);
}

std::string MapManager::LastPoseFileName() { return kLastPoseFileName; }

std::string MapManager::PathOf(const std::string& file_name) const {
  return (std::filesystem::path(directory_) / file_name).string();
}

void MapManager::OnSessionFrozen(const PoseGraphData& graph, SessionId id) {
  const auto it = manifest_.find(id);
  CHECK(it == manifest_.end() || it->second.state != SessionState::FROZEN)
      << "the frozen file is written exactly once and never rewritten";
  CHECK(graph.session(id).frozen()) << "OnSessionFrozen runs after the freeze has landed";
  WriteSessionFile(graph, id, /*frozen=*/true, /*fed=*/false);
  ++num_frozen_files_written_;
  // Migrated cross-session constraints would otherwise live only in files never loaded again.
  RewriteUnfrozenSessionFiles(graph);
}

void MapManager::OnSessionStarted(const PoseGraphData& graph, SessionId id) {
  CHECK(!graph.session(id).frozen());
  WriteSessionFile(graph, id, /*frozen=*/false, /*fed=*/true);
}

void MapManager::Checkpoint(const PoseGraphData& graph, SessionId id) {
  CHECK(!graph.session(id).frozen()) << "a frozen session's file is write-once";
  WriteSessionFile(graph, id, /*frozen=*/false, /*fed=*/true);
  ++num_checkpoints_written_;
}

void MapManager::WriteLastPose(const Node& node) {
  proto::LastPose last_pose;
  *last_pose.mutable_node_id() = ToProto(node.id);
  last_pose.set_time_nanos(TimeToProto(node.constant_data.time));
  *last_pose.mutable_global_pose() = ToProto(node.global_pose);
  std::string contents;
  CHECK(last_pose.SerializeToString(&contents));
  common::WriteFileAtomically(PathOf(kLastPoseFileName), contents);
}

std::optional<MapManager::LastPose> MapManager::ReadLastPose() const {
  const std::optional<std::string> contents = common::ReadFile(PathOf(kLastPoseFileName));
  proto::LastPose proto;
  if (!contents.has_value() || !proto.ParseFromString(*contents)) {
    return std::nullopt;
  }
  LastPose last_pose;
  last_pose.node_id = FromProto(proto.node_id());
  last_pose.time = TimeFromProto(proto.time_nanos());
  last_pose.global_pose = FromProto(proto.global_pose());
  return last_pose;
}

int MapManager::RecordBoot() {
  ++boot_count_;
  WriteManifest();
  return boot_count_;
}

void MapManager::RemoveSession(const PoseGraphData& graph, SessionId id) {
  const auto it = manifest_.find(id);
  if (it != manifest_.end()) {
    std::error_code error;
    std::filesystem::remove(PathOf(it->second.file_name), error);
    LOG_IF(ERROR, error) << "cannot remove " << it->second.file_name << ": " << error.message();
    manifest_.erase(it);
    WriteManifest();
  }
  RewriteUnfrozenSessionFiles(graph);
}

void MapManager::RewriteUnfrozenSessionFiles(const PoseGraphData& graph) {
  for (const auto& [id, session] : graph.sessions()) {
    if (!session.frozen()) {
      WriteSessionFile(graph, id, /*frozen=*/false, /*fed=*/false);
    }
  }
}

void MapManager::WriteSessionFile(const PoseGraphData& graph, SessionId id, bool frozen, bool fed) {
  const SessionData& session = graph.session(id);
  proto::SessionFile file;
  file.set_version(kMapFormatVersion);
  *file.mutable_session() = ToProto(session);
  file.mutable_session()->set_next_submap_index(graph.id_allocator().next_submap_index(id));
  file.mutable_session()->set_next_node_index(graph.id_allocator().next_node_index(id));

  for (const SubmapId& submap_id : session.submap_ids) {
    *file.add_submaps() = ToProto(graph.submap(submap_id));
  }
  for (const NodeId& node_id : session.node_ids) {
    *file.add_nodes() = ToProto(graph.node(node_id), graph.ContainingSubmapIds(node_id));
  }
  for (const Constraint& constraint : ConstraintsOwnedBy(graph, id, frozen)) {
    *file.add_constraints() = ToProto(constraint);
  }

  const std::string file_name = SessionFileName(id);
  std::string contents;
  CHECK(file.SerializeToString(&contents));
  if (!common::WriteFileAtomically(PathOf(file_name), contents)) {
    LOG(ERROR) << "session " << id.session_index << " not written";
    return;
  }

  Entry& entry = manifest_[id];
  const int last_fed_boot = fed ? boot_count_ : entry.last_fed_boot;
  const bool manifest_changed = entry.file_name != file_name || entry.state != session.state ||
                                entry.last_fed_boot != last_fed_boot;
  entry.file_name = file_name;
  entry.state = session.state;
  entry.last_fed_boot = last_fed_boot;
  if (manifest_changed) {
    WriteManifest();
  }
}

void MapManager::WriteManifest() const {
  proto::Manifest manifest;
  manifest.set_version(kMapFormatVersion);
  manifest.set_boot_count(boot_count_);
  for (const auto& [id, entry] : manifest_) {
    proto::ManifestEntry* proto_entry = manifest.add_sessions();
    proto_entry->set_session_index(id.session_index);
    proto_entry->set_state(entry.state == SessionState::FROZEN ? proto::FROZEN : proto::ACTIVE);
    proto_entry->set_file_name(entry.file_name);
    proto_entry->set_last_fed_boot(entry.last_fed_boot);
  }
  std::string contents;
  CHECK(manifest.SerializeToString(&contents));
  common::WriteFileAtomically(PathOf(kManifestFileName), contents);
}

std::optional<MapManager::FileSummary> MapManager::InspectSessionFile(SessionId id) const {
  const std::optional<std::string> contents = common::ReadFile(PathOf(SessionFileName(id)));
  proto::SessionFile file;
  if (!contents.has_value() || !file.ParseFromString(*contents)) {
    return std::nullopt;
  }
  FileSummary summary;
  summary.version = file.version();
  summary.frozen = file.session().state() == proto::FROZEN;
  summary.num_submaps = file.submaps_size();
  summary.num_nodes = file.nodes_size();
  summary.num_constraints = file.constraints_size();
  return summary;
}

std::optional<MapManager::LoadResult> MapManager::Load(PoseGraphData& graph) {
  CHECK(graph.sessions().empty()) << "loading into a graph that already holds sessions";

  const std::optional<std::string> manifest_contents = common::ReadFile(PathOf(kManifestFileName));
  if (!manifest_contents.has_value()) {
    LOG(INFO) << "no manifest in " << directory_ << ", starting from an empty map";
    return std::nullopt;
  }
  proto::Manifest manifest;
  if (!manifest.ParseFromString(*manifest_contents)) {
    LOG(ERROR) << "manifest in " << directory_ << " is not a readable map manifest";
    return std::nullopt;
  }
  if (!VersionSupported(manifest.version())) {
    LOG(ERROR) << "manifest version " << manifest.version() << " is newer than "
               << kMapFormatVersion;
    return std::nullopt;
  }

  std::vector<proto::ManifestEntry> entries(manifest.sessions().begin(), manifest.sessions().end());
  std::sort(entries.begin(), entries.end(),
            [](const proto::ManifestEntry& lhs, const proto::ManifestEntry& rhs) {
              return lhs.session_index() < rhs.session_index();
            });

  LoadResult result;
  std::vector<Constraint> constraints;
  std::vector<std::pair<NodeId, SubmapId>> deferred_memberships;
  std::map<SessionId, Entry> loaded_manifest;
  std::optional<LoadResult::UnfrozenSession> last_fed_with_nodes;
  for (const proto::ManifestEntry& entry : entries) {
    const SessionId id{entry.session_index()};
    const std::optional<std::string> contents = common::ReadFile(PathOf(entry.file_name()));
    if (!contents.has_value()) {
      LOG(ERROR) << "session file " << entry.file_name() << " listed in the manifest is missing";
      return std::nullopt;
    }
    proto::SessionFile file;
    if (!file.ParseFromString(*contents)) {
      LOG(ERROR) << "session file " << entry.file_name() << " is corrupt";
      return std::nullopt;
    }
    if (!VersionSupported(file.version())) {
      LOG(ERROR) << "session file " << entry.file_name() << " has version " << file.version();
      return std::nullopt;
    }
    if (file.session().session_index() != id.session_index) {
      LOG(ERROR) << "session file " << entry.file_name() << " holds session "
                 << file.session().session_index() << ", manifest says " << id.session_index;
      return std::nullopt;
    }

    const SessionData session = FromProto(file.session());
    graph.RestoreSession(session, file.session().next_submap_index(),
                         file.session().next_node_index());
    for (const proto::Submap& submap : file.submaps()) {
      graph.AddSubmap(FromProto(submap));
    }
    for (const proto::Node& node : file.nodes()) {
      // A handed-over submap may live in a session file not loaded yet, so replay these last;
      // a membership whose submap never appears is dropped.
      std::vector<SubmapId> available;
      for (const SubmapId& submap_id : ContainingSubmapIdsFromProto(node)) {
        if (graph.HasSubmap(submap_id)) {
          available.push_back(submap_id);
        } else {
          deferred_memberships.emplace_back(FromProto(node.id()), submap_id);
        }
      }
      graph.AddNode(FromProto(node), available);
    }

    if (session.frozen()) {
      graph.FreezeSession(id);
      ++result.num_frozen_sessions;
    } else {
      for (const proto::Constraint& constraint : file.constraints()) {
        constraints.push_back(FromProto(constraint));
      }
      LoadResult::UnfrozenSession unfrozen;
      unfrozen.id = id;
      unfrozen.next_submap_index = file.session().next_submap_index();
      unfrozen.next_node_index = file.session().next_node_index();
      unfrozen.last_fed_boot = entry.last_fed_boot();
      result.unfrozen_sessions.push_back(unfrozen);
      if (!graph.session(id).node_ids.empty() &&
          (!last_fed_with_nodes.has_value() ||
           last_fed_with_nodes->last_fed_boot < unfrozen.last_fed_boot ||
           (last_fed_with_nodes->last_fed_boot == unfrozen.last_fed_boot &&
            last_fed_with_nodes->id < id))) {
        last_fed_with_nodes = unfrozen;
      }
    }
    CHECK(graph.session(id).last_node_time == session.last_node_time);
    loaded_manifest[id] = Entry{session.state, entry.file_name(), entry.last_fed_boot()};
    ++result.num_sessions;
  }

  for (const auto& [node_id, submap_id] : deferred_memberships) {
    if (graph.HasSubmap(submap_id)) {
      graph.AddNodeMembership(node_id, submap_id);
    }
  }
  for (const Constraint& constraint : constraints) {
    graph.AddConstraint(constraint);
  }
  if (last_fed_with_nodes.has_value()) {
    result.last_checkpoint_node = graph.session(last_fed_with_nodes->id).node_ids.back();
    result.last_checkpoint_global_pose = graph.node(*result.last_checkpoint_node).global_pose;
  }
  manifest_ = std::move(loaded_manifest);
  boot_count_ = manifest.boot_count();
  return result;
}

}  // namespace evergreenslam::lifelong
