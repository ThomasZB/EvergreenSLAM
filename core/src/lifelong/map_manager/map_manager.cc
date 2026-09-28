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
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "common/file.h"
#include "lifelong/map_manager/proto_conversion.h"

namespace evergreenslam::lifelong {
namespace {

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

std::string SessionStem(SessionId id) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "session_%06d", id.session_index);
  return std::string(buffer);
}

MapManager::LoadFailure Failure(MapManager::LoadFailure::Reason reason, std::string detail) {
  LOG(ERROR) << detail;
  return MapManager::LoadFailure{reason, std::move(detail)};
}

const AnchorStore& EmptyAnchorStore() {
  static const AnchorStore empty;
  return empty;
}

}  // namespace

MapManager::MapManager(std::string directory)
    : MapManager(std::move(directory), EmptyAnchorStore()) {}

MapManager::MapManager(std::string directory, const AnchorStore& anchors)
    : files_(std::move(directory)), anchors_(anchors) {}

const char* ToString(MapManager::LoadFailure::Reason reason) {
  switch (reason) {
    case MapManager::LoadFailure::Reason::LOST_MANIFEST:
      return "lost manifest";
    case MapManager::LoadFailure::Reason::UNREADABLE_MANIFEST:
      return "unreadable manifest";
    case MapManager::LoadFailure::Reason::UNSUPPORTED_VERSION:
      return "unsupported version";
    case MapManager::LoadFailure::Reason::MISSING_SESSION_FILE:
      return "missing session file";
    case MapManager::LoadFailure::Reason::CORRUPT_SESSION_FILE:
      return "corrupt session file";
  }
  return "unknown";
}

std::string MapManager::LastPoseFileName() { return kLastPoseFileName; }

std::optional<std::string> MapManager::FileNameOf(SessionId id) const {
  const auto it = manifest_.find(id);
  if (it == manifest_.end()) {
    return std::nullopt;
  }
  return it->second.file_name;
}

std::optional<std::string> MapManager::anchors_file_name() const {
  if (anchors_file_name_.empty()) {
    return std::nullopt;
  }
  return anchors_file_name_;
}

void MapManager::OnSessionFrozen(const PoseGraphData& graph, SessionId id) {
  CHECK(graph.session(id).frozen()) << "OnSessionFrozen runs after the freeze has landed";
  const auto it = manifest_.find(id);
  if (it != manifest_.end() && it->second.state == SessionState::FROZEN) {
    // A commit between the freeze and this callback already healed it in; frozen is write-once.
    return;
  }
  std::vector<SessionWrite> writes = {SessionWrite{id, /*frozen=*/true, /*fed=*/false}};
  // Migrated cross-session constraints would otherwise live only in files never loaded again.
  for (const SessionWrite& write : UnfrozenSessionWrites(graph)) {
    writes.push_back(write);
  }
  CommitSessions(graph, writes, BeginCommit());
}

void MapManager::OnSessionStarted(const PoseGraphData& graph, SessionId id) {
  CHECK(!graph.session(id).frozen());
  CommitSessions(graph, {SessionWrite{id, /*frozen=*/false, /*fed=*/true}}, BeginCommit());
}

bool MapManager::Checkpoint(const PoseGraphData& graph, SessionId id) {
  CHECK(!graph.session(id).frozen()) << "a frozen session's file is write-once";
  if (!CommitSessions(graph, {SessionWrite{id, /*frozen=*/false, /*fed=*/true}}, BeginCommit())) {
    return false;
  }
  ++num_checkpoints_written_;
  return true;
}

bool MapManager::Commit(const PoseGraphData& graph) {
  return CommitSessions(graph, {}, BeginCommit());
}

void MapManager::WriteLastPose(const Node& node) {
  proto::LastPose last_pose;
  *last_pose.mutable_node_id() = ToProto(node.id);
  last_pose.set_time_nanos(TimeToProto(node.constant_data.time));
  *last_pose.mutable_global_pose() = ToProto(node.global_pose);
  std::string contents;
  CHECK(last_pose.SerializeToString(&contents));
  common::WriteFileAtomically(files_.PathOf(kLastPoseFileName), contents);
}

std::optional<MapManager::LastPose> MapManager::ReadLastPose() const {
  const std::optional<std::string> contents = common::ReadFile(files_.PathOf(kLastPoseFileName));
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

std::optional<AnchorTable> MapManager::ReadAnchors() {
  anchors_written_revision_ = 0;
  if (anchors_file_name_.empty()) {
    return AnchorTable{};
  }
  const std::string path = files_.PathOf(anchors_file_name_);
  const std::optional<std::string> contents = files_.Read(anchors_file_name_);
  proto::AnchorFile file;
  if (!contents.has_value() || !file.ParseFromString(*contents)) {
    LOG(ERROR) << path << " is not a readable anchor file";
    return std::nullopt;
  }
  if (file.version() != kAnchorFormatVersion) {
    LOG(ERROR) << path << " has version " << file.version() << ", expected "
               << kAnchorFormatVersion;
    return std::nullopt;
  }
  AnchorTable table;
  table.next_id = file.next_id();
  for (const proto::Anchor& anchor : file.anchors()) {
    if (!proto::Anchor::State_IsValid(anchor.state()) ||
        !proto::Anchor::OrphanReason_IsValid(anchor.orphan_reason())) {
      LOG(ERROR) << path << " holds anchor " << anchor.id() << " in an unknown state";
      return std::nullopt;
    }
    table.anchors.push_back(FromProto(anchor));
  }
  return table;
}

std::optional<int> MapManager::RecordBoot() {
  Staged staged = BeginCommit();
  ++staged.boot_count;
  if (!EndCommit(staged)) {
    return std::nullopt;
  }
  return boot_count_;
}

bool MapManager::RemoveSession(const PoseGraphData& graph, SessionId id) {
  CHECK(!graph.HasSession(id)) << "the graph drops the session before its files go";
  return CommitSessions(graph, UnfrozenSessionWrites(graph), BeginCommit());
}

std::vector<MapManager::SessionWrite> MapManager::UnfrozenSessionWrites(
    const PoseGraphData& graph) const {
  std::vector<SessionWrite> writes;
  for (const auto& [id, session] : graph.sessions()) {
    if (!session.frozen()) {
      writes.push_back(SessionWrite{id, /*frozen=*/false, /*fed=*/false});
    }
  }
  return writes;
}

MapManager::Staged MapManager::BeginCommit() const {
  Staged staged;
  staged.files = files_.Begin();
  staged.boot_count = boot_count_;
  staged.manifest = manifest_;
  staged.anchors_file_name = anchors_file_name_;
  return staged;
}

// Reconciles the manifest with the graph first, so a commit that failed earlier (a freeze, a
// removal) lands with this one instead of being lost.
bool MapManager::CommitSessions(const PoseGraphData& graph, const std::vector<SessionWrite>& writes,
                                Staged staged) {
  bool healed = false;
  for (auto it = staged.manifest.begin(); it != staged.manifest.end();) {
    if (graph.HasSession(it->first)) {
      ++it;
      continue;
    }
    staged.files.superseded.push_back(it->second.file_name);
    it = staged.manifest.erase(it);
    healed = true;
  }
  std::vector<SessionWrite> all = writes;
  const auto planned = [&all](SessionId id) {
    return std::any_of(all.begin(), all.end(),
                       [id](const SessionWrite& write) { return write.id == id; });
  };
  for (const auto& [id, session] : graph.sessions()) {
    const auto entry = staged.manifest.find(id);
    if (session.frozen() && !planned(id) &&
        (entry == staged.manifest.end() || entry->second.state != SessionState::FROZEN)) {
      all.push_back(SessionWrite{id, /*frozen=*/true, /*fed=*/false});
      healed = true;
    }
  }
  if (healed) {
    for (const SessionWrite& write : UnfrozenSessionWrites(graph)) {
      if (!planned(write.id)) {
        all.push_back(write);
      }
    }
  }

  for (const SessionWrite& write : all) {
    if (!StageSessionFile(graph, write, staged)) {
      files_.Abandon(staged.files);
      return false;
    }
  }
  if (!StageAnchorsFile(staged)) {
    files_.Abandon(staged.files);
    return false;
  }
  return EndCommit(staged);
}

bool MapManager::StageSessionFile(const PoseGraphData& graph, const SessionWrite& write,
                                  Staged& staged) {
  const SessionData& session = graph.session(write.id);
  proto::SessionFile file;
  file.set_version(kMapFormatVersion);
  *file.mutable_session() = ToProto(session);
  file.mutable_session()->set_next_submap_index(graph.id_allocator().next_submap_index(write.id));
  file.mutable_session()->set_next_node_index(graph.id_allocator().next_node_index(write.id));

  for (const SubmapId& submap_id : session.submap_ids) {
    *file.add_submaps() = ToProto(graph.submap(submap_id));
  }
  for (const NodeId& node_id : session.node_ids) {
    *file.add_nodes() = ToProto(graph.node(node_id), graph.ContainingSubmapIds(node_id));
  }
  for (const Constraint& constraint : ConstraintsOwnedBy(graph, write.id, write.frozen)) {
    *file.add_constraints() = ToProto(constraint);
  }

  std::string contents;
  CHECK(file.SerializeToString(&contents));
  const std::optional<std::string> file_name =
      files_.Write(staged.files, SessionStem(write.id), contents);
  if (!file_name.has_value()) {
    return false;
  }

  Entry& entry = staged.manifest[write.id];
  if (!entry.file_name.empty()) {
    staged.files.superseded.push_back(entry.file_name);
  }
  entry.file_name = *file_name;
  entry.state = session.state;
  if (write.frozen) {
    ++staged.num_frozen_files;
  }
  if (write.fed) {
    entry.last_fed_boot = staged.boot_count;
  }
  return true;
}

bool MapManager::StageAnchorsFile(Staged& staged) {
  const int64_t revision = anchors_.revision();
  if (revision == anchors_written_revision_) {
    return true;
  }
  const AnchorTable table = anchors_.Table();
  proto::AnchorFile file;
  file.set_version(kAnchorFormatVersion);
  file.set_next_id(table.next_id);
  for (const Anchor& anchor : table.anchors) {
    *file.add_anchors() = ToProto(anchor);
  }
  std::string contents;
  CHECK(file.SerializeToString(&contents));
  const std::optional<std::string> file_name = files_.Write(staged.files, "anchors", contents);
  if (!file_name.has_value()) {
    return false;
  }
  if (!staged.anchors_file_name.empty()) {
    staged.files.superseded.push_back(staged.anchors_file_name);
  }
  staged.anchors_file_name = *file_name;
  staged.anchors_revision = revision;
  return true;
}

bool MapManager::EndCommit(Staged& staged) {
  if (!files_.Publish(staged.files, SerializeManifest(staged))) {
    return false;
  }
  manifest_ = std::move(staged.manifest);
  anchors_file_name_ = std::move(staged.anchors_file_name);
  boot_count_ = staged.boot_count;
  num_frozen_files_written_ += staged.num_frozen_files;
  if (staged.anchors_revision.has_value()) {
    anchors_written_revision_ = *staged.anchors_revision;
    ++num_anchor_writes_;
  }
  return true;
}

std::string MapManager::SerializeManifest(const Staged& staged) const {
  proto::Manifest manifest;
  manifest.set_version(kMapFormatVersion);
  manifest.set_boot_count(staged.boot_count);
  manifest.set_generation(staged.files.generation);
  manifest.set_anchors_file_name(staged.anchors_file_name);
  for (const auto& [id, entry] : staged.manifest) {
    proto::ManifestEntry* proto_entry = manifest.add_sessions();
    proto_entry->set_session_index(id.session_index);
    proto_entry->set_state(entry.state == SessionState::FROZEN ? proto::FROZEN : proto::ACTIVE);
    proto_entry->set_file_name(entry.file_name);
    proto_entry->set_last_fed_boot(entry.last_fed_boot);
  }
  std::string contents;
  CHECK(manifest.SerializeToString(&contents));
  return contents;
}

std::optional<MapManager::FileSummary> MapManager::InspectSessionFile(SessionId id) const {
  const std::optional<std::string> file_name = FileNameOf(id);
  if (!file_name.has_value()) {
    return std::nullopt;
  }
  const std::optional<std::string> contents = files_.Read(*file_name);
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

MapManager::LoadOutcome MapManager::Load(PoseGraphData& graph) {
  using Reason = LoadFailure::Reason;
  CHECK(graph.sessions().empty()) << "loading into a graph that already holds sessions";
  const std::string& directory = files_.directory();

  const std::optional<std::string> manifest_contents = files_.ReadManifest();
  if (!manifest_contents.has_value()) {
    if (files_.HasCommitFiles()) {
      return Failure(Reason::LOST_MANIFEST,
                     "no manifest in " + directory + ", but session or anchor files are there");
    }
    LOG(INFO) << "no manifest in " << directory << ", starting from an empty map";
    return FreshDirectory{};
  }
  proto::Manifest manifest;
  if (!manifest.ParseFromString(*manifest_contents)) {
    return Failure(Reason::UNREADABLE_MANIFEST,
                   "manifest in " + directory + " is not a readable map manifest");
  }
  if (!VersionSupported(manifest.version())) {
    return Failure(Reason::UNSUPPORTED_VERSION, "manifest in " + directory + " has version " +
                                                    std::to_string(manifest.version()) + ", not " +
                                                    std::to_string(kMapFormatVersion) +
                                                    "; older maps are refused, not migrated");
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
    const std::optional<std::string> contents = files_.Read(entry.file_name());
    if (!contents.has_value()) {
      return Failure(Reason::MISSING_SESSION_FILE,
                     "session file " + entry.file_name() + " listed in the manifest is missing");
    }
    proto::SessionFile file;
    if (!file.ParseFromString(*contents)) {
      return Failure(Reason::CORRUPT_SESSION_FILE,
                     "session file " + entry.file_name() + " is corrupt");
    }
    if (!VersionSupported(file.version())) {
      return Failure(
          Reason::UNSUPPORTED_VERSION,
          "session file " + entry.file_name() + " has version " + std::to_string(file.version()));
    }
    if (file.session().session_index() != id.session_index) {
      return Failure(Reason::CORRUPT_SESSION_FILE,
                     "session file " + entry.file_name() + " holds session " +
                         std::to_string(file.session().session_index()) + ", manifest says " +
                         std::to_string(id.session_index));
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
  anchors_file_name_ = manifest.anchors_file_name();
  boot_count_ = manifest.boot_count();
  files_.set_generation(manifest.generation());
  return result;
}

void MapManager::RemoveUnreferencedFiles() {
  std::set<std::string> referenced;
  for (const auto& [id, entry] : manifest_) {
    referenced.insert(entry.file_name);
  }
  if (!anchors_file_name_.empty()) {
    referenced.insert(anchors_file_name_);
  }
  files_.RemoveUnreferenced(referenced);
}

}  // namespace evergreenslam::lifelong
