/**
 * @file map_manager_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Round trip, frozen file immutability, atomic checkpoints, version and corruption.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/map_manager.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph_data.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/submap.h"
#include "sensor/point_cloud.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;

std::string MakeTempDir(const std::string& name) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("evergreenslam_map_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path.string();
}

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Off the resolution lattice on purpose.
sensor::PointCloud RingCloud(double radius) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 120; ++i) {
    const double angle = 2.0 * M_PI * i / 120.0 + 0.013;
    cloud.push_back(sensor::Point2d{
        Eigen::Vector2d(radius * std::cos(angle) + 0.017, radius * std::sin(angle) - 0.023)});
  }
  return cloud;
}

std::shared_ptr<mapping::Submap> MakeSubmap(const SubmapId& id, const Eigen::Affine2d& local_pose,
                                            double radius, bool finish) {
  auto submap = std::make_shared<mapping::Submap>(id.submap_index, local_pose, kResolution);
  const mapping::CastRaysMapping inserter;
  const sensor::PointCloud cloud = RingCloud(radius);
  submap->InsertScan(local_pose, cloud, inserter);
  if (finish) {
    submap->Finish();
  }
  return submap;
}

Eigen::Matrix3d MakeInformation(double seed) {
  Eigen::Matrix3d matrix = OdometrySqrtInformation(ConstraintWeightOption());
  matrix(0, 1) = seed;
  matrix(2, 0) = -seed * 3.25;
  return matrix;
}

// Three doubles cannot round trip a rotation matrix bitwise; what is exact is the [x, y, theta]
// projection, so the loaded pose must equal that coefficient for coefficient.
void ExpectPoseRestored(const Eigen::Affine2d& original, const Eigen::Affine2d& loaded) {
  const Eigen::Affine2d expected = transform::FromArray3(transform::ToArray3(original));
  EXPECT_EQ(expected.translation().x(), loaded.translation().x());
  EXPECT_EQ(expected.translation().y(), loaded.translation().y());
  EXPECT_EQ(original.translation().x(), loaded.translation().x());
  EXPECT_EQ(original.translation().y(), loaded.translation().y());
  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      EXPECT_EQ(expected.linear()(row, col), loaded.linear()(row, col));
    }
  }
  EXPECT_NEAR(transform::GetYaw(original), transform::GetYaw(loaded), 1e-15);
}

void ExpectGridRestored(const mapping::GridMapu8& original, const mapping::GridMapu8& loaded) {
  ASSERT_EQ(original.width(), loaded.width());
  ASSERT_EQ(original.height(), loaded.height());
  EXPECT_EQ(original.resolution(), loaded.resolution());
  EXPECT_EQ(original.origin_x(), loaded.origin_x());
  EXPECT_EQ(original.origin_y(), loaded.origin_y());
  ASSERT_EQ(original.data().size(), loaded.data().size());
  EXPECT_TRUE(original.data() == loaded.data()) << "grid bytes differ";
}

void ExpectCloudRestored(const sensor::PointCloud& original, const sensor::PointCloud& loaded) {
  ASSERT_EQ(original.size(), loaded.size());
  for (size_t i = 0; i < original.size(); ++i) {
    EXPECT_EQ(original[i].point.x(), loaded[i].point.x());
    EXPECT_EQ(original[i].point.y(), loaded[i].point.y());
  }
}

struct BuiltGraph {
  PoseGraphData graph;
  SessionId frozen_session;
  SessionId active_session;
  int num_frozen_internal_constraints = 0;
};

void FillSession(PoseGraphData& graph, SessionId session, int node_offset, double x_offset) {
  for (int k = 0; k < 2; ++k) {
    SubmapRecord record;
    record.id = graph.AllocateSubmapId(session);
    record.local_pose = transform::FromXYTheta(x_offset + 1.37 * k, 0.71 * k, 0.23 * k + 0.11);
    record.global_pose = transform::FromXYTheta(x_offset + 1.41 * k, 0.73 * k, 0.19 * k + 0.07);
    record.submap = MakeSubmap(record.id, record.local_pose, 2.13 + 0.31 * k, k == 0);
    graph.AddSubmap(record);
  }
  const std::vector<SubmapId>& submap_ids = graph.session(session).submap_ids;
  for (int k = 0; k < 3; ++k) {
    Node node;
    node.id = graph.AllocateNodeId(session);
    node.constant_data.time = TestTime(node_offset + k);
    node.constant_data.local_pose =
        transform::FromXYTheta(x_offset + 0.53 * k, 0.29 * k, 0.07 * k - 0.31);
    node.constant_data.point_cloud = RingCloud(1.87 + 0.13 * k);
    node.global_pose = transform::FromXYTheta(x_offset + 0.57 * k, 0.31 * k, 0.09 * k - 0.29);
    const std::vector<SubmapId> containing =
        k == 1 ? submap_ids : std::vector<SubmapId>{submap_ids[k == 0 ? 0 : 1]};
    graph.AddNode(node, containing);
  }
}

BuiltGraph BuildGraph() {
  BuiltGraph built;
  built.frozen_session = built.graph.StartNewSession(TestTime(0));
  FillSession(built.graph, built.frozen_session, 0, 0.0);

  const SessionData& first = built.graph.session(built.frozen_session);
  for (const NodeId& node_id : first.node_ids) {
    Constraint constraint;
    constraint.type = Constraint::Type::INTRA_SUBMAP;
    constraint.from = VariableId::Of(first.submap_ids.front());
    constraint.to = VariableId::Of(node_id);
    constraint.relative_pose = transform::FromXYTheta(0.31, -0.17, 0.041);
    constraint.sqrt_information = MakeInformation(1.5 + node_id.node_index);
    built.graph.AddConstraint(constraint);
    ++built.num_frozen_internal_constraints;
  }
  {
    // A long tree edge becomes a unary absolute prior, so `to` is empty.
    Constraint prior;
    prior.type = Constraint::Type::PRIOR;
    prior.from = VariableId::Of(first.node_ids.back());
    prior.relative_pose = transform::FromXYTheta(2.71, 3.14, -1.23);
    prior.sqrt_information = MakeInformation(7.75);
    built.graph.AddConstraint(prior);
    ++built.num_frozen_internal_constraints;
  }
  built.graph.FreezeSession(built.frozen_session);

  built.active_session =
      built.graph.StartNewSession(TestTime(10), transform::FromXYTheta(9.13, 4.27, 0.61));
  FillSession(built.graph, built.active_session, 10, 9.0);
  const SessionData& second = built.graph.session(built.active_session);
  {
    Constraint intra;
    intra.type = Constraint::Type::INTRA_SUBMAP;
    intra.from = VariableId::Of(second.submap_ids.front());
    intra.to = VariableId::Of(second.node_ids.front());
    intra.relative_pose = transform::FromXYTheta(-0.19, 0.83, 0.29);
    intra.sqrt_information = MakeInformation(2.5);
    built.graph.AddConstraint(intra);

    Constraint inter;
    inter.type = Constraint::Type::INTER_SUBMAP;
    inter.from = VariableId::Of(first.submap_ids.back());
    inter.to = VariableId::Of(second.node_ids.back());
    inter.relative_pose = transform::FromXYTheta(4.51, -2.09, 1.71);
    inter.sqrt_information = MakeInformation(3.25);
    inter.recovered = true;
    built.graph.AddConstraint(inter);

    Constraint prior;
    prior.type = Constraint::Type::PRIOR;
    prior.from = VariableId::Of(second.submap_ids.back());
    prior.relative_pose = transform::FromXYTheta(-5.37, 1.09, 2.81);
    prior.sqrt_information = MakeInformation(9.5);
    built.graph.AddConstraint(prior);
  }
  return built;
}

void SaveWholeGraph(MapManager& manager, const BuiltGraph& built) {
  manager.OnSessionFrozen(built.graph, built.frozen_session);
  manager.Checkpoint(built.graph, built.active_session);
}

void ExpectSessionRestored(const PoseGraphData& original, const PoseGraphData& loaded,
                           SessionId id) {
  const SessionData& before = original.session(id);
  const SessionData& after = loaded.session(id);
  EXPECT_EQ(before.state, after.state);
  EXPECT_EQ(common::ToUnixNanos(before.start_time), common::ToUnixNanos(after.start_time));
  // Exact even when the newest node has been trimmed: RestoreSession carries the timeline.
  EXPECT_EQ(common::ToUnixNanos(before.last_node_time), common::ToUnixNanos(after.last_node_time));
  ExpectPoseRestored(before.local_to_global, after.local_to_global);
  ASSERT_EQ(before.submap_ids, after.submap_ids);
  ASSERT_EQ(before.node_ids, after.node_ids);

  for (const SubmapId& submap_id : before.submap_ids) {
    const SubmapRecord& lhs = original.submap(submap_id);
    const SubmapRecord& rhs = loaded.submap(submap_id);
    ExpectPoseRestored(lhs.local_pose, rhs.local_pose);
    ExpectPoseRestored(lhs.global_pose, rhs.global_pose);
    EXPECT_EQ(lhs.node_ids, rhs.node_ids);
    ASSERT_TRUE(rhs.submap != nullptr);
    EXPECT_EQ(lhs.submap->num_scans(), rhs.submap->num_scans());
    EXPECT_EQ(lhs.submap->finished(), rhs.submap->finished());
    ExpectGridRestored(lhs.submap->Snapshot(), rhs.submap->Snapshot());
  }
  for (const NodeId& node_id : before.node_ids) {
    const Node& lhs = original.node(node_id);
    const Node& rhs = loaded.node(node_id);
    EXPECT_EQ(common::ToUnixNanos(lhs.constant_data.time),
              common::ToUnixNanos(rhs.constant_data.time));
    ExpectPoseRestored(lhs.constant_data.local_pose, rhs.constant_data.local_pose);
    ExpectPoseRestored(lhs.global_pose, rhs.global_pose);
    ExpectCloudRestored(lhs.constant_data.point_cloud, rhs.constant_data.point_cloud);
    EXPECT_EQ(original.ContainingSubmapIds(node_id), loaded.ContainingSubmapIds(node_id));
  }
}

std::string ReadWhole(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

TEST(MapManagerTest, RoundTripsAWholeGraph) {
  const std::string directory = MakeTempDir("roundtrip");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_sessions, 2);
  EXPECT_EQ(result->num_frozen_sessions, 1);
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  EXPECT_EQ(result->unfrozen_sessions.front().id, built.active_session);
  ASSERT_TRUE(result->last_checkpoint_node.has_value());
  EXPECT_EQ(*result->last_checkpoint_node,
            built.graph.session(built.active_session).node_ids.back());
  ASSERT_TRUE(result->last_checkpoint_global_pose.has_value());
  ExpectPoseRestored(built.graph.node(*result->last_checkpoint_node).global_pose,
                     *result->last_checkpoint_global_pose);

  ExpectSessionRestored(built.graph, loaded, built.frozen_session);
  ExpectSessionRestored(built.graph, loaded, built.active_session);
  for (const SessionId id : {built.frozen_session, built.active_session}) {
    EXPECT_EQ(common::ToUnixNanos(built.graph.session(id).last_node_time),
              common::ToUnixNanos(loaded.session(id).last_node_time));
  }

  // The frozen session's own constraints are in its file and deliberately not in memory.
  const std::optional<MapManager::FileSummary> summary =
      reader.InspectSessionFile(built.frozen_session);
  ASSERT_TRUE(summary.has_value());
  EXPECT_TRUE(summary->frozen);
  EXPECT_EQ(summary->num_constraints, built.num_frozen_internal_constraints);
  for (const Constraint& constraint : loaded.constraints()) {
    EXPECT_FALSE(loaded.session(constraint.from.session()).frozen() &&
                 (!constraint.to.has_value() || loaded.session(constraint.to->session()).frozen()))
        << "a constraint internal to a frozen session must not be loaded";
  }
}

TEST(MapManagerTest, RestoresEveryConstraintOfTheActiveSessionBitwise) {
  const std::string directory = MakeTempDir("constraints");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  PoseGraphData loaded;
  MapManager reader(directory);
  ASSERT_TRUE(reader.Load(loaded).has_value());

  std::vector<Constraint> expected;
  for (const Constraint& constraint : built.graph.constraints()) {
    const bool internal_to_frozen =
        built.graph.session(constraint.from.session()).frozen() &&
        (!constraint.to.has_value() || built.graph.session(constraint.to->session()).frozen());
    if (!internal_to_frozen) {
      expected.push_back(constraint);
    }
  }
  ASSERT_EQ(expected.size(), loaded.constraints().size());
  for (size_t i = 0; i < expected.size(); ++i) {
    const Constraint& lhs = expected[i];
    const Constraint& rhs = loaded.constraints()[i];
    EXPECT_EQ(lhs.type, rhs.type);
    EXPECT_TRUE(lhs.from == rhs.from);
    ASSERT_EQ(lhs.to.has_value(), rhs.to.has_value());
    if (lhs.to.has_value()) {
      EXPECT_TRUE(*lhs.to == *rhs.to);
    }
    ExpectPoseRestored(lhs.relative_pose, rhs.relative_pose);
    EXPECT_EQ(lhs.recovered, rhs.recovered);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        EXPECT_EQ(lhs.sqrt_information(row, col), rhs.sqrt_information(row, col));
      }
    }
  }
  EXPECT_TRUE(std::any_of(expected.begin(), expected.end(), [](const Constraint& constraint) {
    return constraint.type == Constraint::Type::PRIOR && !constraint.to.has_value();
  })) << "the round trip has to exercise a unary prior";
}

TEST(MapManagerTest, FrozenFileIsNeverRewritten) {
  const std::string directory = MakeTempDir("immutable");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  manager.OnSessionFrozen(built.graph, built.frozen_session);

  const std::string frozen_path =
      (std::filesystem::path(directory) / MapManager::SessionFileName(built.frozen_session))
          .string();
  const std::string before = ReadWhole(frozen_path);
  ASSERT_FALSE(before.empty());

  for (int i = 0; i < 5; ++i) {
    // Whatever else happens to the graph, the frozen file has to stay byte for byte the same.
    built.graph.SetSubmapGlobalPose(built.graph.session(built.active_session).submap_ids.front(),
                                    transform::FromXYTheta(0.5 * i, 0.25 * i, 0.05 * i));
    manager.Checkpoint(built.graph, built.active_session);
  }
  EXPECT_EQ(before, ReadWhole(frozen_path));
  EXPECT_EQ(manager.num_checkpoints_written(), 5);
}

TEST(MapManagerTest, CheckpointInterruptedBeforeRenameLeavesThePreviousOneLoadable) {
  const std::string directory = MakeTempDir("atomic");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  const std::filesystem::path active_path =
      std::filesystem::path(directory) / MapManager::SessionFileName(built.active_session);
  const std::string good = ReadWhole(active_path.string());

  // A kill between the write and the rename leaves exactly this behind.
  {
    std::ofstream stream(active_path.string() + ".tmp", std::ios::binary | std::ios::trunc);
    stream << "half a session file, truncated by a power cut";
  }

  PoseGraphData loaded;
  MapManager reader(directory);
  ASSERT_TRUE(reader.Load(loaded).has_value());
  ExpectSessionRestored(built.graph, loaded, built.active_session);
  EXPECT_EQ(good, ReadWhole(active_path.string()));
}

TEST(MapManagerTest, RefusesAFileFromAnOlderFormatVersion) {
  const std::string directory = MakeTempDir("version");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  // Field 1 varint kMapFormatVersion leads every file we write; patching the value back is what a
  // previous-version writer would have produced. Older formats are refused whole.
  const std::string path =
      (std::filesystem::path(directory) / MapManager::SessionFileName(built.active_session))
          .string();
  std::string contents = ReadWhole(path);
  ASSERT_GE(contents.size(), 2u);
  ASSERT_EQ(static_cast<uint8_t>(contents[0]), 0x08);
  ASSERT_EQ(static_cast<uint8_t>(contents[1]), static_cast<uint8_t>(kMapFormatVersion));
  contents[1] = static_cast<char>(kMapFormatVersion - 1);
  {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  }

  PoseGraphData loaded;
  MapManager reader(directory);
  EXPECT_FALSE(reader.Load(loaded).has_value());
  EXPECT_EQ(reader.InspectSessionFile(built.active_session)->version, kMapFormatVersion - 1);
}

TEST(MapManagerTest, RefusesACorruptOrMissingMapWithoutCrashing) {
  const std::string empty_directory = MakeTempDir("empty");
  PoseGraphData nothing;
  MapManager reader(empty_directory);
  EXPECT_FALSE(reader.Load(nothing).has_value());
  EXPECT_TRUE(nothing.sessions().empty());

  const std::string directory = MakeTempDir("corrupt");
  BuiltGraph built = BuildGraph();
  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  {
    std::ofstream stream(
        (std::filesystem::path(directory) / MapManager::SessionFileName(built.active_session))
            .string(),
        std::ios::binary | std::ios::trunc);
    const std::string garbage(512, '\xff');
    stream.write(garbage.data(), static_cast<std::streamsize>(garbage.size()));
  }
  PoseGraphData loaded;
  MapManager corrupt_reader(directory);
  EXPECT_FALSE(corrupt_reader.Load(loaded).has_value());

  const std::string manifest_path = (std::filesystem::path(directory) / "manifest.pb").string();
  {
    std::ofstream stream(manifest_path, std::ios::binary | std::ios::trunc);
    const std::string garbage(64, '\xfe');
    stream.write(garbage.data(), static_cast<std::streamsize>(garbage.size()));
  }
  PoseGraphData other;
  MapManager broken_manifest(directory);
  EXPECT_FALSE(broken_manifest.Load(other).has_value());
}

TEST(MapManagerTest, RoundTripsAGraphThatHasBeenTrimmed) {
  const std::string directory = MakeTempDir("trimmed");
  BuiltGraph built = BuildGraph();

  // The file is the only place the retired indices can come from: the surviving records reach 0
  // and 1, the allocator is at 2 and 3.
  TrimRequest request;
  request.deleted_submap_ids.push_back(built.graph.session(built.active_session).submap_ids.back());
  request.successors[request.deleted_submap_ids.front()] =
      built.graph.session(built.active_session).submap_ids.front();
  built.graph.ApplyTrim(request);

  MapManager manager(directory);
  SaveWholeGraph(manager, built);

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  ExpectSessionRestored(built.graph, loaded, built.active_session);

  // The per-session watermarks ride the LoadResult too, one entry per unfrozen session.
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  EXPECT_EQ(result->unfrozen_sessions.front().next_submap_index, 2);
  EXPECT_EQ(result->unfrozen_sessions.front().next_node_index, 3);

  // Trimming leaves holes in the index space; nothing may reissue an index that ever existed.
  const SubmapId next = loaded.AllocateSubmapId(built.active_session);
  EXPECT_EQ(next.submap_index, 2);
  const NodeId next_node = loaded.AllocateNodeId(built.active_session);
  EXPECT_EQ(next_node.node_index, 3);
}

SessionId AddFedSession(BuiltGraph& built) {
  const SessionId fed =
      built.graph.StartNewSession(TestTime(20), transform::FromXYTheta(18.41, 2.07, -0.53));
  FillSession(built.graph, fed, 20, 18.0);

  Constraint cross;
  cross.type = Constraint::Type::INTER_SUBMAP;
  cross.from = VariableId::Of(built.graph.session(built.active_session).submap_ids.back());
  cross.to = VariableId::Of(built.graph.session(fed).node_ids.front());
  cross.relative_pose = transform::FromXYTheta(1.03, -0.61, 0.47);
  cross.sqrt_information = MakeInformation(4.75);
  built.graph.AddConstraint(cross);
  return fed;
}

int CountNonFullyFrozenConstraints(const PoseGraphData& graph) {
  int count = 0;
  for (const Constraint& constraint : graph.constraints()) {
    const bool fully_frozen =
        graph.session(constraint.from.session()).frozen() &&
        (!constraint.to.has_value() || graph.session(constraint.to->session()).frozen());
    if (!fully_frozen) {
      ++count;
    }
  }
  return count;
}

TEST(MapManagerTest, MultipleUnfrozenSessionsRoundTripWithoutOverwritingEachOther) {
  const std::string directory = MakeTempDir("multi");
  BuiltGraph built = BuildGraph();
  const SessionId fed = AddFedSession(built);

  MapManager manager(directory);
  manager.OnSessionFrozen(built.graph, built.frozen_session);
  manager.Checkpoint(built.graph, fed);

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_sessions, 3);
  EXPECT_EQ(result->num_frozen_sessions, 1);
  ASSERT_EQ(result->unfrozen_sessions.size(), 2u);
  EXPECT_EQ(result->unfrozen_sessions[0].id, built.active_session);
  EXPECT_EQ(result->unfrozen_sessions[1].id, fed);
  for (const MapManager::LoadResult::UnfrozenSession& unfrozen : result->unfrozen_sessions) {
    EXPECT_EQ(unfrozen.next_submap_index, 2);
    EXPECT_EQ(unfrozen.next_node_index, 3);
  }
  ExpectSessionRestored(built.graph, loaded, built.frozen_session);
  ExpectSessionRestored(built.graph, loaded, built.active_session);
  ExpectSessionRestored(built.graph, loaded, fed);

  // Both unfrozen sessions were fed in boot 0, so the tie goes to the higher index.
  ASSERT_TRUE(result->last_checkpoint_node.has_value());
  EXPECT_EQ(*result->last_checkpoint_node, built.graph.session(fed).node_ids.back());

  // The union of the unfrozen files is every non-fully-frozen constraint, each exactly once.
  EXPECT_EQ(static_cast<int>(loaded.constraints().size()),
            CountNonFullyFrozenConstraints(built.graph));
}

TEST(MapManagerTest, ConstraintOwnershipSplitsAcrossUnfrozenFilesWithoutDuplication) {
  const std::string directory = MakeTempDir("ownership");
  BuiltGraph built = BuildGraph();
  const SessionId fed = AddFedSession(built);

  MapManager manager(directory);
  manager.OnSessionFrozen(built.graph, built.frozen_session);
  manager.Checkpoint(built.graph, fed);

  // A constraint goes to the file of its highest-indexed unfrozen endpoint.
  const std::optional<MapManager::FileSummary> floating_file =
      manager.InspectSessionFile(built.active_session);
  const std::optional<MapManager::FileSummary> fed_file = manager.InspectSessionFile(fed);
  ASSERT_TRUE(floating_file.has_value());
  ASSERT_TRUE(fed_file.has_value());
  EXPECT_EQ(floating_file->num_constraints, 3);
  EXPECT_EQ(fed_file->num_constraints, 1);
  EXPECT_EQ(floating_file->num_constraints + fed_file->num_constraints,
            CountNonFullyFrozenConstraints(built.graph));
}

TEST(MapManagerTest, AConstraintMigratesToItsNextUnfrozenEndpointWhenItsOwnerFreezes) {
  const std::string directory = MakeTempDir("migration");
  BuiltGraph built = BuildGraph();
  const SessionId fed = AddFedSession(built);

  MapManager manager(directory);
  manager.OnSessionFrozen(built.graph, built.frozen_session);
  manager.Checkpoint(built.graph, fed);
  ASSERT_EQ(manager.InspectSessionFile(built.active_session)->num_constraints, 3);

  // The active-to-fed constraint is not fully frozen, so it may not vanish into the write-once
  // frozen file: ownership migrates to the floating session.
  built.graph.FreezeSession(fed);
  manager.OnSessionFrozen(built.graph, fed);
  EXPECT_EQ(manager.InspectSessionFile(built.active_session)->num_constraints, 4);

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_frozen_sessions, 2);
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  EXPECT_EQ(result->unfrozen_sessions.front().id, built.active_session);
  const int cross_count =
      static_cast<int>(std::count_if(loaded.constraints().begin(), loaded.constraints().end(),
                                     [&built, &fed](const Constraint& constraint) {
                                       return constraint.from.session() == built.active_session &&
                                              constraint.to.has_value() &&
                                              constraint.to->session() == fed;
                                     }));
  EXPECT_EQ(cross_count, 1) << "the migrated constraint has to come back exactly once";
}

TEST(MapManagerTest, BootCountIncrementsAcrossBootsAndStampsTheFedSession) {
  const std::string directory = MakeTempDir("boot");
  BuiltGraph built = BuildGraph();
  {
    MapManager manager(directory);
    EXPECT_EQ(manager.boot_count(), 0);
    EXPECT_EQ(manager.RecordBoot(), 1);
    manager.OnSessionFrozen(built.graph, built.frozen_session);
    manager.Checkpoint(built.graph, built.active_session);
  }
  {
    PoseGraphData loaded;
    MapManager manager(directory);
    const std::optional<MapManager::LoadResult> result = manager.Load(loaded);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(manager.boot_count(), 1);
    ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
    EXPECT_EQ(result->unfrozen_sessions.front().last_fed_boot, 1);
    EXPECT_EQ(manager.RecordBoot(), 2);
    manager.Checkpoint(loaded, built.active_session);
  }
  {
    PoseGraphData loaded;
    MapManager manager(directory);
    const std::optional<MapManager::LoadResult> result = manager.Load(loaded);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(manager.boot_count(), 2);
    ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
    EXPECT_EQ(result->unfrozen_sessions.front().last_fed_boot, 2);
  }
}

TEST(MapManagerTest, RemoveSessionDeletesTheFileAndTheManifestEntryAndScrubsReferences) {
  const std::string directory = MakeTempDir("remove");
  BuiltGraph built = BuildGraph();
  const SessionId fed = AddFedSession(built);

  MapManager manager(directory);
  manager.OnSessionFrozen(built.graph, built.frozen_session);
  manager.Checkpoint(built.graph, fed);
  ASSERT_TRUE(manager.InspectSessionFile(built.active_session).has_value());

  // The graph drops the session and every constraint touching it, then the manager deletes the
  // file and manifest entry, so no other file can hand a dangling constraint back.
  built.graph.RemoveSession(built.active_session);
  manager.RemoveSession(built.graph, built.active_session);
  EXPECT_FALSE(manager.InspectSessionFile(built.active_session).has_value());
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory) /
                                       MapManager::SessionFileName(built.active_session)));

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_sessions, 2);
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  EXPECT_EQ(result->unfrozen_sessions.front().id, fed);
  EXPECT_FALSE(loaded.HasSession(built.active_session));
  for (const Constraint& constraint : loaded.constraints()) {
    EXPECT_NE(constraint.from.session(), built.active_session);
    if (constraint.to.has_value()) {
      EXPECT_NE(constraint.to->session(), built.active_session);
    }
  }
}

// Proto types live inside map_manager and nowhere else.
TEST(MapManagerTest, NoProtoHeaderLeaksOutsideMapManager) {
  const std::filesystem::path root = std::filesystem::path(EVERGREENSLAM_CONFIG_DIR).parent_path();
  std::vector<std::string> offenders;
  for (const std::string& sub : {"core/src", "core/test", "adapters", "apps"}) {
    const std::filesystem::path directory = root / sub;
    if (!std::filesystem::exists(directory)) {
      continue;
    }
    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const std::string path = entry.path().string();
      const std::string extension = entry.path().extension().string();
      if (extension != ".h" && extension != ".cc") {
        continue;
      }
      if (path.find("/map_manager/") != std::string::npos) {
        continue;
      }
      const std::string contents = ReadWhole(path);
      if (contents.find("map.pb.h") != std::string::npos ||
          contents.find("proto_conversion.h") != std::string::npos) {
        offenders.push_back(path);
      }
    }
  }
  EXPECT_TRUE(offenders.empty()) << "proto reached " << offenders.front();
}

}  // namespace
}  // namespace evergreenslam::lifelong
