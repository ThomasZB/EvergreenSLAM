/**
 * @file snapshot_exporter.h
 * @author hang chen (chen@hang.plus)
 * @brief Writes one solve's map, trajectory and places as plain files under snapshots/NNNNNN/.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_SNAPSHOT_EXPORTER_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_SNAPSHOT_EXPORTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "common/time.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"
#include "service/memory/place_store.h"

namespace evergreenslam::agent {

struct SnapshotNode {
  common::Time time;
  Eigen::Affine2d global_pose = Eigen::Affine2d::Identity();
  int session = 0;
};

struct SnapshotSession {
  int id = 0;
  std::string role;
  int num_nodes = 0;
  int num_submaps = 0;
};

// Everything one task copied out; the exporter itself never touches the graph.
struct SnapshotInput {
  mapping::GridMapu8 grid{{}, 0, 0, 0.05, 0.0, 0.0, mapping::kUnknownValue};
  std::vector<SnapshotNode> nodes;
  std::vector<SnapshotSession> sessions;
  std::vector<PlaceRow> places;
  std::optional<Eigen::Affine2d> robot;
  std::optional<int> fed_session;
  int boot_count = 0;
  int num_solves = 0;
  common::Time generated_at;
};

struct SnapshotResult {
  int seq = 0;
  std::string dir;
  std::vector<std::string> files;
};

class SnapshotExporter {
 public:
  // Numbering continues after the highest NNNNNN already in `snapshots_dir`.
  explicit SnapshotExporter(std::string snapshots_dir);

  // Written into NNNNNN.tmp and renamed; empty when any write fails.
  std::optional<SnapshotResult> Export(const SnapshotInput& input);

 private:
  std::string snapshots_dir_;
  int next_seq_ = 1;
};

// nav2 trinary: known and p > 0.5 -> 0, known -> 254, unknown -> 205.
uint8_t TrinaryValue(uint8_t cell);
// Per session in time order: a node is kept once it is >= `min_arc_m` of path from the last kept
// one; each session's first and last node are always kept.
std::vector<SnapshotNode> DecimateTrajectory(std::vector<SnapshotNode> nodes, double min_arc_m);
// One past the highest leading 6-digit number among the entries of `dir`, at least 1.
int NextSequenceNumber(const std::string& dir);
std::string SequenceName(int seq);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_SNAPSHOT_EXPORTER_H_
