/**
 * @file explicit_ingest.h
 * @author hang chen (chen@hang.plus)
 * @brief Test-side ingest with explicit graph ids, for fixtures that build arbitrary sessions.
 * @version 0.1
 * @date 2026-08-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_TEST_LIFELONG_TESTING_EXPLICIT_INGEST_H_
#define EVERGREENSLAM_TEST_LIFELONG_TESTING_EXPLICIT_INGEST_H_

#include <glog/logging.h>

#include <memory>
#include <utility>
#include <vector>

#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph.h"

namespace evergreenslam::lifelong::testing {

// What production ingest hands the graph, but with the ids chosen by the test. The session
// manager is not told: the test notifies the manager it drives.
struct ExplicitInsertion {
  NodeId node_id;
  mapping::TrajectoryNode node;
  std::vector<std::pair<SubmapId, std::shared_ptr<const mapping::Submap>>> insertion_submaps;
};

inline void EnqueueExplicitInsertion(PoseGraph& pose_graph, ExplicitInsertion insertion) {
  pose_graph.Enqueue([&pose_graph, insertion = std::move(insertion)] {
    PoseGraphData& data = pose_graph.mutable_graph();
    Optimization& optimization = pose_graph.mutable_optimization();
    const SessionId session_id = SessionOf(insertion.node_id);
    CHECK(data.HasSession(session_id)) << "open the session before feeding it";
    CHECK(!insertion.insertion_submaps.empty());

    std::vector<SubmapId> submap_ids;
    submap_ids.reserve(insertion.insertion_submaps.size());
    for (const auto& [submap_id, submap] : insertion.insertion_submaps) {
      CHECK(submap != nullptr);
      submap_ids.push_back(submap_id);
      if (data.HasSubmap(submap_id)) {
        continue;
      }
      SubmapRecord record;
      record.id = submap_id;
      record.submap = submap;
      record.local_pose = submap->local_pose();
      // Recomputed per submap on purpose: adding one submap moves the alignment chain onto it.
      const std::optional<Eigen::Affine2d> session_to_global =
          data.ComputeSessionToGlobal(session_id);
      const Eigen::Affine2d alignment = session_to_global.has_value()
                                            ? *session_to_global
                                            : data.session(session_id).local_to_global;
      record.global_pose = alignment * record.local_pose;
      data.AddSubmap(record);
      optimization.AddVariable(VariableId::Of(record.id), record.global_pose);
    }

    const std::optional<Eigen::Affine2d> session_to_global =
        data.ComputeSessionToGlobal(session_id);
    CHECK(session_to_global.has_value());

    Node node;
    node.id = insertion.node_id;
    node.constant_data = insertion.node;
    node.global_pose = *session_to_global * node.constant_data.local_pose;
    data.AddNode(node, submap_ids);
    optimization.AddVariable(VariableId::Of(node.id), node.global_pose);

    for (const SubmapId& submap_id : submap_ids) {
      Constraint constraint;
      constraint.type = Constraint::Type::INTRA_SUBMAP;
      constraint.from = VariableId::Of(submap_id);
      constraint.to = VariableId::Of(node.id);
      constraint.relative_pose =
          data.submap(submap_id).local_pose.inverse() * node.constant_data.local_pose;
      constraint.sqrt_information = OdometrySqrtInformation(ConstraintWeightOption());
      data.AddConstraint(constraint);
      optimization.AddConstraint(constraint);
    }

    // Keeps the production solve cadence so poses settle the same way mid-feed.
    const int every_n_nodes = optimization.option().optimize_every_n_nodes;
    if (static_cast<int>(data.session(session_id).node_ids.size()) % every_n_nodes == 0) {
      optimization.Optimize(data);
    }
  });
}

}  // namespace evergreenslam::lifelong::testing

#endif  // EVERGREENSLAM_TEST_LIFELONG_TESTING_EXPLICIT_INGEST_H_
