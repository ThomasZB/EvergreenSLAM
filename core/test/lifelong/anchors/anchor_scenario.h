/**
 * @file anchor_scenario.h
 * @author hang chen (chen@hang.plus)
 * @brief Shared pieces of the anchor e2e tests: task-side calls, options and pose error.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_TEST_LIFELONG_ANCHORS_ANCHOR_SCENARIO_H_
#define EVERGREENSLAM_TEST_LIFELONG_ANCHORS_ANCHOR_SCENARIO_H_

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <future>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "../testing/load_map.h"
#include "../testing/loop_scenario.h"
#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong::testing {

constexpr double kAnchorTranslationTolerance = 0.05;
constexpr double kAnchorRotationTolerance = 0.05;

// graph() and anchors() belong to the backend task, as they do for the agent service.
template <typename Function>
auto RunOnTask(PoseGraph& backend, Function function) -> std::invoke_result_t<Function> {
  using Result = std::invoke_result_t<Function>;
  std::promise<Result> promise;
  std::future<Result> future = promise.get_future();
  backend.Enqueue([&promise, &function] {
    if constexpr (std::is_void_v<Result>) {
      function();
      promise.set_value();
    } else {
      promise.set_value(function());
    }
  });
  return future.get();
}

inline std::optional<Anchor> SaveAnchor(PoseGraph& backend, bool keep_scan = true) {
  return RunOnTask(backend,
                   [&backend, keep_scan] { return backend.SaveAnchorOnTask(keep_scan).anchor; });
}

inline ResolvedAnchor ResolveAnchor(PoseGraph& backend, AnchorId id) {
  const std::optional<ResolvedAnchor> resolved =
      RunOnTask(backend, [&backend, id] { return backend.anchors().Resolve(backend.graph(), id); });
  CHECK(resolved.has_value()) << "anchor " << id << " is unknown";
  return *resolved;
}

inline std::optional<Anchor> GetAnchor(PoseGraph& backend, AnchorId id) {
  return RunOnTask(backend, [&backend, id] { return backend.anchors().Get(id); });
}

inline std::vector<Eigen::Affine2d> GroundTruthOdometry(int num_steps) {
  return LoopTruth(num_steps);
}

inline double TranslationError(const Eigen::Affine2d& pose, const Eigen::Affine2d& truth) {
  return (pose.translation() - truth.translation()).norm();
}

inline double RotationError(const Eigen::Affine2d& pose, const Eigen::Affine2d& truth) {
  return std::abs(utils::transform::NormalizeAngle(utils::transform::GetYaw(pose) -
                                                   utils::transform::GetYaw(truth)));
}

inline void ExpectNearTruth(const ResolvedAnchor& resolved, int step) {
  ASSERT_TRUE(resolved.global_pose.has_value())
      << "anchor " << resolved.id << " is " << ToString(resolved.state) << " ("
      << ToString(resolved.orphan_reason) << ")";
  EXPECT_LT(TranslationError(*resolved.global_pose, LoopGroundTruth(step)),
            kAnchorTranslationTolerance)
      << "anchor " << resolved.id << " saved at step " << step;
  EXPECT_LT(RotationError(*resolved.global_pose, LoopGroundTruth(step)), kAnchorRotationTolerance)
      << "anchor " << resolved.id << " saved at step " << step;
}

// Relaxed judge and a short solve cadence so the stories fit in a few laps; one match worker so
// the closure set does not vary between runs. Trimming is opted into by the stories about it.
inline PoseGraphOption AnchorTestOption() {
  PoseGraphOption option;
  option.constraint_builder.num_match_workers = 1;
  option.optimization.optimize_every_n_nodes = 10;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.freeze_judge.max_translation_stddev = 1.0;
  option.session_manager.freeze_judge.max_rotation_stddev = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.constraint_builder.max_candidate_slack = 5.0;
  option.trim = false;
  return option;
}

inline PoseGraphOption NoFreeze(PoseGraphOption option) {
  option.session_manager.auto_freeze = false;
  return option;
}

// Boot 1: drift-free, frozen at the bootstrap. Returns the step the freeze landed after.
inline int FreezeABase(PoseGraph& backend, Frontend& frontend, int max_steps) {
  int step = 0;
  while (backend.session_manager().num_sessions_frozen() == 0 && step < max_steps) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  EXPECT_EQ(backend.session_manager().num_sessions_frozen(), 1)
      << "the boot session never froze; last verdict: "
      << ToString(backend.session_manager().last_verdict().rejection);
  return step;
}

// A fresh reader: the anchors file is whichever one the manifest on disk names.
inline std::optional<AnchorTable> ReadAnchorsFromDisk(const std::string& directory) {
  MapManager reader(directory);
  PoseGraphData graph;
  LoadMap(reader, graph);
  return reader.ReadAnchors();
}

inline std::string AnchorsPath(const MapManager& manager) {
  CHECK(manager.anchors_file_name().has_value())
      << "no anchors committed in " << manager.directory();
  return (std::filesystem::path(manager.directory()) / *manager.anchors_file_name()).string();
}

}  // namespace evergreenslam::lifelong::testing

#endif  // EVERGREENSLAM_TEST_LIFELONG_ANCHORS_ANCHOR_SCENARIO_H_
