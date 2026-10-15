/**
 * @file pose_history.h
 * @author hang chen (chen@hang.plus)
 * @brief The host's last 10 s of frontend poses: the newest sample for GET /pose, interpolation
 * for observe --at.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_POSE_HISTORY_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_POSE_HISTORY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>

#include "common/time.h"
#include "service/http/service_hooks.h"

namespace evergreenslam::agent {

class PoseHistory {
 public:
  static constexpr common::Duration kSpan = std::chrono::seconds(10);
  static constexpr common::Duration kMaxAhead = std::chrono::milliseconds(200);
  static constexpr common::Duration kMaxGap = std::chrono::milliseconds(500);

  void Add(common::Time time, const Eigen::Affine2d& local_pose,
           const Eigen::Vector3d& local_velocity,
           std::chrono::steady_clock::time_point received = std::chrono::steady_clock::now());
  void Reset();

  std::optional<HostPose> Latest() const;
  PoseAt At(common::Time time) const;

 private:
  struct Sample {
    common::Time time;
    Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
  };

  std::deque<Sample> samples_;
  std::optional<HostPose> latest_;
  uint64_t seq_ = 0;
  uint64_t epoch_ = 0;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_POSE_HISTORY_H_
