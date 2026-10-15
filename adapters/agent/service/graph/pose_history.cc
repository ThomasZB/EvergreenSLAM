/**
 * @file pose_history.cc
 * @author hang chen (chen@hang.plus)
 * @brief The host's last 10 s of frontend poses: the newest sample for GET /pose, interpolation
 * for observe --at.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/pose_history.h"

#include <algorithm>

#include "utils/transform/transform.h"

namespace evergreenslam::agent {

void PoseHistory::Add(common::Time time, const Eigen::Affine2d& local_pose,
                      const Eigen::Vector3d& local_velocity,
                      std::chrono::steady_clock::time_point received) {
  if (!samples_.empty() && time <= samples_.back().time) {
    samples_.clear();
  }
  samples_.push_back({time, local_pose});
  while (samples_.front().time < time - kSpan) {
    samples_.pop_front();
  }
  HostPose pose;
  pose.pose = StampedPose{time, local_pose};
  pose.velocity << local_pose.linear().transpose() * local_velocity.head<2>(), local_velocity.z();
  pose.seq = ++seq_;
  pose.epoch = epoch_;
  pose.received = received;
  latest_ = pose;
}

void PoseHistory::Reset() {
  samples_.clear();
  latest_.reset();
  ++epoch_;
}

std::optional<HostPose> PoseHistory::Latest() const { return latest_; }

PoseAt PoseHistory::At(common::Time time) const {
  using Refusal = PoseAt::Refusal;
  if (samples_.empty()) {
    return PoseAt{std::nullopt, Refusal::NO_SAMPLE};
  }
  if (time < samples_.front().time) {
    return PoseAt{std::nullopt, Refusal::TOO_OLD};
  }
  if (time >= samples_.back().time) {
    if (time - samples_.back().time > kMaxAhead) {
      return PoseAt{std::nullopt, Refusal::IN_FUTURE};
    }
    return PoseAt{samples_.back().local_pose, Refusal::NONE};
  }
  const auto after = std::upper_bound(
      samples_.begin(), samples_.end(), time,
      [](common::Time value, const Sample& sample) { return value < sample.time; });
  const Sample& start = *(after - 1);
  const Sample& end = *after;
  if (start.time == time) {
    return PoseAt{start.local_pose, Refusal::NONE};
  }
  if (end.time - start.time > kMaxGap) {
    return PoseAt{std::nullopt, Refusal::GAP};
  }
  const double factor =
      common::ToSeconds(time - start.time) / common::ToSeconds(end.time - start.time);
  return PoseAt{utils::transform::Interpolate(start.local_pose, end.local_pose, factor),
                Refusal::NONE};
}

}  // namespace evergreenslam::agent
