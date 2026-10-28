/**
 * @file pose_graph_events.h
 * @author hang chen (chen@hang.plus)
 * @brief Bounded log of what the backend did to the robot's pose and to the sessions.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_REPORTING_POSE_GRAPH_EVENTS_H_
#define EVERGREENSLAM_LIFELONG_REPORTING_POSE_GRAPH_EVENTS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "common/time.h"
#include "lifelong/ids.h"

namespace evergreenslam::lifelong {

struct PoseCorrection {
  double translation = 0.0;
  double rotation = 0.0;
};
PoseCorrection CorrectionBetween(const Eigen::Affine2d& before, const Eigen::Affine2d& after);

struct PoseGraphEvent {
  enum class Kind {
    RELOCALIZATION,
    POSE_CORRECTION,
    SESSION_FROZEN,
    SESSION_REMOVED,
    FED_SESSION_DROPPED
  };

  int64_t seq = 0;
  common::Time time;
  Kind kind = Kind::RELOCALIZATION;
  int candidates = 0;
  int constraints_added = 0;
  bool reseeded = false;
  std::optional<PoseCorrection> correction;
  std::optional<SessionId> session;
  std::optional<SessionId> fed_now;
};
const char* ToString(PoseGraphEvent::Kind kind);

class PoseGraphEvents {
 public:
  struct Page {
    int64_t first_seq = 0;
    int64_t next_seq = 0;
    bool lost = false;
    std::vector<PoseGraphEvent> events;
  };

  explicit PoseGraphEvents(size_t capacity = 256) : capacity_(capacity) {}

  void Append(PoseGraphEvent event);
  Page Since(int64_t seq) const;

 private:
  const size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<PoseGraphEvent> events_;
  int64_t next_seq_ = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_REPORTING_POSE_GRAPH_EVENTS_H_
