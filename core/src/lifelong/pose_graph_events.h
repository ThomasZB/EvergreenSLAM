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

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_EVENTS_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_EVENTS_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "common/time.h"
#include "lifelong/ids.h"

namespace evergreenslam::lifelong {

using RelocalizationId = int64_t;

enum class RelocalizationState { PENDING, MATCHED, NO_MATCH, SUPERSEDED };
const char* ToString(RelocalizationState state);

// Who asked: an operator through the API, or the backend's own kidnap detector.
enum class RelocalizationTrigger { OPERATOR, KIDNAP };
const char* ToString(RelocalizationTrigger trigger);

struct PoseCorrection {
  double translation = 0.0;
  double rotation = 0.0;
};

// What the kidnap detector saw over the keyframes that made it start a global search.
struct KidnapEvidence {
  int keyframes = 0;
  // Lowest frontend match score since the fed session's last closure onto the frozen base;
  // empty when it never had one, which counts as a break in continuity by itself.
  std::optional<double> lowest_match_score;
  // The newest keyframe against the frozen base.
  double base_score = 0.0;
  double base_coverage = 0.0;
  double distance_without_base_match = 0.0;
};

struct PoseGraphEvent {
  enum class Kind {
    RELOCALIZATION,
    POSE_CORRECTION,
    SESSION_FROZEN,
    SESSION_REMOVED,
    FED_SESSION_DROPPED,
    KIDNAP_SUSPECTED
  };

  int64_t seq = 0;
  common::Time time;
  Kind kind = Kind::RELOCALIZATION;
  std::optional<RelocalizationId> relocalization_id;
  std::optional<RelocalizationState> outcome;
  int candidates = 0;
  int constraints_added = 0;
  std::optional<RelocalizationId> superseded_by;
  std::optional<PoseCorrection> correction;
  std::optional<SessionId> session;
  std::optional<SessionId> fed_now;
  std::optional<KidnapEvidence> kidnap;
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

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_EVENTS_H_
