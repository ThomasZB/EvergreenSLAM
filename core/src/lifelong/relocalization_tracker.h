/**
 * @file relocalization_tracker.h
 * @author hang chen (chen@hang.plus)
 * @brief Ids and outcomes of operator relocalization requests, settled per candidate round.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_RELOCALIZATION_TRACKER_H_
#define EVERGREENSLAM_LIFELONG_RELOCALIZATION_TRACKER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph_events.h"

namespace evergreenslam::lifelong {

struct RelocalizationStatus {
  RelocalizationId id = 0;
  bool has_prior = false;
  RelocalizationTrigger trigger = RelocalizationTrigger::OPERATOR;
  RelocalizationState state = RelocalizationState::PENDING;
  int candidates = 0;
  int constraints_added = 0;
  std::optional<PoseCorrection> correction;
  std::optional<common::Time> requested_at;
  std::optional<common::Time> resolved_at;
  std::optional<RelocalizationId> superseded_by;
};

class RelocalizationTracker {
 public:
  struct Round {
    RelocalizationId id = 0;
    std::optional<int> expected;
    int finished = 0;
    int accepted = 0;
    int solves_before_last_accept = 0;
    Eigen::Affine2d robot_before = Eigen::Affine2d::Identity();
    Eigen::Affine2d node_local = Eigen::Affine2d::Identity();
  };

  explicit RelocalizationTracker(size_t capacity = 16) : capacity_(capacity) {}

  RelocalizationId Mint(bool has_prior,
                        RelocalizationTrigger trigger = RelocalizationTrigger::OPERATOR);
  std::optional<RelocalizationStatus> Get(RelocalizationId id) const;
  std::optional<RelocalizationStatus> Latest() const;

  // Started requests still pending; `by` empty when their fed session is gone.
  std::vector<RelocalizationStatus> SupersedePending(std::optional<RelocalizationId> by,
                                                     common::Time time);
  void MarkStarted(RelocalizationId id);
  void BeginRound(Round round, std::optional<common::Time> requested_at);
  bool NoteMatch(RelocalizationId id, bool accepted, int solves_before_add);
  bool SetCandidates(RelocalizationId id, int candidates);
  const std::optional<Round>& round() const { return round_; }
  RelocalizationStatus Resolve(RelocalizationState state, std::optional<PoseCorrection> correction,
                               common::Time time);

 private:
  struct Entry {
    RelocalizationStatus status;
    bool started = false;
  };

  std::deque<Entry>::iterator FindLocked(RelocalizationId id);
  bool RoundComplete() const;

  const size_t capacity_;
  mutable std::mutex mutex_;
  RelocalizationId next_id_ = 1;
  std::deque<Entry> recent_;
  std::optional<Round> round_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_RELOCALIZATION_TRACKER_H_
