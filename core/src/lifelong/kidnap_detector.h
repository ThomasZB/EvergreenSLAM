/**
 * @file kidnap_detector.h
 * @author hang chen (chen@hang.plus)
 * @brief Per keyframe: did the robot lose its place on the frozen base, so a global search is due.
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_H_
#define EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_H_

#include <optional>

#include "lifelong/kidnap_detector_option.h"
#include "lifelong/pose_graph_events.h"
#include "lifelong/sessions/frozen_base_map.h"

namespace evergreenslam::lifelong {

// The fed session's newest keyframe as the detector sees it.
struct KidnapSignals {
  std::optional<double> match_score;
  // Empty while there is no frozen base to compare with.
  std::optional<FrozenBaseMap::Agreement> base;
  // DriftTracker::SinceBaseMatch and PathLength of the fed session.
  std::optional<double> since_base_match;
  double path_length = 0.0;
};

// Three signals must hold together, since each alone has an innocent reading: the frontend lost
// continuity (a low match score since the last base closure, or no base closure ever: a boot
// seed is a guess), the newest keyframe disagrees with the frozen base where it can be checked,
// and no closure onto the base came for a while. Ground the base never mapped leaves coverage
// low, so exploring is not a kidnap. Backend task only.
class KidnapDetector {
 public:
  struct Status {
    int suspect_keyframes = 0;
    bool continuity_broken = false;
    int cooldown_keyframes_left = 0;
  };

  explicit KidnapDetector(const KidnapDetectorOption& option = KidnapDetectorOption());

  // One call per fed-session keyframe. While a relocalization is open the streak restarts: a
  // correction is on its way, and the signals still describe the pose before it. Returns the
  // evidence when this keyframe completes a streak and no cooldown runs.
  std::optional<KidnapEvidence> Observe(const KidnapSignals& signals, bool relocalization_open);
  // The search the last verdict started settled; starts the cooldown.
  void OnSearchSettled(RelocalizationState state);
  // The fed session was replaced by a fresh one: its marks mean nothing for the next.
  void Reset();

  const Status& status() const { return status_; }
  const KidnapDetectorOption& option() const { return option_; }

 private:
  KidnapDetectorOption option_;
  Status status_;
  std::optional<double> lowest_match_score_;
  // PathLength - SinceBaseMatch: where the last base closure happened; it only moves forward.
  std::optional<double> base_match_mark_;
  int cooldown_keyframes_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_KIDNAP_DETECTOR_H_
