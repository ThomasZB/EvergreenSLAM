/**
 * @file session_data.h
 * @author hang chen (chen@hang.plus)
 * @brief One continuous run of mapping: the unit that gets frozen.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_DATA_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_DATA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "common/time.h"
#include "lifelong/ids.h"

namespace evergreenslam::lifelong {

enum class SessionState { ACTIVE, FROZEN };

struct SessionData {
  SessionId id;
  SessionState state = SessionState::ACTIVE;
  common::Time start_time;
  common::Time last_node_time;

  // Creation order, trimmed entries removed; back() is what the next node aligns through.
  std::vector<SubmapId> submap_ids;
  std::vector<NodeId> node_ids;

  Eigen::Affine2d local_to_global = Eigen::Affine2d::Identity();

  bool frozen() const { return state == SessionState::FROZEN; }
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_DATA_H_
