/**
 * @file here_tracker.cc
 * @author hang chen (chen@hang.plus)
 * @brief Which place the robot is at, with hysteresis so `here` does not flicker at a boundary.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/here_tracker.h"

namespace evergreenslam::agent {

std::optional<size_t> HereTracker::Update(const std::vector<PlaceDistance>& places) {
  if (current_.has_value()) {
    for (size_t i = 0; i < places.size(); ++i) {
      if (places[i].path == *current_ && places[i].distance_m < kLeaveRadius) {
        return i;
      }
    }
  }
  std::optional<size_t> nearest;
  for (size_t i = 0; i < places.size(); ++i) {
    if (places[i].distance_m <= kEnterRadius &&
        (!nearest.has_value() || places[i].distance_m < places[*nearest].distance_m)) {
      nearest = i;
    }
  }
  current_ = nearest.has_value() ? std::optional<std::string>(places[*nearest].path) : std::nullopt;
  return nearest;
}

}  // namespace evergreenslam::agent
