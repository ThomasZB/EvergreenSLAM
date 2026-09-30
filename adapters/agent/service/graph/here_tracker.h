/**
 * @file here_tracker.h
 * @author hang chen (chen@hang.plus)
 * @brief Which place the robot is at, with hysteresis so `here` does not flicker at a boundary.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_HERE_TRACKER_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_HERE_TRACKER_H_

#include <optional>
#include <string>
#include <vector>

namespace evergreenslam::agent {

struct PlaceDistance {
  std::string path;
  double distance_m = 0.0;
};

// Service-thread state: the one httplib worker is its only caller.
class HereTracker {
 public:
  static constexpr double kEnterRadius = 2.5;
  static constexpr double kLeaveRadius = 3.75;

  // `places`: every resolvable place with its distance from the robot; empty clears the state.
  // Returns the index into `places` of the current place.
  std::optional<size_t> Update(const std::vector<PlaceDistance>& places);

 private:
  std::optional<std::string> current_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_HERE_TRACKER_H_
