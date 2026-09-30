/**
 * @file match_score_average.h
 * @author hang chen (chen@hang.plus)
 * @brief The scan-match score averaged over time, as hosts feed it to HostFrame.
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_MATCH_SCORE_AVERAGE_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_MATCH_SCORE_AVERAGE_H_

#include <optional>

#include "common/time.h"

namespace evergreenslam::agent {

// Exponential average with a time constant, so the scan rate does not change what it remembers.
// Sampled per scan on the host, since /here is read too rarely to sample it.
class MatchScoreAverage {
 public:
  static constexpr double kTimeConstantSeconds = 2.0;

  // Returns the average including `score`; the first sample seeds it.
  double Add(common::Time time, double score);

 private:
  std::optional<common::Time> last_time_;
  double average_ = 0.0;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_MATCH_SCORE_AVERAGE_H_
