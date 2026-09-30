/**
 * @file match_score_average.cc
 * @author hang chen (chen@hang.plus)
 * @brief The scan-match score averaged over time, as hosts feed it to HostFrame.
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/match_score_average.h"

#include <algorithm>
#include <cmath>

namespace evergreenslam::agent {

double MatchScoreAverage::Add(common::Time time, double score) {
  if (!last_time_.has_value()) {
    average_ = score;
  } else {
    const double dt = std::max(0.0, common::ToSeconds(time - *last_time_));
    average_ += (1.0 - std::exp(-dt / kTimeConstantSeconds)) * (score - average_);
  }
  last_time_ = time;
  return average_;
}

}  // namespace evergreenslam::agent
