/**
 * @file debug_sink.h
 * @author hang chen (chen@hang.plus)
 * @brief Where the pipeline hands out its intermediates. Interface only, no transport.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_DEBUG_DEBUG_SINK_H_
#define EVERGREENSLAM_DEBUG_DEBUG_SINK_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <unordered_map>

#include "common/time.h"
#include "mapping/grid_mapping/grid_map.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::debug {

// Implementations run on the SLAM thread and every one of these is a silent failure:
// return promptly, never block on I/O, drop rather than wait, and copy anything kept past the
// call. Nothing here may be non-const: an implementation that writes back changes the pipeline
// it is supposed to be observing.
class DebugSink {
 public:
  virtual ~DebugSink() = default;

  // `poses` is keyed by stage name ("predicted", "coarse", "matched") so a new intermediate
  // costs a string, not a signature change. `point_cloud` is exactly what the matcher consumed,
  // untransformed, so a viewer can draw it under any of the poses.
  virtual void PublishScanMatch(common::Time time,
                                const std::unordered_map<std::string, Eigen::Affine2d>& poses,
                                const sensor::PointCloud& point_cloud,
                                const mapping::GridMapu8& grid_map, double score) = 0;
};

}  // namespace evergreenslam::debug

#endif  // EVERGREENSLAM_DEBUG_DEBUG_SINK_H_
