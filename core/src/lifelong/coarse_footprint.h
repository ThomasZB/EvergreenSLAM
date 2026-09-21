/**
 * @file coarse_footprint.h
 * @author hang chen (chen@hang.plus)
 * @brief The set of coarse global cells a submap's known area covers.
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_COARSE_FOOTPRINT_H_
#define EVERGREENSLAM_LIFELONG_COARSE_FOOTPRINT_H_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

std::int64_t CoarseCellOf(const Eigen::Vector2d& point, double resolution);

// Footprint at record.global_pose; unordered, without duplicates.
std::vector<std::int64_t> ComputeCoarseFootprint(const SubmapRecord& record, double resolution);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_COARSE_FOOTPRINT_H_
