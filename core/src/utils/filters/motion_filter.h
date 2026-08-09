/**
 * @file motion_filter.h
 * @author hang chen (chen@hang.plus)
 * @brief Drops scans taken too close to the last one that was kept.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_H_
#define EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/time.h"
#include "utils/filters/motion_filter_option.h"

namespace evergreenslam::utils::filters {

class MotionFilter {
 public:
  explicit MotionFilter(const MotionFilterOption& option = MotionFilterOption())
      : option_(option) {}

  void Reset();
  bool IsSimilar(common::Time time, const Eigen::Affine2d& pose);

  int num_total() const { return num_total_; }
  int num_kept() const { return num_kept_; }

 private:
  MotionFilterOption option_;

  bool has_reference_ = false;
  common::Time last_time_ = {};
  Eigen::Affine2d last_pose_ = Eigen::Affine2d::Identity();

  int num_total_ = 0;
  int num_kept_ = 0;
};

}  // namespace evergreenslam::utils::filters

#endif  // EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_H_
