/**
 * @file generic_tracking_filter.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_H_
#define EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>

#include "common/time.h"
#include "utils/filters/generic_tracking_filter_option.h"

namespace evergreenslam::utils::filters {

class GenericTrackingFilter {
 public:
  struct Measurement {
    common::Time time;
    Eigen::Affine2d pose = Eigen::Affine2d::Identity();
    Eigen::Matrix<double, 3, 3> covariance;
  };
  struct State {
    common::Time time;
    // [x, y, theta, vx, vy, omega, ax, ay, alpha]
    Eigen::Matrix<double, 9, 1> state;
    Eigen::Matrix<double, 9, 9> covariance;
    Measurement measurement;
  };
  explicit GenericTrackingFilter(
      const GenericTrackingFilterOption& option = GenericTrackingFilterOption());
  GenericTrackingFilter(double trans_alpha, double rot_alpha, double trans_sigma, double rot_sigma);
  ~GenericTrackingFilter() = default;

  void Reset();
  void ResetZero();
  State PredictTime(common::Time time) const;
  void Update(const Measurement& m, bool from_history = false);

  bool empty() const { return state_history_.empty(); }

 private:
  bool Predict(common::Time time);
  Eigen::Matrix<double, 9, 1> PredictState(const Eigen::Matrix<double, 9, 1>& state,
                                           double dt) const;
  State NearestState(common::Time time) const;
  Eigen::Matrix<double, 9, 9> BuildQ(double dt) const;
  Eigen::Matrix<double, 9, 9> BuildF(const Eigen::Matrix<double, 9, 1>& state, double dt) const;
  void UpdateHistoryAndRePredict();

  double trans_alpha_;
  double rot_alpha_;
  double trans_sigma_;
  double rot_sigma_;
  State current_state_;
  Eigen::Matrix<double, 3, 9> H_;
  const size_t max_history_size_ = 100;
  std::deque<State> state_history_;
};

}  // namespace evergreenslam::utils::filters

#endif  // EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_H_