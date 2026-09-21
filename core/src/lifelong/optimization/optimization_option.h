/**
 * @file optimization_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_OPTION_H_
#define EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <ostream>

namespace evergreenslam::lifelong {

struct ConstraintWeightOption {
  // Constants rather than per-scene tuning only because MotionFilter keeps adjacent keyframes
  // close enough for the error to stay at this scale.
  double odometry_translation_stddev = 0.01;      // m
  double odometry_rotation_stddev = 0.01;         // rad
  double loop_closure_translation_stddev = 0.01;  // m
  double loop_closure_rotation_stddev = 0.01;     // rad
  // Translation-equivalent: the loss sees the whole whitened 3-vector, so rotation spends it too.
  double loop_closure_huber_distance = 0.1;  // m

  friend std::ostream& operator<<(std::ostream& os, const ConstraintWeightOption& option) {
    os << "ConstraintWeightOption:" << std::endl;
    os << "  odometry_translation_stddev: " << option.odometry_translation_stddev << std::endl;
    os << "  odometry_rotation_stddev: " << option.odometry_rotation_stddev << std::endl;
    os << "  loop_closure_translation_stddev: " << option.loop_closure_translation_stddev
       << std::endl;
    os << "  loop_closure_rotation_stddev: " << option.loop_closure_rotation_stddev << std::endl;
    os << "  loop_closure_huber_distance: " << option.loop_closure_huber_distance << std::endl;
    return os;
  }
};

struct OptimizationOption {
  int optimize_every_n_nodes = 30;
  int max_num_iterations = 50;
  int num_threads = 1;

  friend std::ostream& operator<<(std::ostream& os, const OptimizationOption& option) {
    os << "OptimizationOption:" << std::endl;
    os << "  optimize_every_n_nodes: " << option.optimize_every_n_nodes << std::endl;
    os << "  max_num_iterations: " << option.max_num_iterations << std::endl;
    os << "  num_threads: " << option.num_threads << std::endl;
    return os;
  }
};

OptimizationOption LoadOptimizationOption(const YAML::Node& node);

ConstraintWeightOption LoadConstraintWeightOption(const YAML::Node& node);
Eigen::Matrix3d OdometrySqrtInformation(const ConstraintWeightOption& option);
Eigen::Matrix3d LoopClosureSqrtInformation(const ConstraintWeightOption& option);
// The grid's orientation error moves the far endpoint by lever_arm * angle, independently of
// the position noise at the node: sigma_t_eff^2 = sigma_t^2 + (sigma_theta * L)^2, so the
// translation rows shrink by 1 / sqrt(1 + (w_t * sigma_theta * L)^2).
Eigen::Matrix3d LeverArmSqrtInformation(const Eigen::Matrix3d& sqrt_information, double lever_arm);
// Ceres wants the knee in residual units, and the residual is already whitened.
double LoopClosureHuberDelta(const ConstraintWeightOption& option);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_OPTION_H_
