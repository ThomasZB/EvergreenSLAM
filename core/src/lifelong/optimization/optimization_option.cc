/**
 * @file optimization_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/optimization_option.h"

#include <glog/logging.h>

#include <cmath>

#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

namespace {

Eigen::Matrix3d SqrtInformation(double translation_stddev, double rotation_stddev) {
  CHECK_GT(translation_stddev, 0.0);
  CHECK_GT(rotation_stddev, 0.0);
  Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Zero();
  sqrt_information.diagonal() << 1.0 / translation_stddev, 1.0 / translation_stddev,
      1.0 / rotation_stddev;
  return sqrt_information;
}

}  // namespace

OptimizationOption LoadOptimizationOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  OptimizationOption option;
  option.optimize_every_n_nodes =
      LoadOption(node, "optimize_every_n_nodes", option.optimize_every_n_nodes);
  option.max_num_iterations = LoadOption(node, "max_num_iterations", option.max_num_iterations);
  option.num_threads = LoadOption(node, "num_threads", option.num_threads);
  return option;
}

ConstraintWeightOption LoadConstraintWeightOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  ConstraintWeightOption option;
  option.odometry_translation_stddev =
      LoadOption(node, "odometry_translation_stddev", option.odometry_translation_stddev);
  option.odometry_rotation_stddev =
      LoadOption(node, "odometry_rotation_stddev", option.odometry_rotation_stddev);
  option.loop_closure_translation_stddev =
      LoadOption(node, "loop_closure_translation_stddev", option.loop_closure_translation_stddev);
  option.loop_closure_rotation_stddev =
      LoadOption(node, "loop_closure_rotation_stddev", option.loop_closure_rotation_stddev);
  option.loop_closure_huber_distance =
      LoadOption(node, "loop_closure_huber_distance", option.loop_closure_huber_distance);
  return option;
}

Eigen::Matrix3d OdometrySqrtInformation(const ConstraintWeightOption& option) {
  return SqrtInformation(option.odometry_translation_stddev, option.odometry_rotation_stddev);
}

Eigen::Matrix3d LoopClosureSqrtInformation(const ConstraintWeightOption& option) {
  return SqrtInformation(option.loop_closure_translation_stddev,
                         option.loop_closure_rotation_stddev);
}

Eigen::Matrix3d LeverArmSqrtInformation(const Eigen::Matrix3d& sqrt_information, double lever_arm) {
  CHECK_GE(lever_arm, 0.0);
  CHECK_GT(sqrt_information(2, 2), 0.0);
  const double sigma_theta = 1.0 / sqrt_information(2, 2);
  const double scale = 1.0 / std::hypot(1.0, sqrt_information(0, 0) * sigma_theta * lever_arm);
  Eigen::Matrix3d scaled = sqrt_information;
  scaled(0, 0) *= scale;
  scaled(1, 1) *= scale;
  return scaled;
}

double LoopClosureHuberDelta(const ConstraintWeightOption& option) {
  CHECK_GT(option.loop_closure_huber_distance, 0.0);
  CHECK_GT(option.loop_closure_translation_stddev, 0.0);
  return option.loop_closure_huber_distance / option.loop_closure_translation_stddev;
}

}  // namespace evergreenslam::lifelong
