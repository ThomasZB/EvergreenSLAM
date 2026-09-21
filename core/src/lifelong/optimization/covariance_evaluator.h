/**
 * @file covariance_evaluator.h
 * @author hang chen (chen@hang.plus)
 * @brief The two covariances the pose graph asks for, kept apart on purpose.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_OPTIMIZATION_COVARIANCE_EVALUATOR_H_
#define EVERGREENSLAM_LIFELONG_OPTIMIZATION_COVARIANCE_EVALUATOR_H_

#include <Eigen/Core>
#include <map>
#include <optional>
#include <vector>

#include "lifelong/optimization/optimization.h"
#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

class CovarianceEvaluator {
 public:
  explicit CovarianceEvaluator(Optimization& optimization);

  std::optional<Eigen::Matrix3d> ComputeMarginalCovarianceInGlobal(const VariableId& id);

  std::optional<std::map<VariableId, Eigen::Matrix3d>> ComputeMarginalCovariancesInGlobal(
      const std::vector<VariableId>& ids);

  // Conditioned on everything outside `blanket`, so systematically smaller than the marginal.
  std::optional<Eigen::MatrixXd> ComputeConditionalJointCovarianceInBlanket(
      const std::vector<VariableId>& blanket);

 private:
  Optimization& optimization_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_OPTIMIZATION_COVARIANCE_EVALUATOR_H_
