/**
 * @file optimization.h
 * @author hang chen (chen@hang.plus)
 * @brief Sparse pose adjustment over a resident, incrementally maintained ceres::Problem.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_H_
#define EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_H_

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <map>
#include <ostream>
#include <set>
#include <vector>

#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

// The Problem lives as long as the graph does; it is never rebuilt.
class Optimization {
 public:
  explicit Optimization(const OptimizationOption& option = OptimizationOption(),
                        const ConstraintWeightOption& weight = ConstraintWeightOption());

  void AddVariable(const VariableId& id, const Eigen::Affine2d& global_pose);
  void SetVariablePose(const VariableId& id, const Eigen::Affine2d& global_pose);
  void AddConstraint(const Constraint& constraint);

  void RemoveVariable(const VariableId& id);
  void RenameVariable(const VariableId& old_id, const VariableId& new_id);

  void FreezeSession(SessionId id);
  void BuildFrom(const PoseGraphData& graph);

  void Optimize(PoseGraphData& graph);

  bool HasVariable(const VariableId& id) const;
  bool IsVariableConstant(const VariableId& id) const;
  bool IsSessionFrozen(SessionId id) const { return frozen_sessions_.count(id) > 0; }
  // Pinned by frozen truth (or a prior measured against it) rather than by a gauge datum of its
  // own; a datum-relative marginal says nothing about the global frame. Meaningful after a solve.
  bool IsSessionAnchoredToFrozen(SessionId id) const;
  Eigen::Affine2d GetVariablePose(const VariableId& id) const;

  int num_variables() const { return static_cast<int>(blocks_.size()); }
  int num_residual_blocks() const { return problem_.NumResidualBlocks(); }
  int num_solves() const { return num_solves_; }
  const OptimizationOption& option() const { return option_; }

  // The covariance evaluator and the trimmer need the live linearization.
  ceres::Problem& mutable_problem() { return problem_; }
  double* mutable_parameter_block(const VariableId& id);

 private:
  // Fix one node per unanchored connected component to eliminate gauge freedom.
  void SetGaugeConstraints();
  void WriteBack(PoseGraphData& graph);
  void RemoveResidual(ceres::ResidualBlockId residual_id);
  bool IsFrozen(const VariableId& id) const;

  OptimizationOption option_;
  // Declared before problem_ so it outlives it: the problem holds it without owning it.
  ceres::HuberLoss inter_submap_loss_;
  ceres::Problem problem_;

  // std::map keeps element addresses stable, which is what lets Ceres hold them.
  std::map<VariableId, std::array<double, 3>> blocks_;
  std::map<VariableId, std::set<ceres::ResidualBlockId>> residuals_by_variable_;
  std::map<ceres::ResidualBlockId, std::vector<VariableId>> residual_endpoints_;
  std::set<ceres::ResidualBlockId> prior_residuals_;

  std::set<SessionId> frozen_sessions_;
  std::set<VariableId> gauge_variables_;

  int num_solves_ = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_OPTIMIZATION_OPTIMIZATION_H_
