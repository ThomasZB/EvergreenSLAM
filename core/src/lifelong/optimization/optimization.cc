/**
 * @file optimization.cc
 * @author hang chen (chen@hang.plus)
 * @brief Sparse pose adjustment over a resident, incrementally maintained ceres::Problem.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/optimization.h"

#include <glog/logging.h>

#include <algorithm>
#include <numeric>
#include <utility>

#include "utils/config/yaml_utils.h"
#include "utils/cost_functor/spa_cost_function.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

ceres::Problem::Options MakeProblemOptions() {
  ceres::Problem::Options options;
  // Without this every trimmer removal is a linear scan; it cannot be enabled later.
  options.enable_fast_removal = true;
  // The inter-submap loss is a member of this class, so the problem must not delete it.
  options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  return options;
}

class ComponentSet {
 public:
  explicit ComponentSet(size_t size) : parent_(size) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  size_t Find(size_t index) {
    while (parent_[index] != index) {
      parent_[index] = parent_[parent_[index]];
      index = parent_[index];
    }
    return index;
  }

  void Union(size_t a, size_t b) { parent_[Find(a)] = Find(b); }

 private:
  std::vector<size_t> parent_;
};

}  // namespace

Optimization::Optimization(const OptimizationOption& option, const ConstraintWeightOption& weight)
    : option_(option),
      inter_submap_loss_(LoopClosureHuberDelta(weight)),
      problem_(MakeProblemOptions()) {
  CHECK_GT(option_.optimize_every_n_nodes, 0);
}

void Optimization::AddVariable(const VariableId& id, const Eigen::Affine2d& global_pose) {
  if (blocks_.count(id) > 0) {
    return;
  }
  std::array<double, 3>& block = blocks_[id];
  block = transform::ToArray3(global_pose);
  problem_.AddParameterBlock(block.data(), 3);
  if (IsFrozen(id)) {
    problem_.SetParameterBlockConstant(block.data());
  }
}

void Optimization::SetVariablePose(const VariableId& id, const Eigen::Affine2d& global_pose) {
  const auto it = blocks_.find(id);
  CHECK(it != blocks_.end()) << "unknown variable";
  CHECK(!IsFrozen(id)) << "frozen poses are truth";
  it->second = transform::ToArray3(global_pose);
}

void Optimization::AddConstraint(const Constraint& constraint) {
  CHECK_EQ(constraint.type == Constraint::Type::PRIOR, !constraint.to.has_value())
      << "a unary residual is exactly a prior";
  CHECK(blocks_.count(constraint.from) > 0) << "add the variable before its constraints";

  double* from = blocks_.at(constraint.from).data();
  ceres::ResidualBlockId residual_id = nullptr;
  std::vector<VariableId> endpoints;

  if (!constraint.to.has_value()) {
    if (IsFrozen(constraint.from)) {
      return;
    }
    residual_id =
        problem_.AddResidualBlock(utils::cost_functor::CreateSpaAbsolutePoseCostFunction(
                                      constraint.relative_pose, constraint.sqrt_information),
                                  nullptr, from);
    endpoints = {constraint.from};
    prior_residuals_.insert(residual_id);
  } else {
    CHECK(blocks_.count(*constraint.to) > 0) << "add the variable before its constraints";
    if (IsFrozen(constraint.from) && IsFrozen(*constraint.to)) {
      return;
    }
    double* to = blocks_.at(*constraint.to).data();
    ceres::LossFunction* loss =
        constraint.type == Constraint::Type::INTER_SUBMAP ? &inter_submap_loss_ : nullptr;
    residual_id =
        problem_.AddResidualBlock(utils::cost_functor::CreateSpaRelativePoseCostFunction(
                                      constraint.relative_pose, constraint.sqrt_information),
                                  loss, from, to);
    endpoints = {constraint.from, *constraint.to};
  }

  for (const VariableId& endpoint : endpoints) {
    residuals_by_variable_[endpoint].insert(residual_id);
  }
  residual_endpoints_.emplace(residual_id, std::move(endpoints));
}

void Optimization::FreezeSession(SessionId id) {
  frozen_sessions_.insert(id);

  std::vector<ceres::ResidualBlockId> internal;
  for (const auto& [residual_id, endpoints] : residual_endpoints_) {
    const bool all_inside = std::all_of(endpoints.begin(), endpoints.end(),
                                        [id](const VariableId& v) { return v.session() == id; });
    if (all_inside) {
      internal.push_back(residual_id);
    }
  }
  for (const ceres::ResidualBlockId residual_id : internal) {
    problem_.RemoveResidualBlock(residual_id);
    RemoveResidual(residual_id);
  }

  for (auto& [variable_id, block] : blocks_) {
    if (variable_id.session() != id) {
      continue;
    }
    gauge_variables_.erase(variable_id);
    problem_.SetParameterBlockConstant(block.data());
  }
}

void Optimization::RemoveVariable(const VariableId& id) {
  const auto block_it = blocks_.find(id);
  CHECK(block_it != blocks_.end()) << "removing a variable that was never added";

  // RemoveParameterBlock takes the residuals with it; purge them or they go to Ceres twice.
  const auto residuals_it = residuals_by_variable_.find(id);
  if (residuals_it != residuals_by_variable_.end()) {
    const std::vector<ceres::ResidualBlockId> residual_ids(residuals_it->second.begin(),
                                                           residuals_it->second.end());
    for (const ceres::ResidualBlockId residual_id : residual_ids) {
      RemoveResidual(residual_id);
    }
  }

  problem_.RemoveParameterBlock(block_it->second.data());
  residuals_by_variable_.erase(id);
  gauge_variables_.erase(id);
  blocks_.erase(block_it);
}

void Optimization::RenameVariable(const VariableId& old_id, const VariableId& new_id) {
  CHECK(blocks_.count(new_id) == 0) << "renaming onto an id that already exists";
  auto block = blocks_.extract(old_id);
  CHECK(!block.empty()) << "renaming a variable that was never added";
  CHECK(!problem_.IsParameterBlockConstant(block.mapped().data()))
      << "a constant block (frozen truth or a gauge datum) cannot change sessions";
  block.key() = new_id;
  blocks_.insert(std::move(block));

  auto residuals = residuals_by_variable_.extract(old_id);
  if (!residuals.empty()) {
    for (const ceres::ResidualBlockId residual_id : residuals.mapped()) {
      for (VariableId& endpoint : residual_endpoints_.at(residual_id)) {
        if (endpoint == old_id) {
          endpoint = new_id;
        }
      }
    }
    residuals.key() = new_id;
    residuals_by_variable_.insert(std::move(residuals));
  }
}

void Optimization::BuildFrom(const PoseGraphData& graph) {
  problem_ = ceres::Problem(MakeProblemOptions());
  blocks_.clear();
  residuals_by_variable_.clear();
  residual_endpoints_.clear();
  prior_residuals_.clear();
  frozen_sessions_.clear();
  gauge_variables_.clear();

  for (const auto& [session_id, session] : graph.sessions()) {
    if (session.frozen()) {
      frozen_sessions_.insert(session_id);
    }
  }
  for (const auto& [submap_id, record] : graph.submaps()) {
    AddVariable(VariableId::Of(submap_id), record.global_pose);
  }
  for (const auto& [node_id, node] : graph.nodes()) {
    AddVariable(VariableId::Of(node_id), node.global_pose);
  }
  for (const Constraint& constraint : graph.constraints()) {
    AddConstraint(constraint);
  }
}

void Optimization::Optimize(PoseGraphData& graph) {
  if (blocks_.empty()) {
    return;
  }
  SetGaugeConstraints();

  ceres::Solver::Options options;
  options.max_num_iterations = option_.max_num_iterations;
  options.num_threads = option_.num_threads;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.function_tolerance = 1e-14;
  options.gradient_tolerance = 1e-16;
  options.parameter_tolerance = 1e-14;
  options.use_nonmonotonic_steps = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  ++num_solves_;
  VLOG(1) << summary.BriefReport();

  for (auto& [variable_id, block] : blocks_) {
    if (problem_.IsParameterBlockConstant(block.data())) {
      continue;
    }
    block[2] = transform::NormalizeAngle(block[2]);
  }
  WriteBack(graph);
}

bool Optimization::HasVariable(const VariableId& id) const { return blocks_.count(id) > 0; }

bool Optimization::IsVariableConstant(const VariableId& id) const {
  const auto it = blocks_.find(id);
  CHECK(it != blocks_.end()) << "unknown variable";
  return problem_.IsParameterBlockConstant(it->second.data());
}

bool Optimization::IsSessionAnchoredToFrozen(SessionId id) const {
  if (frozen_sessions_.empty()) {
    return false;
  }
  for (const VariableId& datum : gauge_variables_) {
    if (datum.session() == id) {
      return false;
    }
  }
  return true;
}

Eigen::Affine2d Optimization::GetVariablePose(const VariableId& id) const {
  const auto it = blocks_.find(id);
  CHECK(it != blocks_.end()) << "unknown variable";
  return transform::FromArray3(it->second);
}

double* Optimization::mutable_parameter_block(const VariableId& id) {
  const auto it = blocks_.find(id);
  CHECK(it != blocks_.end()) << "unknown variable";
  return it->second.data();
}

void Optimization::SetGaugeConstraints() {
  for (const VariableId& id : gauge_variables_) {
    const auto it = blocks_.find(id);
    if (it != blocks_.end() && !IsFrozen(id)) {
      problem_.SetParameterBlockVariable(it->second.data());
    }
  }
  gauge_variables_.clear();

  std::vector<VariableId> ids;
  ids.reserve(blocks_.size());
  std::map<VariableId, size_t> index_of;
  for (const auto& [id, block] : blocks_) {
    index_of.emplace(id, ids.size());
    ids.push_back(id);
  }

  ComponentSet components(ids.size());
  for (const auto& [residual_id, endpoints] : residual_endpoints_) {
    for (size_t i = 1; i < endpoints.size(); ++i) {
      components.Union(index_of.at(endpoints[0]), index_of.at(endpoints[i]));
    }
  }

  // A frozen pose or a trimmer prior already pins the component.
  std::set<size_t> anchored;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (problem_.IsParameterBlockConstant(blocks_.at(ids[i]).data())) {
      anchored.insert(components.Find(i));
    }
  }
  for (const ceres::ResidualBlockId residual_id : prior_residuals_) {
    const VariableId& id = residual_endpoints_.at(residual_id).front();
    anchored.insert(components.Find(index_of.at(id)));
  }

  // Submaps before nodes: a submap moves a whole neighbourhood with it.
  std::map<size_t, VariableId> datum;
  for (size_t i = 0; i < ids.size(); ++i) {
    const size_t root = components.Find(i);
    if (anchored.count(root) > 0) {
      continue;
    }
    const auto it = datum.find(root);
    if (it == datum.end()) {
      datum.emplace(root, ids[i]);
      continue;
    }
    const bool candidate_is_submap = ids[i].kind == VariableId::Kind::SUBMAP;
    const bool current_is_submap = it->second.kind == VariableId::Kind::SUBMAP;
    if (candidate_is_submap && !current_is_submap) {
      it->second = ids[i];
    }
  }

  for (const auto& [root, id] : datum) {
    problem_.SetParameterBlockConstant(blocks_.at(id).data());
    gauge_variables_.insert(id);
  }
}

void Optimization::WriteBack(PoseGraphData& graph) {
  for (const auto& [id, block] : blocks_) {
    if (problem_.IsParameterBlockConstant(block.data())) {
      continue;
    }
    const Eigen::Affine2d pose = transform::FromArray3(block);
    if (id.kind == VariableId::Kind::NODE) {
      graph.SetNodeGlobalPose(id.node_id(), pose);
    } else {
      graph.SetSubmapGlobalPose(id.submap_id(), pose);
    }
  }
}

void Optimization::RemoveResidual(ceres::ResidualBlockId residual_id) {
  const auto it = residual_endpoints_.find(residual_id);
  CHECK(it != residual_endpoints_.end());
  for (const VariableId& endpoint : it->second) {
    const auto by_variable = residuals_by_variable_.find(endpoint);
    if (by_variable != residuals_by_variable_.end()) {
      by_variable->second.erase(residual_id);
    }
  }
  prior_residuals_.erase(residual_id);
  residual_endpoints_.erase(it);
}

bool Optimization::IsFrozen(const VariableId& id) const {
  return frozen_sessions_.count(id.session()) > 0;
}

}  // namespace evergreenslam::lifelong
