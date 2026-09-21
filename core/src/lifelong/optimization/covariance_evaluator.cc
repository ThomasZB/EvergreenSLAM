/**
 * @file covariance_evaluator.cc
 * @author hang chen (chen@hang.plus)
 * @brief The two covariances the pose graph asks for, kept apart on purpose.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/covariance_evaluator.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <map>
#include <set>
#include <utility>

namespace evergreenslam::lifelong {
namespace {

constexpr int kDim = 3;

ceres::Covariance::Options MakeCovarianceOptions() {
  ceres::Covariance::Options options;
  options.algorithm_type = ceres::SPARSE_QR;
  options.num_threads = 1;
  return options;
}

// Puts back exactly the constancy the Problem had, whichever way the evaluation exits.
class ConstancyGuard {
 public:
  ConstancyGuard(ceres::Problem& problem, std::vector<double*> made_constant)
      : problem_(problem), made_constant_(std::move(made_constant)) {}

  ~ConstancyGuard() {
    for (double* block : made_constant_) {
      problem_.SetParameterBlockVariable(block);
    }
  }

  ConstancyGuard(const ConstancyGuard&) = delete;
  ConstancyGuard& operator=(const ConstancyGuard&) = delete;

 private:
  ceres::Problem& problem_;
  std::vector<double*> made_constant_;
};

}  // namespace

CovarianceEvaluator::CovarianceEvaluator(Optimization& optimization)
    : optimization_(optimization) {}

std::optional<Eigen::Matrix3d> CovarianceEvaluator::ComputeMarginalCovarianceInGlobal(
    const VariableId& id) {
  CHECK(optimization_.HasVariable(id)) << "unknown variable";
  ceres::Problem& problem = optimization_.mutable_problem();
  const double* block = optimization_.mutable_parameter_block(id);

  ceres::Covariance covariance(MakeCovarianceOptions());
  const std::vector<std::pair<const double*, const double*>> blocks = {{block, block}};
  if (!covariance.Compute(blocks, &problem)) {
    LOG(WARNING) << "marginal covariance is unavailable: the pose graph did not factorize";
    return std::nullopt;
  }

  Eigen::Matrix<double, kDim, kDim, Eigen::RowMajor> row_major;
  covariance.GetCovarianceBlock(block, block, row_major.data());
  return Eigen::Matrix3d(row_major);
}

std::optional<std::map<VariableId, Eigen::Matrix3d>>
CovarianceEvaluator::ComputeMarginalCovariancesInGlobal(const std::vector<VariableId>& ids) {
  std::map<VariableId, Eigen::Matrix3d> result;
  if (ids.empty()) {
    return result;
  }
  ceres::Problem& problem = optimization_.mutable_problem();

  std::map<VariableId, const double*> blocks_by_id;
  for (const VariableId& id : ids) {
    CHECK(optimization_.HasVariable(id)) << "unknown variable";
    blocks_by_id.emplace(id, optimization_.mutable_parameter_block(id));
  }

  std::vector<std::pair<const double*, const double*>> requested;
  requested.reserve(blocks_by_id.size());
  for (const auto& [id, block] : blocks_by_id) {
    requested.emplace_back(block, block);
  }

  ceres::Covariance covariance(MakeCovarianceOptions());
  if (!covariance.Compute(requested, &problem)) {
    LOG(WARNING) << "marginal covariances are unavailable: the pose graph did not factorize";
    return std::nullopt;
  }

  Eigen::Matrix<double, kDim, kDim, Eigen::RowMajor> row_major;
  for (const auto& [id, block] : blocks_by_id) {
    covariance.GetCovarianceBlock(block, block, row_major.data());
    result.emplace(id, Eigen::Matrix3d(row_major));
  }
  return result;
}

std::optional<Eigen::MatrixXd> CovarianceEvaluator::ComputeConditionalJointCovarianceInBlanket(
    const std::vector<VariableId>& blanket) {
  CHECK(!blanket.empty());
  ceres::Problem& problem = optimization_.mutable_problem();

  std::vector<double*> blanket_blocks;
  blanket_blocks.reserve(blanket.size());
  std::set<const double*> inside;
  for (const VariableId& id : blanket) {
    CHECK(optimization_.HasVariable(id)) << "unknown variable in the blanket";
    double* block = optimization_.mutable_parameter_block(id);
    CHECK(inside.insert(block).second) << "the blanket lists a variable twice";
    blanket_blocks.push_back(block);
  }

  std::vector<double*> all_blocks;
  problem.GetParameterBlocks(&all_blocks);
  std::vector<double*> made_constant;
  for (double* block : all_blocks) {
    if (inside.count(block) > 0 || problem.IsParameterBlockConstant(block)) {
      continue;
    }
    problem.SetParameterBlockConstant(block);
    made_constant.push_back(block);
  }
  const ConstancyGuard guard(problem, made_constant);

  std::vector<std::pair<const double*, const double*>> requested;
  requested.reserve(blanket_blocks.size() * (blanket_blocks.size() + 1) / 2);
  for (size_t row = 0; row < blanket_blocks.size(); ++row) {
    for (size_t col = row; col < blanket_blocks.size(); ++col) {
      requested.emplace_back(blanket_blocks[row], blanket_blocks[col]);
    }
  }

  ceres::Covariance covariance(MakeCovarianceOptions());
  if (!covariance.Compute(requested, &problem)) {
    LOG(WARNING) << "conditional covariance is unavailable: the blanket did not factorize";
    return std::nullopt;
  }

  const int size = static_cast<int>(blanket_blocks.size()) * kDim;
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(size, size);
  Eigen::Matrix<double, kDim, kDim, Eigen::RowMajor> row_major;
  for (size_t row = 0; row < blanket_blocks.size(); ++row) {
    for (size_t col = row; col < blanket_blocks.size(); ++col) {
      covariance.GetCovarianceBlock(blanket_blocks[row], blanket_blocks[col], row_major.data());
      result.block<kDim, kDim>(static_cast<int>(row) * kDim, static_cast<int>(col) * kDim) =
          row_major;
      if (row != col) {
        result.block<kDim, kDim>(static_cast<int>(col) * kDim, static_cast<int>(row) * kDim) =
            row_major.transpose();
      }
    }
  }
  return result;
}

}  // namespace evergreenslam::lifelong
