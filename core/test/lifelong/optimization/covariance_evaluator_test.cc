/**
 * @file covariance_evaluator_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The two covariance meanings, and the caveat that keeps them apart (decision 9).
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/covariance_evaluator.h"

#include <gtest/gtest.h>

#include <vector>

#include "common/time.h"
#include "lifelong/optimization/optimization.h"
#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph_data.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr int kChainLength = 6;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

Constraint MakeRelative(const VariableId& from, const VariableId& to,
                        const Eigen::Affine2d& relative_pose) {
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = from;
  constraint.to = to;
  constraint.relative_pose = relative_pose;
  constraint.sqrt_information = OdometrySqrtInformation(ConstraintWeightOption());
  return constraint;
}

// A straight chain of submaps one metre apart, at its solution, with only the datum holding it.
struct Chain {
  PoseGraphData graph;
  SessionId session;
  std::vector<SubmapId> ids;
};

Chain MakeChain() {
  Chain chain;
  chain.session = chain.graph.StartNewSession(TestTime(0));
  for (int i = 0; i < kChainLength; ++i) {
    SubmapRecord record;
    record.id = SubmapId{chain.session.session_index, i};
    record.local_pose = transform::FromXYTheta(static_cast<double>(i), 0.0, 0.0);
    record.global_pose = record.local_pose;
    chain.graph.AddSubmap(record);
    chain.ids.push_back(record.id);
    if (i > 0) {
      chain.graph.AddConstraint(MakeRelative(VariableId::Of(chain.ids[i - 1]),
                                             VariableId::Of(record.id),
                                             transform::FromXYTheta(1.0, 0.0, 0.0)));
    }
  }
  return chain;
}

TEST(CovarianceEvaluator, MarginalCovarianceGrowsAlongAnUnanchoredChain) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  CovarianceEvaluator evaluator(optimization);

  const std::optional<Eigen::Matrix3d> datum =
      evaluator.ComputeMarginalCovarianceInGlobal(VariableId::Of(chain.ids.front()));
  ASSERT_TRUE(datum.has_value());
  EXPECT_NEAR(datum->trace(), 0.0, 1e-15) << "the datum of a component carries no uncertainty";

  double previous = 0.0;
  for (int i = 1; i < kChainLength; ++i) {
    const std::optional<Eigen::Matrix3d> covariance =
        evaluator.ComputeMarginalCovarianceInGlobal(VariableId::Of(chain.ids[i]));
    ASSERT_TRUE(covariance.has_value());
    EXPECT_GT(covariance->trace(), previous) << "at chain index " << i;
    previous = covariance->trace();
  }
}

TEST(CovarianceEvaluator, MarginalCovarianceShrinksOnceTheChainIsClosed) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  CovarianceEvaluator evaluator(optimization);

  const VariableId middle = VariableId::Of(chain.ids[kChainLength / 2]);
  const std::optional<Eigen::Matrix3d> before = evaluator.ComputeMarginalCovarianceInGlobal(middle);
  ASSERT_TRUE(before.has_value());

  const Constraint closure =
      MakeRelative(VariableId::Of(chain.ids.front()), VariableId::Of(chain.ids.back()),
                   transform::FromXYTheta(static_cast<double>(kChainLength - 1), 0.0, 0.0));
  chain.graph.AddConstraint(closure);
  optimization.AddConstraint(closure);
  optimization.Optimize(chain.graph);

  const std::optional<Eigen::Matrix3d> after = evaluator.ComputeMarginalCovarianceInGlobal(middle);
  ASSERT_TRUE(after.has_value());
  EXPECT_LT(after->trace(), before->trace());
}

TEST(CovarianceEvaluator, AFrozenVariableHasNoMarginalUncertainty) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  chain.graph.FreezeSession(chain.session);
  optimization.FreezeSession(chain.session);
  CovarianceEvaluator evaluator(optimization);

  const std::optional<Eigen::Matrix3d> covariance =
      evaluator.ComputeMarginalCovarianceInGlobal(VariableId::Of(chain.ids.back()));
  ASSERT_TRUE(covariance.has_value());
  EXPECT_NEAR(covariance->trace(), 0.0, 1e-15);
}

TEST(CovarianceEvaluator, BatchMarginalsMatchTheOneAtATimeReadings) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  CovarianceEvaluator evaluator(optimization);

  // The datum is in the batch on purpose: the constant block semantics must match too.
  std::vector<VariableId> ids;
  for (const SubmapId& id : chain.ids) {
    ids.push_back(VariableId::Of(id));
  }
  const auto batch = evaluator.ComputeMarginalCovariancesInGlobal(ids);
  ASSERT_TRUE(batch.has_value());
  ASSERT_EQ(batch->size(), ids.size());

  for (size_t i = 0; i < ids.size(); ++i) {
    const std::optional<Eigen::Matrix3d> single =
        evaluator.ComputeMarginalCovarianceInGlobal(ids[i]);
    ASSERT_TRUE(single.has_value());
    ASSERT_EQ(batch->count(ids[i]), 1u) << "chain index " << i;
    EXPECT_TRUE(((batch->at(ids[i]) - *single).array().abs() <= 1e-12).all())
        << "chain index " << i;
  }
}

// Fixing everything outside the blanket removes real uncertainty, so the conditional reading is
// systematically optimistic: comparable across candidates, never against a freeze threshold.
TEST(CovarianceEvaluator, ConditionalCovarianceUnderestimatesTheMarginalOne) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  CovarianceEvaluator evaluator(optimization);

  // The datum stays out of the blanket: it is the frame everything else is measured against.
  const std::vector<VariableId> blanket = {VariableId::Of(chain.ids[2]),
                                           VariableId::Of(chain.ids[3])};
  const std::optional<Eigen::MatrixXd> joint =
      evaluator.ComputeConditionalJointCovarianceInBlanket(blanket);
  ASSERT_TRUE(joint.has_value());
  ASSERT_EQ(joint->rows(), 6);
  ASSERT_EQ(joint->cols(), 6);
  EXPECT_NEAR((*joint - joint->transpose()).norm(), 0.0, 1e-15);

  for (size_t i = 0; i < blanket.size(); ++i) {
    const std::optional<Eigen::Matrix3d> marginal =
        evaluator.ComputeMarginalCovarianceInGlobal(blanket[i]);
    ASSERT_TRUE(marginal.has_value());
    const Eigen::Matrix3d conditional =
        joint->block<3, 3>(static_cast<int>(i) * 3, static_cast<int>(i) * 3);
    EXPECT_LT(conditional.trace(), marginal->trace()) << "blanket entry " << i;
    // Matrix sense, not just the trace: marginal - conditional is positive semidefinite.
    const Eigen::Matrix3d difference = *marginal - conditional;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(difference);
    EXPECT_GT(solver.eigenvalues().minCoeff(), -1e-12) << "blanket entry " << i;
  }
}

TEST(CovarianceEvaluator, TheBlanketEvaluationPutsConstancyBack) {
  Chain chain = MakeChain();
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);
  CovarianceEvaluator evaluator(optimization);

  std::vector<bool> before;
  for (const SubmapId& id : chain.ids) {
    before.push_back(optimization.IsVariableConstant(VariableId::Of(id)));
  }

  const std::vector<VariableId> blanket = {VariableId::Of(chain.ids[2]),
                                           VariableId::Of(chain.ids[3])};
  ASSERT_TRUE(evaluator.ComputeConditionalJointCovarianceInBlanket(blanket).has_value());

  for (size_t i = 0; i < chain.ids.size(); ++i) {
    EXPECT_EQ(optimization.IsVariableConstant(VariableId::Of(chain.ids[i])), before[i])
        << "variable " << i;
  }
  EXPECT_TRUE(before.front()) << "the datum was constant to begin with";
}

}  // namespace
}  // namespace evergreenslam::lifelong
