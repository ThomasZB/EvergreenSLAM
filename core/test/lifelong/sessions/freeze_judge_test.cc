/**
 * @file freeze_judge_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The freeze criterion: bootstrap, the anchor, then the covariances.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/freeze_judge.h"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

const Eigen::Vector3d kStrong(1000.0, 1000.0, 1000.0);
// x is what gives: sqrt(1 / (2 * 10^2)) = 0.07 m between two strong anchors.
const Eigen::Vector3d kWeak(10.0, 1000.0, 1000.0);

// A pose graph and the Problem that mirrors it, driven directly so a test can dictate the exact
// covariance a submap ends up with.
class Fixture {
 public:
  SessionId StartSession() { return graph_.StartNewSession(TestTime(0)); }

  SubmapId AddSubmap(SessionId session, int index, const Eigen::Affine2d& global_pose) {
    SubmapRecord record;
    record.id = SubmapId{session.session_index, index};
    record.local_pose = Eigen::Affine2d::Identity();
    record.global_pose = global_pose;
    graph_.AddSubmap(record);
    return record.id;
  }

  SubmapId AddUnfinishedSubmap(SessionId session, int index, const Eigen::Affine2d& global_pose) {
    SubmapRecord record;
    record.id = SubmapId{session.session_index, index};
    record.submap = std::make_shared<mapping::Submap>(index, global_pose, 0.05);
    record.local_pose = Eigen::Affine2d::Identity();
    record.global_pose = global_pose;
    graph_.AddSubmap(record);
    return record.id;
  }

  void Link(const SubmapId& from, const SubmapId& to, const Eigen::Vector3d& sqrt_information) {
    Constraint constraint;
    constraint.type = Constraint::Type::INTER_SUBMAP;
    constraint.from = VariableId::Of(from);
    constraint.to = VariableId::Of(to);
    constraint.relative_pose =
        graph_.submap(from).global_pose.inverse() * graph_.submap(to).global_pose;
    constraint.sqrt_information = Eigen::Matrix3d(sqrt_information.asDiagonal());
    graph_.AddConstraint(constraint);
  }

  void Freeze(SessionId session) {
    graph_.FreezeSession(session);
    optimization_.FreezeSession(session);
  }

  void Build() { optimization_.BuildFrom(graph_); }
  void Solve() { optimization_.Optimize(graph_); }

  FreezeVerdict Judge(const FreezeJudge& judge, SessionId session) {
    return judge.Judge(graph_, optimization_, evaluator_, session);
  }

 private:
  PoseGraphData graph_;
  Optimization optimization_;
  CovarianceEvaluator evaluator_{optimization_};
};

// A frozen base and a three-submap chain next to it, solved; the caller adds the links.
struct ChainBesideBase {
  SessionId base;
  SubmapId anchor;
  SessionId session;
  std::vector<SubmapId> ids;
};

ChainBesideBase MakeChainBesideAFrozenBase(Fixture& fixture, const Eigen::Vector3d& chain_weights,
                                           bool link_head, bool link_tail) {
  ChainBesideBase out;
  out.base = fixture.StartSession();
  out.anchor = fixture.AddSubmap(out.base, 0, transform::FromXYTheta(0.0, 9.0, 0.0));
  out.session = fixture.StartSession();
  for (int i = 0; i < 3; ++i) {
    out.ids.push_back(fixture.AddSubmap(out.session, i, transform::FromXYTheta(i * 1.0, 0.0, 0.0)));
  }
  for (int i = 1; i < 3; ++i) {
    fixture.Link(out.ids[i - 1], out.ids[i], chain_weights);
  }
  if (link_head) {
    fixture.Link(out.anchor, out.ids.front(), kStrong);
  }
  if (link_tail) {
    fixture.Link(out.anchor, out.ids.back(), kStrong);
  }
  fixture.Build();
  fixture.Freeze(out.base);
  fixture.Solve();
  return out;
}

TEST(FreezeJudgeTest, BootstrapSessionIsEligibleWithOneFinishedSubmap) {
  Fixture fixture;
  const SessionId session = fixture.StartSession();
  const SubmapId first = fixture.AddSubmap(session, 0, transform::FromXYTheta(0.0, 0.0, 0.0));
  const SubmapId second =
      fixture.AddUnfinishedSubmap(session, 1, transform::FromXYTheta(1.0, 0.0, 0.0));
  fixture.Link(first, second, kWeak);
  fixture.Build();
  fixture.Solve();

  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, session);
  EXPECT_TRUE(verdict.eligible) << ToString(verdict.rejection);
  EXPECT_TRUE(verdict.bootstrap);
  EXPECT_EQ(verdict.rejection, FreezeRejection::NONE);
  EXPECT_TRUE(verdict.submaps.empty()) << "bootstrap is structural, no covariance is read";
}

// The bootstrap exception has to survive a graph with two sessions and nothing frozen.
TEST(FreezeJudgeTest, TheFrameDefiningSessionMayCarryTheDatumUntilSomethingIsFrozen) {
  Fixture fixture;
  const SessionId first = fixture.StartSession();
  fixture.AddSubmap(first, 0, transform::FromXYTheta(0.0, 0.0, 0.0));
  const SessionId second = fixture.StartSession();
  fixture.AddSubmap(second, 0, transform::FromXYTheta(0.0, 9.0, 0.0));
  fixture.Build();
  fixture.Solve();

  const FreezeJudge judge;
  EXPECT_TRUE(fixture.Judge(judge, first).bootstrap);
  EXPECT_TRUE(fixture.Judge(judge, second).bootstrap);

  // Once one of them is truth, the other has to earn its place in that frame.
  fixture.Freeze(first);
  fixture.Solve();
  const FreezeVerdict verdict = fixture.Judge(judge, second);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_FALSE(verdict.bootstrap);
  EXPECT_EQ(verdict.rejection, FreezeRejection::NOT_ANCHORED);
}

TEST(FreezeJudgeTest, NoFrozenLinkAtAllIsNotAnchored) {
  Fixture fixture;
  const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kStrong, false, false);
  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.rejection, FreezeRejection::NOT_ANCHORED);
  for (const SubmapId& id : chain.ids) {
    EXPECT_EQ(verdict.frozen_links.at(id), 0);
  }
  EXPECT_TRUE(verdict.submaps.empty()) << "no covariance is read before the session is anchored";
}

// One frozen link anywhere anchors the session; the covariance gate then decides on its own.
TEST(FreezeJudgeTest, ALinkAtOneEndAnchorsAndRunsTheCovarianceGate) {
  const FreezeJudge judge;
  {
    Fixture fixture;
    const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kStrong, true, false);
    const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
    EXPECT_TRUE(verdict.eligible) << ToString(verdict.rejection);
    EXPECT_EQ(verdict.rejection, FreezeRejection::NONE);
    EXPECT_EQ(verdict.frozen_links.at(chain.ids[0]), 1);
    EXPECT_EQ(verdict.frozen_links.at(chain.ids[1]), 0);
    EXPECT_EQ(verdict.frozen_links.at(chain.ids[2]), 0);
    ASSERT_EQ(verdict.submaps.size(), 3u) << "anchored: the covariance is read";
    for (const SubmapUncertainty& uncertainty : verdict.submaps) {
      EXPECT_TRUE(uncertainty.passed);
    }
  }
  {
    Fixture fixture;
    const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kStrong, false, true);
    const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
    EXPECT_TRUE(verdict.eligible) << ToString(verdict.rejection);
    EXPECT_EQ(verdict.frozen_links.at(chain.ids[2]), 1);
    EXPECT_EQ(verdict.submaps.size(), 3u);
  }
  {
    Fixture fixture;
    const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kWeak, true, false);
    const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
    EXPECT_FALSE(verdict.eligible);
    EXPECT_EQ(verdict.rejection, FreezeRejection::COVARIANCE_TOO_LARGE);
    ASSERT_EQ(verdict.submaps.size(), 3u);
    EXPECT_TRUE(verdict.submaps[0].passed);
    EXPECT_FALSE(verdict.submaps[1].passed) << "one weak edge from the anchor";
    EXPECT_FALSE(verdict.submaps[2].passed) << "two weak edges from the anchor";
  }
}

TEST(FreezeJudgeTest, AnAnchoredSessionWithOneWeakSubmapFailsTheCovarianceGate) {
  Fixture fixture;
  const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kWeak, true, true);
  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.rejection, FreezeRejection::COVARIANCE_TOO_LARGE);
  ASSERT_EQ(verdict.submaps.size(), 3u);
  EXPECT_TRUE(verdict.submaps[0].passed);
  EXPECT_FALSE(verdict.submaps[1].passed);
  EXPECT_NEAR(verdict.submaps[1].stddev.x(), 0.0707, 1e-3);
  EXPECT_TRUE(verdict.submaps[2].passed);
}

TEST(FreezeJudgeTest, AnAnchoredSessionWithGoodCovarianceIsEligible) {
  Fixture fixture;
  const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kStrong, true, true);
  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
  EXPECT_TRUE(verdict.eligible) << ToString(verdict.rejection);
  EXPECT_FALSE(verdict.bootstrap);
  EXPECT_EQ(verdict.frozen_links.at(chain.ids[1]), 0);
  ASSERT_EQ(verdict.submaps.size(), 3u);
  for (const SubmapUncertainty& uncertainty : verdict.submaps) {
    EXPECT_TRUE(uncertainty.covariance_available);
    EXPECT_TRUE(uncertainty.passed);
  }
}

// The Problem knows nothing of a submap the graph gained after its last build: that covariance
// cannot be read, and the verdict says so instead of passing the submap silently.
TEST(FreezeJudgeTest, ASubmapMissingFromTheProblemMakesTheCovarianceUnavailable) {
  Fixture fixture;
  const ChainBesideBase chain = MakeChainBesideAFrozenBase(fixture, kStrong, true, true);
  const SubmapId late = fixture.AddSubmap(chain.session, 3, transform::FromXYTheta(3.0, 0.0, 0.0));
  fixture.Link(chain.anchor, late, kStrong);

  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, chain.session);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.rejection, FreezeRejection::COVARIANCE_UNAVAILABLE);
  ASSERT_EQ(verdict.submaps.size(), 4u);
  EXPECT_TRUE(verdict.submaps[0].covariance_available);
  EXPECT_EQ(verdict.submaps[3].id, late);
  EXPECT_FALSE(verdict.submaps[3].covariance_available);
  EXPECT_FALSE(verdict.submaps[3].passed);
}

TEST(FreezeJudgeTest, AFrozenSessionIsNeverEligibleAgain) {
  Fixture fixture;
  const SessionId session = fixture.StartSession();
  fixture.AddSubmap(session, 0, transform::FromXYTheta(0.0, 0.0, 0.0));
  fixture.Build();
  fixture.Solve();
  const FreezeJudge judge;
  ASSERT_TRUE(fixture.Judge(judge, session).eligible);

  fixture.Freeze(session);
  const FreezeVerdict verdict = fixture.Judge(judge, session);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.rejection, FreezeRejection::NOT_ACTIVE);
}

TEST(FreezeJudgeTest, ASessionWhoseSubmapsAreAllUnfinishedIsNotEligible) {
  Fixture fixture;
  const SessionId session = fixture.StartSession();
  const SubmapId first =
      fixture.AddUnfinishedSubmap(session, 0, transform::FromXYTheta(0.0, 0.0, 0.0));
  const SubmapId second =
      fixture.AddUnfinishedSubmap(session, 1, transform::FromXYTheta(1.0, 0.0, 0.0));
  fixture.Link(first, second, kStrong);
  fixture.Build();
  fixture.Solve();

  const FreezeJudge judge;
  const FreezeVerdict verdict = fixture.Judge(judge, session);
  EXPECT_FALSE(verdict.eligible);
  EXPECT_EQ(verdict.rejection, FreezeRejection::NO_FINISHED_SUBMAP);
}

TEST(FreezeJudgeTest, OptionRoundTripsThroughYaml) {
  const YAML::Node node = YAML::Load("max_translation_stddev: 0.07\nmax_rotation_stddev: 0.011\n");
  const FreezeJudgeOption option = LoadFreezeJudgeOption(node);
  EXPECT_DOUBLE_EQ(option.max_translation_stddev, 0.07);
  EXPECT_DOUBLE_EQ(option.max_rotation_stddev, 0.011);

  const FreezeJudgeOption defaults = LoadFreezeJudgeOption(YAML::Load("{}"));
  EXPECT_DOUBLE_EQ(defaults.max_translation_stddev, FreezeJudgeOption().max_translation_stddev);
}

}  // namespace
}  // namespace evergreenslam::lifelong
