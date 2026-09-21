/**
 * @file constraint_sampler_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/constraint_sampler.h"

#include <gtest/gtest.h>

#include <vector>

namespace evergreenslam::lifelong {
namespace {

LoopCandidate Candidate(int session, int node_index, int submap_index, double distance,
                        bool cross_session) {
  return LoopCandidate{NodeId{session, node_index}, SubmapId{session, submap_index}, distance, 0.0,
                       cross_session};
}

TEST(ConstraintSampler, CrossSessionComesFirstThenDistance) {
  ConstraintSampler sampler;
  std::vector<LoopCandidate> candidates;
  candidates.push_back(Candidate(0, 0, 0, 1.0, false));
  candidates.push_back(Candidate(0, 1, 1, 5.0, true));
  candidates.push_back(Candidate(0, 2, 2, 2.0, true));
  candidates.push_back(Candidate(0, 3, 3, 0.5, false));

  const std::vector<LoopCandidate> selected = sampler.Select(candidates);
  ASSERT_EQ(selected.size(), 4u);
  EXPECT_EQ(selected[0].node_id.node_index, 2);
  EXPECT_EQ(selected[1].node_id.node_index, 1);
  EXPECT_EQ(selected[2].node_id.node_index, 3);
  EXPECT_EQ(selected[3].node_id.node_index, 0);
}

TEST(ConstraintSampler, EqualKeysBreakTiesOnIdsDeterministically) {
  ConstraintSampler sampler;
  std::vector<LoopCandidate> candidates;
  candidates.push_back(Candidate(0, 7, 3, 1.0, false));
  candidates.push_back(Candidate(0, 7, 1, 1.0, false));
  candidates.push_back(Candidate(0, 2, 9, 1.0, false));

  const std::vector<LoopCandidate> selected = sampler.Select(candidates);
  ASSERT_EQ(selected.size(), 3u);
  EXPECT_EQ(selected[0].node_id.node_index, 2);
  EXPECT_EQ(selected[1].node_id.node_index, 7);
  EXPECT_EQ(selected[1].submap_id.submap_index, 1);
  EXPECT_EQ(selected[2].submap_id.submap_index, 3);
}

TEST(ConstraintSampler, TruncatesToTheDefaultBudget) {
  ConstraintSamplerOption option;
  option.max_matches_per_round = 2;
  ConstraintSampler sampler(option);

  std::vector<LoopCandidate> candidates;
  for (int i = 0; i < 5; ++i) {
    candidates.push_back(Candidate(0, i, i, static_cast<double>(i), false));
  }
  const std::vector<LoopCandidate> selected = sampler.Select(candidates);
  ASSERT_EQ(selected.size(), 2u);
  EXPECT_EQ(selected[0].node_id.node_index, 0);
  EXPECT_EQ(selected[1].node_id.node_index, 1);
}

TEST(ConstraintSampler, PerRoundBudgetOverridesTheDefault) {
  ConstraintSamplerOption option;
  option.max_matches_per_round = 2;
  ConstraintSampler sampler(option);

  std::vector<LoopCandidate> candidates;
  for (int i = 0; i < 5; ++i) {
    candidates.push_back(Candidate(0, i, i, static_cast<double>(i), false));
  }
  EXPECT_EQ(sampler.Select(candidates, 4).size(), 4u);
  EXPECT_EQ(sampler.Select(candidates, 0).size(), 0u);
  EXPECT_EQ(sampler.Select(candidates, 100).size(), 5u);
}

TEST(ConstraintSampler, RatioGateIsDeterministicAndCounterBased) {
  ConstraintSamplerOption half;
  half.sampling_ratio = 0.5;
  ConstraintSampler sampler(half);
  std::vector<bool> decisions;
  for (int i = 0; i < 6; ++i) {
    decisions.push_back(sampler.ShouldSampleRound());
  }
  const std::vector<bool> expected{true, false, true, false, true, false};
  EXPECT_EQ(decisions, expected);
}

TEST(ConstraintSampler, RatioOneAlwaysSamplesAndRatioZeroNever) {
  ConstraintSampler always;
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(always.ShouldSampleRound());
  }

  ConstraintSamplerOption zero;
  zero.sampling_ratio = 0.0;
  ConstraintSampler never(zero);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(never.ShouldSampleRound());
  }
}

TEST(ConstraintSampler, QuarterRatioSamplesOneInFour) {
  ConstraintSamplerOption quarter;
  quarter.sampling_ratio = 0.25;
  ConstraintSampler sampler(quarter);
  int sampled = 0;
  for (int i = 0; i < 100; ++i) {
    sampled += sampler.ShouldSampleRound() ? 1 : 0;
  }
  EXPECT_EQ(sampled, 25);
}

}  // namespace
}  // namespace evergreenslam::lifelong
