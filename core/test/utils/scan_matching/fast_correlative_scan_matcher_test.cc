/**
 * @file fast_correlative_scan_matcher_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/scan_matching/fast_correlative_scan_matcher.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::scan_matching {
namespace {

namespace transform = utils::transform;

constexpr int kGridWidth = 30;
constexpr int kGridHeight = 24;
constexpr double kResolution = 0.05;
constexpr double kOriginX = -0.31;
constexpr double kOriginY = -0.22;

struct Scene {
  mapping::GridMapu8 grid;
  Eigen::Affine2d ground_pose;
  sensor::PointCloud point_cloud;
};

// A sparse random map plus a cloud manufactured to fit it at ground_pose. Clutter values are
// capped well below the planted values, so the ground pose is the unique global optimum.
Scene MakeScene(uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> value(1, 60);
  std::uniform_int_distribution<int> sparsity(0, 6);
  std::vector<uint8_t> data(static_cast<size_t>(kGridWidth) * kGridHeight, mapping::kUnknownValue);
  for (uint8_t& cell : data) {
    if (sparsity(rng) == 0) {
      cell = static_cast<uint8_t>(value(rng));
    }
  }

  std::uniform_int_distribution<int> cell_x(2, kGridWidth - 3);
  std::uniform_int_distribution<int> cell_y(2, kGridHeight - 3);
  std::uniform_int_distribution<int> high_value(100, mapping::kMaxValue);
  std::vector<Eigen::Array2i> planted;
  for (int i = 0; i < 20; ++i) {
    const Eigen::Array2i cell(cell_x(rng), cell_y(rng));
    data[static_cast<size_t>(cell.y()) * kGridWidth + cell.x()] =
        static_cast<uint8_t>(high_value(rng));
    planted.push_back(cell);
  }

  mapping::GridMapu8 grid(std::move(data), kGridWidth, kGridHeight, kResolution, kOriginX, kOriginY,
                          mapping::kUnknownValue);

  std::uniform_real_distribution<double> fraction(0.3, 0.7);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  const Eigen::Affine2d ground_pose =
      transform::FromXYTheta(kOriginX + fraction(rng) * kGridWidth * kResolution,
                             kOriginY + fraction(rng) * kGridHeight * kResolution, yaw(rng));

  std::uniform_int_distribution<size_t> pick(0, planted.size() - 1);
  std::uniform_real_distribution<double> jitter(-0.015, 0.015);
  const Eigen::Affine2d to_sensor = ground_pose.inverse();
  sensor::PointCloud point_cloud;
  for (int i = 0; i < 15; ++i) {
    const Eigen::Vector2d center = grid.ToCenter(planted[pick(rng)]);
    const Eigen::Vector2d in_grid_frame = center + Eigen::Vector2d(jitter(rng), jitter(rng));
    point_cloud.push_back(sensor::Point2d{to_sensor * in_grid_frame});
  }
  return Scene{std::move(grid), ground_pose, std::move(point_cloud)};
}

FastCorrelativeScanMatcherOption SmallMapOption() {
  FastCorrelativeScanMatcherOption option;
  option.branch_and_bound_depth = 4;
  return option;
}

FastCorrelativeScanMatcher::MatchParams SmallMapParams() {
  FastCorrelativeScanMatcher::MatchParams params;
  params.linear_search_window = 0.5;
  params.min_score = 0.3;
  return params;
}

void ExpectBitwiseEqual(const std::optional<GlobalMatchResult>& bnb,
                        const std::optional<GlobalMatchResult>& brute, const char* what) {
  ASSERT_EQ(bnb.has_value(), brute.has_value()) << what;
  if (!bnb.has_value()) {
    return;
  }
  EXPECT_TRUE((bnb->pose.matrix().array() == brute->pose.matrix().array()).all())
      << what << ":\n"
      << bnb->pose.matrix() << "\nversus\n"
      << brute->pose.matrix();
  EXPECT_EQ(bnb->score, brute->score) << what;
}

TEST(FastCorrelativeScanMatcherTest, BranchAndBoundEqualsBruteForceWithNoPrior) {
  for (uint32_t seed = 1; seed <= 6; ++seed) {
    const Scene scene = MakeScene(seed);
    const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
    const FastCorrelativeScanMatcher matcher(SmallMapOption());
    const FastCorrelativeScanMatcher::MatchParams params = SmallMapParams();

    const std::optional<GlobalMatchResult> bnb =
        matcher.Match(scene.point_cloud, candidates, std::nullopt, params);
    const std::optional<GlobalMatchResult> brute =
        matcher.MatchBruteForce(scene.point_cloud, candidates, std::nullopt, params);
    ExpectBitwiseEqual(bnb, brute, "no prior");

    ASSERT_TRUE(bnb.has_value()) << "seed " << seed;
    EXPECT_LT((bnb->pose.translation() - scene.ground_pose.translation()).norm(), 0.15)
        << "seed " << seed;
    EXPECT_LT(std::abs(transform::NormalizeAngle(transform::GetYaw(bnb->pose) -
                                                 transform::GetYaw(scene.ground_pose))),
              0.08)
        << "seed " << seed;
  }
}

TEST(FastCorrelativeScanMatcherTest, BranchAndBoundEqualsBruteForceAroundAPrior) {
  for (uint32_t seed = 11; seed <= 16; ++seed) {
    const Scene scene = MakeScene(seed);
    const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
    FastCorrelativeScanMatcherOption option = SmallMapOption();
    option.angular_search_window = 0.3;
    const FastCorrelativeScanMatcher matcher(option);
    const FastCorrelativeScanMatcher::MatchParams params = SmallMapParams();

    const Eigen::Affine2d prior = scene.ground_pose * transform::FromXYTheta(0.12, -0.08, 0.1);
    const std::optional<GlobalMatchResult> bnb =
        matcher.Match(scene.point_cloud, candidates, prior, params);
    const std::optional<GlobalMatchResult> brute =
        matcher.MatchBruteForce(scene.point_cloud, candidates, prior, params);
    ExpectBitwiseEqual(bnb, brute, "with prior");

    ASSERT_TRUE(bnb.has_value()) << "seed " << seed;
    EXPECT_LT((bnb->pose.translation() - scene.ground_pose.translation()).norm(), 0.15)
        << "seed " << seed;
  }
}

TEST(FastCorrelativeScanMatcherTest, APrebuiltStackGivesTheSameResultAsAnInCallBuild) {
  for (uint32_t seed = 11; seed <= 16; ++seed) {
    const Scene scene = MakeScene(seed);
    const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
    const FastCorrelativeScanMatcher matcher(SmallMapOption());

    FastCorrelativeScanMatcher::MatchParams cached = SmallMapParams();
    cached.stack = matcher.BuildPrecomputationStack(scene.grid);
    ExpectBitwiseEqual(matcher.Match(scene.point_cloud, candidates, std::nullopt, cached),
                       matcher.Match(scene.point_cloud, candidates, std::nullopt, SmallMapParams()),
                       "cached stack");
  }
}

TEST(FastCorrelativeScanMatcherTest, TheLinearWindowBoundsTheSearchAroundThePrior) {
  int num_widened = 0;
  for (uint32_t seed = 11; seed <= 16; ++seed) {
    const Scene scene = MakeScene(seed);
    const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
    FastCorrelativeScanMatcherOption option = SmallMapOption();
    option.angular_search_window = 0.3;
    const FastCorrelativeScanMatcher matcher(option);
    FastCorrelativeScanMatcher::MatchParams narrow = SmallMapParams();
    narrow.linear_search_window = 0.05;
    FastCorrelativeScanMatcher::MatchParams wide = SmallMapParams();
    wide.linear_search_window = 0.5;

    const Eigen::Affine2d prior = scene.ground_pose * transform::FromXYTheta(0.12, -0.08, 0.1);
    const std::optional<GlobalMatchResult> widened =
        matcher.Match(scene.point_cloud, candidates, prior, wide);
    ExpectBitwiseEqual(widened, matcher.MatchBruteForce(scene.point_cloud, candidates, prior, wide),
                       "wide window");

    ASSERT_TRUE(widened.has_value()) << "seed " << seed;
    EXPECT_LT((widened->pose.translation() - scene.ground_pose.translation()).norm(), 0.15);
    const std::optional<GlobalMatchResult> unwidened =
        matcher.Match(scene.point_cloud, candidates, prior, narrow);
    if (!unwidened.has_value() ||
        (unwidened->pose.translation() - widened->pose.translation()).norm() > 1e-9) {
      ++num_widened;
    }
  }
  EXPECT_GT(num_widened, 0) << "the narrow window has to be a real restriction";
}

TEST(FastCorrelativeScanMatcherTest, MapsTheResultThroughTheSubmapFrameAndSkipsPoorSubmaps) {
  const Scene scene = MakeScene(21);
  const mapping::GridMapu8 empty_grid(
      std::vector<uint8_t>(static_cast<size_t>(kGridWidth) * kGridHeight, mapping::kUnknownValue),
      kGridWidth, kGridHeight, kResolution, kOriginX, kOriginY, mapping::kUnknownValue);
  const Eigen::Affine2d grid_to_global = transform::FromXYTheta(3.2, -1.1, 0.8);
  const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), empty_grid},
                                                   {grid_to_global, scene.grid}};
  const FastCorrelativeScanMatcher matcher(SmallMapOption());
  const FastCorrelativeScanMatcher::MatchParams params = SmallMapParams();

  const std::optional<GlobalMatchResult> bnb =
      matcher.Match(scene.point_cloud, candidates, std::nullopt, params);
  const std::optional<GlobalMatchResult> brute =
      matcher.MatchBruteForce(scene.point_cloud, candidates, std::nullopt, params);
  ExpectBitwiseEqual(bnb, brute, "two submaps");

  ASSERT_TRUE(bnb.has_value());
  const Eigen::Affine2d expected = grid_to_global * scene.ground_pose;
  EXPECT_LT((bnb->pose.translation() - expected.translation()).norm(), 0.15);
  EXPECT_LT(std::abs(transform::NormalizeAngle(transform::GetYaw(bnb->pose) -
                                               transform::GetYaw(expected))),
            0.08);
}

TEST(FastCorrelativeScanMatcherTest, ReturnsNothingWhenTheScoreBarIsUnreachable) {
  const Scene scene = MakeScene(31);
  const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
  const FastCorrelativeScanMatcher matcher(SmallMapOption());
  FastCorrelativeScanMatcher::MatchParams params = SmallMapParams();

  EXPECT_TRUE(matcher.Match(scene.point_cloud, candidates, std::nullopt, params).has_value());
  params.min_score = 0.98;  // above kMaxProbability, nothing can clear it
  EXPECT_FALSE(matcher.Match(scene.point_cloud, candidates, std::nullopt, params).has_value());
  EXPECT_FALSE(
      matcher.MatchBruteForce(scene.point_cloud, candidates, std::nullopt, params).has_value());
}

TEST(FastCorrelativeScanMatcherOptionTest, OverridesApplyAndAbsentKeysKeepDefaults) {
  const FastCorrelativeScanMatcherOption defaults;
  const YAML::Node node = YAML::Load(R"(
angular_search_window: 0.35
branch_and_bound_depth: 5
)");
  const FastCorrelativeScanMatcherOption option = LoadFastCorrelativeScanMatcherOption(node);
  EXPECT_DOUBLE_EQ(option.angular_search_window, 0.35);
  EXPECT_EQ(option.branch_and_bound_depth, 5);

  const FastCorrelativeScanMatcherOption from_empty =
      LoadFastCorrelativeScanMatcherOption(YAML::Node());
  EXPECT_DOUBLE_EQ(from_empty.angular_search_window, defaults.angular_search_window);
  EXPECT_EQ(from_empty.branch_and_bound_depth, defaults.branch_and_bound_depth);
}

// Both shipped configs document the defaults, so each must reproduce them exactly.
void ExpectFileMatchesTheDefaults(const std::string& name) {
  const std::string path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/" + name;
  const YAML::Node root = YAML::LoadFile(path);
  ASSERT_TRUE(root["global_scan_matcher"]);
  ASSERT_TRUE(root["global_scan_matcher"]["fast_correlative"]);
  const FastCorrelativeScanMatcherOption option =
      LoadFastCorrelativeScanMatcherOption(root["global_scan_matcher"]["fast_correlative"]);
  const FastCorrelativeScanMatcherOption defaults;
  EXPECT_DOUBLE_EQ(option.angular_search_window, defaults.angular_search_window);
  EXPECT_EQ(option.branch_and_bound_depth, defaults.branch_and_bound_depth);
}

TEST(FastCorrelativeScanMatcherOptionTest, ShippedConfigMatchesTheDefaults) {
  ExpectFileMatchesTheDefaults("evergreenslam.yaml");
}

TEST(FastCorrelativeScanMatcherOptionTest, ReferenceConfigMatchesTheDefaults) {
  ExpectFileMatchesTheDefaults("reference.yaml");
}

TEST(FastCorrelativeScanMatcherTest, DegenerateInputsGiveNoResult) {
  const Scene scene = MakeScene(41);
  const FastCorrelativeScanMatcher matcher(SmallMapOption());
  const FastCorrelativeScanMatcher::MatchParams params = SmallMapParams();

  const sensor::PointCloud empty_cloud;
  const std::vector<CandidateSubmap> candidates = {{Eigen::Affine2d::Identity(), scene.grid}};
  EXPECT_FALSE(matcher.Match(empty_cloud, candidates, std::nullopt, params).has_value());

  const std::vector<CandidateSubmap> no_candidates;
  EXPECT_FALSE(matcher.Match(scene.point_cloud, no_candidates, std::nullopt, params).has_value());

  const mapping::GridMapu8 zero_sized(std::vector<uint8_t>(), 0, 0, kResolution, 0.0, 0.0,
                                      mapping::kUnknownValue);
  const std::vector<CandidateSubmap> only_empty = {{Eigen::Affine2d::Identity(), zero_sized}};
  EXPECT_FALSE(matcher.Match(scene.point_cloud, only_empty, std::nullopt, params).has_value());
}

}  // namespace
}  // namespace evergreenslam::utils::scan_matching
