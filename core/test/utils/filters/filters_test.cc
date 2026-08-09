/**
 * @file filters_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <random>

#include "common/time.h"
#include "utils/filters/generic_tracking_filter.h"
#include "utils/filters/motion_filter.h"
#include "utils/filters/pose_extrapolator.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::filters {
namespace {

// Sensor stamps are absolute; the epoch is irrelevant to what these test.
common::Time At(double seconds) { return common::FromUnixSeconds(seconds); }

using transform::FromXYTheta;
using transform::GetYaw;

void ExpectPoseNear(const Eigen::Affine2d& pose, float x, float y, float theta, float tolerance) {
  EXPECT_NEAR(pose.translation().x(), x, tolerance);
  EXPECT_NEAR(pose.translation().y(), y, tolerance);
  EXPECT_NEAR(GetYaw(pose), theta, tolerance);
}

TEST(MotionFilterTest, FirstScanIsAlwaysKept) {
  MotionFilter filter;
  EXPECT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_EQ(filter.num_total(), 1);
  EXPECT_EQ(filter.num_kept(), 1);
}

TEST(MotionFilterTest, ScanBelowEveryThresholdIsDropped) {
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_TRUE(filter.IsSimilar(At(0.5), FromXYTheta(0.01f, 0.01f, 0.005f)));
  EXPECT_EQ(filter.num_total(), 2);
  EXPECT_EQ(filter.num_kept(), 1);
}

TEST(MotionFilterTest, DistanceAloneTriggersAKeep) {
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_FALSE(filter.IsSimilar(At(0.1), FromXYTheta(0.5f, 0.f, 0.f)));
  EXPECT_EQ(filter.num_kept(), 2);
}

TEST(MotionFilterTest, AngleAloneTriggersAKeep) {
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_FALSE(filter.IsSimilar(At(0.1), FromXYTheta(0.001f, 0.f, 0.5f)));
  EXPECT_EQ(filter.num_kept(), 2);
}

TEST(MotionFilterTest, TimeAloneTriggersAKeep) {
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_FALSE(filter.IsSimilar(At(10.0), FromXYTheta(0.001f, 0.f, 0.0001f)));
  EXPECT_EQ(filter.num_kept(), 2);
}

TEST(MotionFilterTest, OptionsAreHonoured) {
  MotionFilterOption options;
  options.max_distance_meters = 0.01;
  MotionFilter filter(options);
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  EXPECT_FALSE(filter.IsSimilar(At(0.1), FromXYTheta(0.05f, 0.f, 0.f)));
}

TEST(MotionFilterTest, SmallStepsAccumulateUntilTheyAreKept) {
  // Ten steps of 5 cm, none of which clears the 20 cm threshold on its own.
  // If the reference pose were refreshed on dropped scans it would never move
  // more than one step ahead and nothing after the first scan would be kept.
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  for (int i = 1; i <= 10; ++i) {
    filter.IsSimilar(At(static_cast<double>(i) * 0.1),
                     FromXYTheta(static_cast<float>(i) * 0.05f, 0.f, 0.f));
  }
  EXPECT_GT(filter.num_kept(), 1) << "the reference pose is being updated on dropped scans";
  // Whether the threshold is exclusive or inclusive, 0.5 m of travel in 0.2 m
  // steps yields the first scan plus two more.
  EXPECT_EQ(filter.num_kept(), 3);
  EXPECT_EQ(filter.num_total(), 11);
}

TEST(MotionFilterTest, ResetForcesTheNextScanToBeKept) {
  MotionFilter filter;
  ASSERT_FALSE(filter.IsSimilar(At(0.0), FromXYTheta(0.f, 0.f, 0.f)));
  ASSERT_TRUE(filter.IsSimilar(At(0.1), FromXYTheta(0.001f, 0.f, 0.f)));
  filter.Reset();
  EXPECT_FALSE(filter.IsSimilar(At(0.2), FromXYTheta(0.001f, 0.f, 0.f)));
}

TEST(PoseExtrapolatorTest, NoPosesGivesIdentity) {
  PoseExtrapolator extrapolator;
  EXPECT_TRUE(extrapolator.empty());
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(0.0)), 0.f, 0.f, 0.f, 1e-6f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(7.5)), 0.f, 0.f, 0.f, 1e-6f);
}

TEST(PoseExtrapolatorTest, OnePoseIsReturnedUnchanged) {
  PoseExtrapolator extrapolator;
  extrapolator.AddPose(At(1.0), FromXYTheta(2.f, -3.f, 0.4f));
  EXPECT_FALSE(extrapolator.empty());
  // The single pose must land in slot 1, which is what latest_pose() reads.
  EXPECT_EQ(extrapolator.latest_time(), At(1.0));
  ExpectPoseNear(extrapolator.latest_pose(), 2.f, -3.f, 0.4f, 1e-5f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(9.0)), 2.f, -3.f, 0.4f, 1e-5f);
}

TEST(PoseExtrapolatorTest, TwoPosesExtrapolateLinearly) {
  PoseExtrapolator extrapolator;
  extrapolator.AddPose(At(0.0), FromXYTheta(0.f, 0.f, 0.f));
  extrapolator.AddPose(At(1.0), FromXYTheta(1.f, 2.f, 0.1f));

  ExpectPoseNear(extrapolator.ExtrapolatePose(At(1.0)), 1.f, 2.f, 0.1f, 1e-5f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(1.5)), 1.5f, 3.f, 0.15f, 1e-4f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(2.0)), 2.f, 4.f, 0.2f, 1e-4f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(3.0)), 3.f, 6.f, 0.3f, 1e-4f);
}

// Real stamps are ~1.8e9 s with ~0.1 s between scans. As double seconds the
// span was 9 significant digits into a 15 digit mantissa; the ticks are int64
// so nothing here depends on that any more.
TEST(PoseExtrapolatorTest, InterpolatesAcrossASubSecondSpanAtARealUtcStamp) {
  const common::Time base = common::FromUnixSeconds(1785000000.0);
  PoseExtrapolator extrapolator;
  extrapolator.AddPose(base, FromXYTheta(0.f, 0.f, 0.f));
  extrapolator.AddPose(base + common::FromSeconds(0.1), FromXYTheta(0.2f, 0.1f, 0.02f));

  ExpectPoseNear(extrapolator.ExtrapolatePose(base + common::FromSeconds(0.05)), 0.1f, 0.05f, 0.01f,
                 1e-5f);
  ExpectPoseNear(extrapolator.ExtrapolatePose(base + common::FromSeconds(0.2)), 0.4f, 0.2f, 0.04f,
                 1e-5f);
}

TEST(PoseExtrapolatorTest, OnlyTheLastTwoPosesAreUsed) {
  PoseExtrapolator extrapolator;
  extrapolator.AddPose(At(0.0), FromXYTheta(100.f, 100.f, 1.f));
  extrapolator.AddPose(At(1.0), FromXYTheta(0.f, 0.f, 0.f));
  extrapolator.AddPose(At(2.0), FromXYTheta(1.f, 0.f, 0.f));
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(3.0)), 2.f, 0.f, 0.f, 1e-4f);
}

TEST(PoseExtrapolatorTest, AngleIsNormalisedAcrossTheWrap) {
  // 3.0 -> -3.1 is a +0.1832 rad turn through pi, not a -6.1 rad one. The
  // factor is 1.5 so that the two readings do not differ by a multiple of
  // 2 pi and therefore cannot alias onto the same rotation matrix.
  PoseExtrapolator forward;
  forward.AddPose(At(0.0), FromXYTheta(0.f, 0.f, 3.0f));
  forward.AddPose(At(1.0), FromXYTheta(0.f, 0.f, -3.1f));
  const float forward_yaw = GetYaw(forward.ExtrapolatePose(At(1.5)));
  EXPECT_NEAR(forward_yaw, -3.0084073f, 1e-3f);
  EXPECT_LE(forward_yaw, static_cast<float>(M_PI) + 1e-5f);
  EXPECT_GE(forward_yaw, -static_cast<float>(M_PI) - 1e-5f);

  PoseExtrapolator backward;
  backward.AddPose(At(0.0), FromXYTheta(0.f, 0.f, -3.0f));
  backward.AddPose(At(1.0), FromXYTheta(0.f, 0.f, 3.1f));
  const float backward_yaw = GetYaw(backward.ExtrapolatePose(At(1.5)));
  EXPECT_NEAR(backward_yaw, 3.0084073f, 1e-3f);
  EXPECT_LE(backward_yaw, static_cast<float>(M_PI) + 1e-5f);
  EXPECT_GE(backward_yaw, -static_cast<float>(M_PI) - 1e-5f);
}

TEST(PoseExtrapolatorTest, ResetClearsTheHistory) {
  PoseExtrapolator extrapolator;
  extrapolator.AddPose(At(0.0), FromXYTheta(0.f, 0.f, 0.f));
  extrapolator.AddPose(At(1.0), FromXYTheta(1.f, 2.f, 0.1f));
  extrapolator.Reset();
  EXPECT_TRUE(extrapolator.empty());
  ExpectPoseNear(extrapolator.ExtrapolatePose(At(5.0)), 0.f, 0.f, 0.f, 1e-6f);
}

// A unicycle driving at a constant body velocity and turn rate, in closed form. Stamps sit at a
// realistic epoch so a double-seconds regression in the time handling would show up here.
constexpr double kTrackStart = 1785000000.0;
constexpr double kSpeed = 0.5;     // m/s
constexpr double kTurnRate = 0.2;  // rad/s

Eigen::Affine2d TrackPoseAt(double t) {
  const double theta = kTurnRate * t;
  return FromXYTheta(kSpeed / kTurnRate * std::sin(theta),
                     kSpeed / kTurnRate * (1.0 - std::cos(theta)), theta);
}

GenericTrackingFilter::Measurement TrackMeasurementAt(double t) {
  GenericTrackingFilter::Measurement measurement;
  measurement.time = At(kTrackStart + t);
  measurement.pose = TrackPoseAt(t);
  measurement.covariance = Eigen::Vector3d(4e-4, 4e-4, 1e-4).asDiagonal();
  return measurement;
}

Eigen::Affine2d PredictedPoseAt(const GenericTrackingFilter& filter, double t) {
  const Eigen::Matrix<double, 9, 1> state = filter.PredictTime(At(kTrackStart + t)).state;
  return FromXYTheta(state(0), state(1), state(2));
}

double TranslationErrorAt(const Eigen::Affine2d& pose, double t) {
  return (pose.translation() - TrackPoseAt(t).translation()).norm();
}

// Holding the last pose is already inside the matcher search window on any gentle trajectory,
// so a filter that estimates nothing would still let the pipeline pass. The baseline comparison
// is what makes this test able to fail.
TEST(GenericTrackingFilterTest, PredictsAheadBetterThanHoldingTheLastPose) {
  GenericTrackingFilter filter;
  PoseExtrapolator constant_velocity;
  for (int i = 0; i < 20; ++i) {
    const GenericTrackingFilter::Measurement measurement = TrackMeasurementAt(0.1 * i);
    filter.Update(measurement);
    constant_velocity.AddPose(measurement.time, measurement.pose);
  }

  const double last_t = 0.1 * 19;
  const double query_t = last_t + 0.1;
  const double hold_error = TranslationErrorAt(TrackPoseAt(last_t), query_t);
  const double filter_error = TranslationErrorAt(PredictedPoseAt(filter, query_t), query_t);
  const double constant_velocity_error =
      TranslationErrorAt(constant_velocity.ExtrapolatePose(At(kTrackStart + query_t)), query_t);

  EXPECT_NEAR(hold_error, kSpeed * 0.1, 1e-3);
  EXPECT_LT(filter_error, hold_error / 5.0)
      << "filter error " << filter_error << " m, holding the last pose gives " << hold_error;
  // A no-regression guard, not evidence of a win: on noise free input at a constant turn rate the
  // two agree to about a millimetre, because that is exactly what a constant velocity model
  // assumes. The test below is the one that separates them.
  EXPECT_LT(filter_error, constant_velocity_error * 1.1)
      << "filter error " << filter_error << " m, constant velocity gives "
      << constant_velocity_error;
}

// The comparison that reflects deployment: PoseExtrapolator derives its velocity from two noisy
// poses and amplifies that noise, while the filter averages it down. On noise free input the
// two agree to a millimetre, which is why that version of this test would prove nothing.
TEST(GenericTrackingFilterTest, BeatsConstantVelocityOnNoisyMeasurements) {
  GenericTrackingFilter filter;
  PoseExtrapolator constant_velocity;
  std::mt19937 rng(42);
  std::normal_distribution<double> translation_noise(0.0, 0.02);
  std::normal_distribution<double> rotation_noise(0.0, 0.01);

  double filter_error = 0.0;
  double constant_velocity_error = 0.0;
  int num_predictions = 0;
  for (int i = 0; i < 100; ++i) {
    const double t = 0.1 * i;
    GenericTrackingFilter::Measurement measurement = TrackMeasurementAt(t);
    const Eigen::Affine2d truth = measurement.pose;
    measurement.pose = FromXYTheta(truth.translation().x() + translation_noise(rng),
                                   truth.translation().y() + translation_noise(rng),
                                   GetYaw(truth) + rotation_noise(rng));
    filter.Update(measurement);
    constant_velocity.AddPose(measurement.time, measurement.pose);

    // The first two seconds are the filter converging; what is being compared is steady state.
    if (i < 20) {
      continue;
    }
    const double query_t = t + 0.1;
    const Eigen::Vector2d gt = TrackPoseAt(query_t).translation();
    filter_error += (PredictedPoseAt(filter, query_t).translation() - gt).squaredNorm();
    constant_velocity_error +=
        (constant_velocity.ExtrapolatePose(At(kTrackStart + query_t)).translation() - gt)
            .squaredNorm();
    ++num_predictions;
  }

  const double filter_rms = std::sqrt(filter_error / num_predictions);
  const double constant_velocity_rms = std::sqrt(constant_velocity_error / num_predictions);
  // No margin: the shipped Singer sigmas deliberately leave headroom for aggressive manoeuvres
  // instead of smoothing hard, so on this benign track the filter wins only narrowly.
  EXPECT_LT(filter_rms, constant_velocity_rms)
      << "filter rms " << filter_rms << " m, constant velocity gives " << constant_velocity_rms;
}

TEST(GenericTrackingFilterTest, VelocityIsUnobservableUntilTheSecondMeasurement) {
  GenericTrackingFilter filter;
  EXPECT_TRUE(filter.empty());

  filter.Update(TrackMeasurementAt(0.0));
  EXPECT_FALSE(filter.empty());
  ExpectPoseNear(PredictedPoseAt(filter, 0.5), 0.f, 0.f, 0.f, 1e-6f);

  for (int i = 1; i < 10; ++i) {
    filter.Update(TrackMeasurementAt(0.1 * i));
  }
  EXPECT_LT(TranslationErrorAt(PredictedPoseAt(filter, 1.0), 1.0), 0.02);
}

TEST(GenericTrackingFilterTest, IgnoresAMeasurementOlderThanItsWholeHistory) {
  GenericTrackingFilter filter;
  for (int i = 0; i < 10; ++i) {
    filter.Update(TrackMeasurementAt(0.1 * i));
  }
  const Eigen::Affine2d before = PredictedPoseAt(filter, 1.0);

  GenericTrackingFilter::Measurement stale = TrackMeasurementAt(0.0);
  stale.time = At(kTrackStart - 5.0);
  stale.pose = FromXYTheta(50.0, -50.0, 1.0);
  filter.Update(stale);

  const Eigen::Affine2d after = PredictedPoseAt(filter, 1.0);
  EXPECT_LT((after.translation() - before.translation()).norm(), 1e-9);

  for (int i = 10; i < 20; ++i) {
    filter.Update(TrackMeasurementAt(0.1 * i));
  }
  EXPECT_LT(TranslationErrorAt(PredictedPoseAt(filter, 2.0), 2.0), 0.02);
}

// Measurements that arrive late but still land inside the history rewind the filter and replay
// what came after, so the estimate must not depend on arrival order.
TEST(GenericTrackingFilterTest, ReplaysAnOutOfOrderMeasurementInsideTheHistory) {
  GenericTrackingFilter in_order;
  GenericTrackingFilter delayed;
  for (int i = 0; i < 20; ++i) {
    in_order.Update(TrackMeasurementAt(0.1 * i));
    if (i != 12) {
      delayed.Update(TrackMeasurementAt(0.1 * i));
    }
  }
  delayed.Update(TrackMeasurementAt(0.1 * 12));

  const Eigen::Affine2d expected = PredictedPoseAt(in_order, 2.0);
  const Eigen::Affine2d actual = PredictedPoseAt(delayed, 2.0);
  EXPECT_LT((actual.translation() - expected.translation()).norm(), 1e-6);
  EXPECT_LT(std::abs(transform::NormalizeAngle(GetYaw(actual) - GetYaw(expected))), 1e-6);
}

TEST(GenericTrackingFilterTest, ResetTakesTheNextMeasurementAtFaceValue) {
  GenericTrackingFilter filter;
  for (int i = 0; i < 20; ++i) {
    filter.Update(TrackMeasurementAt(0.1 * i));
  }
  filter.Reset();
  EXPECT_TRUE(filter.empty());

  // A jump this large would be smoothed away for many scans if Reset left the pose confident,
  // which is the difference between Reset() and ResetZero().
  GenericTrackingFilter::Measurement elsewhere = TrackMeasurementAt(2.0);
  elsewhere.pose = FromXYTheta(100.0, -30.0, 2.0);
  filter.Update(elsewhere);
  ExpectPoseNear(PredictedPoseAt(filter, 2.0), 100.f, -30.f, 2.f, 1e-4f);
}

}  // namespace
}  // namespace evergreenslam::utils::filters
