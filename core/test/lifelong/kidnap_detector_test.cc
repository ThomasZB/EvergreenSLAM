/**
 * @file kidnap_detector_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The three signals together, each alone innocent, the streak, and the cooldown backoff.
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/kidnap_detector.h"

#include <gtest/gtest.h>

#include <optional>

namespace evergreenslam::lifelong {
namespace {

constexpr double kGoodMatch = 0.72;
constexpr double kBrokenMatch = 0.2;
constexpr double kGoodBase = 0.65;
constexpr double kBadBase = 0.25;

// A robot driving 0.5 m per keyframe, with the drift tracker's base-match bookkeeping.
class Drive {
 public:
  explicit Drive(const KidnapDetectorOption& option = KidnapDetectorOption())
      : detector_(option) {}

  std::optional<KidnapEvidence> Step(std::optional<double> match_score, double base_score,
                                     double coverage = 0.9, bool relocalization_open = false) {
    path_ += 0.5;
    KidnapSignals signals;
    signals.match_score = match_score;
    signals.base = FrozenBaseMap::Agreement{coverage, base_score};
    if (mark_.has_value()) {
      signals.since_base_match = path_ - *mark_;
    }
    signals.path_length = path_;
    return detector_.Observe(signals, relocalization_open);
  }

  // Keyframes fed with these signals until the detector fires; -1 when it never does in `limit`.
  int StepsToFire(double match_score, double base_score, int limit = 200) {
    for (int i = 1; i <= limit; ++i) {
      if (Step(match_score, base_score).has_value()) {
        return i;
      }
    }
    return -1;
  }

  void MatchBase() { mark_ = path_; }
  double path() const { return path_; }
  KidnapDetector& detector() { return detector_; }

 private:
  KidnapDetector detector_;
  double path_ = 0.0;
  std::optional<double> mark_;
};

// Localized on the base for a while: a closure, then twenty good keyframes past it.
Drive Localized(const KidnapDetectorOption& option = KidnapDetectorOption()) {
  Drive drive(option);
  drive.Step(kGoodMatch, kGoodBase);
  drive.MatchBase();
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(drive.Step(kGoodMatch, kGoodBase).has_value());
  }
  return drive;
}

TEST(KidnapDetectorTest, AFrontendBreakThenADisagreeingBaseFiresAfterTheStreak) {
  Drive drive = Localized();
  // The carry: one bad frontend match, then tracking resumes in the wrong place.
  EXPECT_FALSE(drive.Step(kBrokenMatch, kBadBase).has_value());
  for (int i = 2; i < 8; ++i) {
    EXPECT_FALSE(drive.Step(kGoodMatch, kBadBase).has_value()) << i;
  }
  EXPECT_EQ(drive.detector().status().suspect_keyframes, 7);
  EXPECT_TRUE(drive.detector().status().continuity_broken);
  const std::optional<KidnapEvidence> evidence = drive.Step(kGoodMatch, kBadBase);
  ASSERT_TRUE(evidence.has_value());
  EXPECT_EQ(evidence->keyframes, 8);
  ASSERT_TRUE(evidence->lowest_match_score.has_value());
  EXPECT_DOUBLE_EQ(*evidence->lowest_match_score, kBrokenMatch);
  EXPECT_DOUBLE_EQ(evidence->base_score, kBadBase);
  EXPECT_DOUBLE_EQ(evidence->base_coverage, 0.9);
  EXPECT_DOUBLE_EQ(evidence->distance_without_base_match, 14.0);
  EXPECT_EQ(drive.detector().status().suspect_keyframes, 0);
}

// Furniture moved since the freeze: the base disagrees, but tracking never broke.
TEST(KidnapDetectorTest, ADisagreeingBaseWithUnbrokenTrackingIsAChangedRoom) {
  Drive drive = Localized();
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), -1);
  EXPECT_FALSE(drive.detector().status().continuity_broken);
}

// A bump the frontend shrugged off: the next base closure confirms the pose.
TEST(KidnapDetectorTest, ABaseClosureAfterTheBreakMendsContinuity) {
  Drive drive = Localized();
  drive.Step(kBrokenMatch, kGoodBase);
  EXPECT_TRUE(drive.detector().status().continuity_broken);
  drive.MatchBase();
  drive.Step(kGoodMatch, kGoodBase);
  EXPECT_FALSE(drive.detector().status().continuity_broken);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), -1);
}

TEST(KidnapDetectorTest, NoStreakBeforeTheDistanceWithoutABaseClosure) {
  Drive drive = Localized();
  drive.MatchBase();
  // 3 m at 0.5 m a keyframe: the sixth keyframe is the first suspect, the thirteenth fires.
  drive.Step(kBrokenMatch, kBadBase);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 12);
}

// A boot seed is a guess until the base confirms it, so it needs no frontend break. The
// distance counts from the session's start: suspect from the sixth keyframe, fired at the 13th.
TEST(KidnapDetectorTest, ASessionThatNeverMatchedTheBaseCountsAsBroken) {
  Drive drive;
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 13);
}

TEST(KidnapDetectorTest, EvidenceOfANeverMatchedSessionHasNoLowestScore) {
  Drive drive;
  std::optional<KidnapEvidence> evidence;
  for (int i = 0; i < 13 && !evidence.has_value(); ++i) {
    evidence = drive.Step(kBrokenMatch, kBadBase);
  }
  ASSERT_TRUE(evidence.has_value());
  EXPECT_FALSE(evidence->lowest_match_score.has_value());
  EXPECT_DOUBLE_EQ(evidence->distance_without_base_match, drive.path());
}

// Driving off the frozen map into new ground: most points land on unknown cells.
TEST(KidnapDetectorTest, LowCoverageIsUnmappedGroundNotAKidnap) {
  Drive drive = Localized();
  drive.Step(kBrokenMatch, kBadBase, 0.3);
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(drive.Step(kGoodMatch, kBadBase, 0.3).has_value());
  }
}

TEST(KidnapDetectorTest, NoFrozenBaseOrNoScoreNeverFires) {
  KidnapDetector detector;
  for (int i = 1; i <= 100; ++i) {
    KidnapSignals signals;
    signals.match_score = kBrokenMatch;
    signals.path_length = 0.5 * i;
    if (i % 2 == 0) {
      signals.base = FrozenBaseMap::Agreement{0.0, std::nullopt};
    }
    EXPECT_FALSE(detector.Observe(signals, false).has_value());
  }
}

TEST(KidnapDetectorTest, WithoutFrontendScoresOnlyANeverMatchedSessionCanFire) {
  Drive matched = Localized();
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(matched.Step(std::nullopt, kBadBase).has_value());
  }
  Drive boot;
  int fired_at = -1;
  for (int i = 1; i <= 13 && fired_at < 0; ++i) {
    fired_at = boot.Step(std::nullopt, kBadBase).has_value() ? i : -1;
  }
  EXPECT_EQ(fired_at, 13);
}

TEST(KidnapDetectorTest, AnOpenRelocalizationRestartsTheStreak) {
  KidnapDetectorOption option;
  option.min_distance_without_base_match = 0.0;
  Drive drive(option);
  for (int i = 0; i < 7; ++i) {
    EXPECT_FALSE(drive.Step(kGoodMatch, kBadBase).has_value());
  }
  EXPECT_FALSE(drive.Step(kGoodMatch, kBadBase, 0.9, /*relocalization_open=*/true).has_value());
  EXPECT_EQ(drive.detector().status().suspect_keyframes, 0);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 8);
}

TEST(KidnapDetectorTest, ResetRestartsTheStreak) {
  KidnapDetectorOption option;
  option.min_distance_without_base_match = 0.0;
  Drive drive(option);
  for (int i = 0; i < 7; ++i) {
    drive.Step(kGoodMatch, kBadBase);
  }
  drive.detector().Reset();
  EXPECT_EQ(drive.detector().status().suspect_keyframes, 0);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 8);
}

// Each no_match doubles the wait up to the cap; a match restores it.
TEST(KidnapDetectorTest, TheCooldownBacksOffAfterEachNoMatch) {
  KidnapDetectorOption option;
  option.min_suspect_keyframes = 2;
  option.min_distance_without_base_match = 0.0;
  option.cooldown_keyframes = 4;
  option.max_cooldown_keyframes = 10;
  Drive drive(option);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 2);
  drive.detector().OnSearchSettled(RelocalizationState::NO_MATCH);
  EXPECT_EQ(drive.detector().status().cooldown_keyframes_left, 4);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 4);
  drive.detector().OnSearchSettled(RelocalizationState::NO_MATCH);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 8);
  drive.detector().OnSearchSettled(RelocalizationState::NO_MATCH);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 10);
  drive.detector().OnSearchSettled(RelocalizationState::NO_MATCH);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 10);
  drive.detector().OnSearchSettled(RelocalizationState::MATCHED);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 4);
}

// An operator request or a drop took the search over: wait, but do not back off.
TEST(KidnapDetectorTest, ASupersededSearchKeepsTheCooldown) {
  KidnapDetectorOption option;
  option.min_suspect_keyframes = 2;
  option.min_distance_without_base_match = 0.0;
  option.cooldown_keyframes = 4;
  Drive drive(option);
  drive.StepsToFire(kGoodMatch, kBadBase);
  drive.detector().OnSearchSettled(RelocalizationState::SUPERSEDED);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 4);
  drive.detector().OnSearchSettled(RelocalizationState::SUPERSEDED);
  EXPECT_EQ(drive.StepsToFire(kGoodMatch, kBadBase), 4);
}

TEST(KidnapDetectorTest, DisabledNeverFires) {
  KidnapDetectorOption option;
  option.enabled = false;
  Drive drive(option);
  EXPECT_EQ(drive.StepsToFire(kBrokenMatch, kBadBase), -1);
}

}  // namespace
}  // namespace evergreenslam::lifelong
