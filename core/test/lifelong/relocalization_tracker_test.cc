/**
 * @file relocalization_tracker_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Request ids, round settlement in either report order, supersession and the event ring.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/relocalization_tracker.h"

#include <gtest/gtest.h>

#include <vector>

#include "lifelong/pose_graph_events.h"

namespace evergreenslam::lifelong {
namespace {

common::Time At(int seconds) { return common::FromUnixSeconds(1785000000.0 + seconds); }

RelocalizationTracker::Round RoundFor(RelocalizationId id) {
  RelocalizationTracker::Round round;
  round.id = id;
  return round;
}

TEST(RelocalizationTrackerTest, IdsStartAtOneAndTheLatestIsTheNewest) {
  RelocalizationTracker tracker;
  EXPECT_FALSE(tracker.Latest().has_value());
  EXPECT_EQ(tracker.Mint(true), 1);
  EXPECT_EQ(tracker.Mint(false), 2);
  EXPECT_EQ(tracker.Latest()->id, 2);
  EXPECT_FALSE(tracker.Latest()->has_prior);
  EXPECT_EQ(tracker.Get(1)->state, RelocalizationState::PENDING);
}

TEST(RelocalizationTrackerTest, KeepsOnlyTheMostRecentRequests) {
  RelocalizationTracker tracker(3);
  for (int i = 0; i < 5; ++i) {
    tracker.Mint(true);
  }
  EXPECT_FALSE(tracker.Get(2).has_value());
  EXPECT_TRUE(tracker.Get(3).has_value());
  EXPECT_EQ(tracker.Latest()->id, 5);
}

TEST(RelocalizationTrackerTest, InlineReportsBeforeTheCandidateCountStillSettle) {
  RelocalizationTracker tracker;
  const RelocalizationId id = tracker.Mint(true);
  tracker.MarkStarted(id);
  tracker.BeginRound(RoundFor(id), At(3));
  EXPECT_FALSE(tracker.NoteMatch(id, false, 0));
  EXPECT_FALSE(tracker.NoteMatch(id, true, 7));
  EXPECT_TRUE(tracker.SetCandidates(id, 2));
  EXPECT_EQ(tracker.round()->accepted, 1);
  EXPECT_EQ(tracker.round()->solves_before_last_accept, 7);
  const RelocalizationStatus status =
      tracker.Resolve(RelocalizationState::MATCHED, PoseCorrection{1.0, 0.5}, At(4));
  EXPECT_EQ(status.candidates, 2);
  EXPECT_EQ(status.constraints_added, 1);
  EXPECT_EQ(tracker.Get(id)->state, RelocalizationState::MATCHED);
  EXPECT_EQ(tracker.Get(id)->requested_at, At(3));
  EXPECT_EQ(tracker.Get(id)->resolved_at, At(4));
  EXPECT_FALSE(tracker.round().has_value());
}

TEST(RelocalizationTrackerTest, ZeroCandidatesSettleAtOnceAndLateReportsAreIgnored) {
  RelocalizationTracker tracker;
  const RelocalizationId id = tracker.Mint(true);
  tracker.MarkStarted(id);
  tracker.BeginRound(RoundFor(id), At(3));
  EXPECT_TRUE(tracker.SetCandidates(id, 0));
  tracker.Resolve(RelocalizationState::NO_MATCH, std::nullopt, At(3));
  EXPECT_FALSE(tracker.NoteMatch(id, true, 0));
  EXPECT_EQ(tracker.Get(id)->state, RelocalizationState::NO_MATCH);
}

TEST(RelocalizationTrackerTest, SupersedesOnlyStartedPendingRequests) {
  RelocalizationTracker tracker;
  const RelocalizationId first = tracker.Mint(true);
  tracker.MarkStarted(first);
  tracker.BeginRound(RoundFor(first), At(1));
  const RelocalizationId second = tracker.Mint(true);
  const RelocalizationId third = tracker.Mint(true);

  const std::vector<RelocalizationStatus> superseded = tracker.SupersedePending(second, At(2));
  ASSERT_EQ(superseded.size(), 1u);
  EXPECT_EQ(superseded[0].id, first);
  EXPECT_EQ(tracker.Get(first)->superseded_by, second);
  EXPECT_FALSE(tracker.round().has_value());
  EXPECT_FALSE(tracker.NoteMatch(first, true, 0));
  EXPECT_EQ(tracker.Get(third)->state, RelocalizationState::PENDING);

  tracker.MarkStarted(second);
  EXPECT_EQ(tracker.SupersedePending(std::nullopt, At(3)).size(), 1u);
  EXPECT_FALSE(tracker.Get(second)->superseded_by.has_value());
  EXPECT_EQ(tracker.Get(third)->state, RelocalizationState::PENDING);
}

PoseGraphEvent EventOf(PoseGraphEvent::Kind kind) {
  PoseGraphEvent event;
  event.kind = kind;
  return event;
}

TEST(PoseGraphEventsTest, PagesBySequenceAndReportsWhatFellOffTheRing) {
  PoseGraphEvents events(3);
  PoseGraphEvents::Page page = events.Since(0);
  EXPECT_EQ(page.first_seq, 0);
  EXPECT_EQ(page.next_seq, 0);
  EXPECT_FALSE(page.lost);
  EXPECT_TRUE(page.events.empty());

  events.Append(EventOf(PoseGraphEvent::Kind::SESSION_FROZEN));
  events.Append(EventOf(PoseGraphEvent::Kind::SESSION_REMOVED));
  page = events.Since(1);
  EXPECT_FALSE(page.lost);
  ASSERT_EQ(page.events.size(), 1u);
  EXPECT_EQ(page.events[0].seq, 1);
  EXPECT_EQ(page.events[0].kind, PoseGraphEvent::Kind::SESSION_REMOVED);
  EXPECT_EQ(page.next_seq, 2);
  EXPECT_TRUE(events.Since(2).events.empty());

  for (int i = 0; i < 3; ++i) {
    events.Append(EventOf(PoseGraphEvent::Kind::POSE_CORRECTION));
  }
  page = events.Since(1);
  EXPECT_TRUE(page.lost);
  EXPECT_EQ(page.first_seq, 2);
  EXPECT_EQ(page.next_seq, 5);
  ASSERT_EQ(page.events.size(), 3u);
  EXPECT_EQ(page.events.front().seq, 2);

  page = events.Since(9);
  EXPECT_TRUE(page.lost) << "a cursor past the end belongs to an earlier process";
  EXPECT_EQ(page.events.size(), 3u);
}

}  // namespace
}  // namespace evergreenslam::lifelong
