/**
 * @file relocalization_tracker.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/relocalization_tracker.h"

#include <glog/logging.h>

#include <algorithm>
#include <utility>

namespace evergreenslam::lifelong {

RelocalizationId RelocalizationTracker::Mint(bool has_prior) {
  std::lock_guard<std::mutex> lock(mutex_);
  Entry entry;
  entry.status.id = next_id_++;
  entry.status.has_prior = has_prior;
  recent_.push_back(entry);
  while (recent_.size() > capacity_) {
    recent_.pop_front();
  }
  return entry.status.id;
}

std::optional<RelocalizationStatus> RelocalizationTracker::Get(RelocalizationId id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const Entry& entry : recent_) {
    if (entry.status.id == id) {
      return entry.status;
    }
  }
  return std::nullopt;
}

std::optional<RelocalizationStatus> RelocalizationTracker::Latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (recent_.empty()) {
    return std::nullopt;
  }
  return recent_.back().status;
}

std::vector<RelocalizationStatus> RelocalizationTracker::SupersedePending(
    std::optional<RelocalizationId> by, common::Time time) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<RelocalizationStatus> superseded;
  for (Entry& entry : recent_) {
    if (!entry.started || entry.status.state != RelocalizationState::PENDING) {
      continue;
    }
    entry.status.state = RelocalizationState::SUPERSEDED;
    entry.status.superseded_by = by;
    entry.status.resolved_at = time;
    superseded.push_back(entry.status);
  }
  round_.reset();
  return superseded;
}

void RelocalizationTracker::MarkStarted(RelocalizationId id) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = FindLocked(id);
  if (entry != recent_.end()) {
    entry->started = true;
  }
}

void RelocalizationTracker::BeginRound(Round round, std::optional<common::Time> requested_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = FindLocked(round.id);
  if (entry != recent_.end()) {
    entry->status.requested_at = requested_at;
  }
  round_ = std::move(round);
}

bool RelocalizationTracker::NoteMatch(RelocalizationId id, bool accepted, int solves_before_add) {
  if (!round_.has_value() || round_->id != id) {
    return false;
  }
  ++round_->finished;
  if (accepted) {
    ++round_->accepted;
    round_->solves_before_last_accept = solves_before_add;
  }
  return RoundComplete();
}

bool RelocalizationTracker::SetCandidates(RelocalizationId id, int candidates) {
  if (!round_.has_value() || round_->id != id) {
    return false;
  }
  round_->expected = candidates;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = FindLocked(id);
  if (entry != recent_.end()) {
    entry->status.candidates = candidates;
  }
  return RoundComplete();
}

RelocalizationStatus RelocalizationTracker::Resolve(RelocalizationState state,
                                                    std::optional<PoseCorrection> correction,
                                                    common::Time time) {
  CHECK(round_.has_value()) << "no relocalization round is open";
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = FindLocked(round_->id);
  RelocalizationStatus resolved;
  if (entry != recent_.end()) {
    resolved = entry->status;
  } else {
    resolved.id = round_->id;
  }
  resolved.state = state;
  resolved.candidates = round_->expected.value_or(0);
  resolved.constraints_added = round_->accepted;
  resolved.correction = correction;
  resolved.resolved_at = time;
  if (entry != recent_.end()) {
    entry->status = resolved;
  }
  round_.reset();
  return resolved;
}

std::deque<RelocalizationTracker::Entry>::iterator RelocalizationTracker::FindLocked(
    RelocalizationId id) {
  return std::find_if(recent_.begin(), recent_.end(),
                      [id](const Entry& entry) { return entry.status.id == id; });
}

bool RelocalizationTracker::RoundComplete() const {
  return round_.has_value() && round_->expected.has_value() &&
         round_->finished >= *round_->expected;
}

}  // namespace evergreenslam::lifelong
