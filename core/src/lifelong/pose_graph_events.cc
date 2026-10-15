/**
 * @file pose_graph_events.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_events.h"

#include <utility>

namespace evergreenslam::lifelong {

const char* ToString(RelocalizationState state) {
  switch (state) {
    case RelocalizationState::PENDING:
      return "PENDING";
    case RelocalizationState::MATCHED:
      return "MATCHED";
    case RelocalizationState::NO_MATCH:
      return "NO_MATCH";
    case RelocalizationState::SUPERSEDED:
      return "SUPERSEDED";
  }
  return "PENDING";
}

const char* ToString(PoseGraphEvent::Kind kind) {
  using Kind = PoseGraphEvent::Kind;
  switch (kind) {
    case Kind::RELOCALIZATION:
      return "RELOCALIZATION";
    case Kind::POSE_CORRECTION:
      return "POSE_CORRECTION";
    case Kind::SESSION_FROZEN:
      return "SESSION_FROZEN";
    case Kind::SESSION_REMOVED:
      return "SESSION_REMOVED";
    case Kind::FED_SESSION_DROPPED:
      return "FED_SESSION_DROPPED";
  }
  return "RELOCALIZATION";
}

void PoseGraphEvents::Append(PoseGraphEvent event) {
  std::lock_guard<std::mutex> lock(mutex_);
  event.seq = next_seq_++;
  events_.push_back(std::move(event));
  while (events_.size() > capacity_) {
    events_.pop_front();
  }
}

PoseGraphEvents::Page PoseGraphEvents::Since(int64_t seq) const {
  std::lock_guard<std::mutex> lock(mutex_);
  Page page;
  page.next_seq = next_seq_;
  page.first_seq = events_.empty() ? next_seq_ : events_.front().seq;
  page.lost = seq < page.first_seq || seq > page.next_seq;
  const int64_t from = page.lost ? page.first_seq : seq;
  for (const PoseGraphEvent& event : events_) {
    if (event.seq >= from) {
      page.events.push_back(event);
    }
  }
  return page;
}

}  // namespace evergreenslam::lifelong
