/**
 * @file session_manager.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/session_manager.h"

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <unordered_set>
#include <utility>

#include "lifelong/coarse_footprint.h"
#include "lifelong/sessions/frozen_links.h"
#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {
namespace {

bool AnySessionFrozen(const PoseGraphData& graph) {
  for (const auto& [id, session] : graph.sessions()) {
    if (session.frozen()) {
      return true;
    }
  }
  return false;
}

bool IsFinished(const SubmapRecord& record) {
  return record.submap == nullptr || record.submap->finished();
}

using Phase = SessionManager::Phase;

bool Grew(const SessionManagerOption& option, const SessionManager::ExpansionState& state,
          double area_from, double track_from) {
  return state.novel_area - area_from >= option.growth_tolerance &&
         state.novel_track - track_from >= option.track_growth_tolerance;
}

void Step(const SessionManagerOption& option, double min_new_area,
          SessionManager::ExpansionState& state, int new_submaps, bool fed) {
  // A floating session gets no more submaps: its area is final, so it is judged as soon as it
  // is worth freezing.
  if (!fed) {
    if (state.phase != Phase::JUDGING && state.novel_area >= min_new_area) {
      state.phase = Phase::JUDGING;
      state.baseline_area = state.novel_area;
      state.baseline_track = state.novel_track;
    }
    return;
  }
  switch (state.phase) {
    case Phase::BOOTSTRAP:
      break;
    case Phase::LOCALIZING:
      if (state.novel_area >= min_new_area) {
        state.phase = Phase::EXPANDING;
        state.last_area = state.novel_area;
        state.last_track = state.novel_track;
      }
      break;
    case Phase::EXPANDING:
      if (new_submaps == 0) {
        break;
      }
      if (!Grew(option, state, state.last_area, state.last_track)) {
        state.phase = Phase::NO_GROWTH;
        state.baseline_area = state.novel_area;
        state.baseline_track = state.novel_track;
        state.submaps_without_growth = 0;
      }
      state.last_area = state.novel_area;
      state.last_track = state.novel_track;
      break;
    case Phase::NO_GROWTH:
    case Phase::JUDGING:
      if (Grew(option, state, state.baseline_area, state.baseline_track)) {
        state.phase = Phase::EXPANDING;
        state.last_area = state.novel_area;
        state.last_track = state.novel_track;
        break;
      }
      state.submaps_without_growth += new_submaps;
      if (state.phase == Phase::NO_GROWTH &&
          state.submaps_without_growth >= option.submaps_without_growth) {
        state.phase = Phase::JUDGING;
      }
      break;
  }
}

}  // namespace

SessionManagerOption LoadSessionManagerOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  SessionManagerOption option;
  option.auto_freeze = LoadOption(node, "auto_freeze", option.auto_freeze);
  option.min_new_area = LoadOption(node, "min_new_area", option.min_new_area);
  option.min_new_area_fraction =
      LoadOption(node, "min_new_area_fraction", option.min_new_area_fraction);
  option.growth_tolerance = LoadOption(node, "growth_tolerance", option.growth_tolerance);
  option.track_growth_tolerance =
      LoadOption(node, "track_growth_tolerance", option.track_growth_tolerance);
  option.submaps_without_growth =
      LoadOption(node, "submaps_without_growth", option.submaps_without_growth);
  option.coverage_resolution = LoadOption(node, "coverage_resolution", option.coverage_resolution);
  option.track_resolution = LoadOption(node, "track_resolution", option.track_resolution);
  option.frozen_covered_fraction =
      LoadOption(node, "frozen_covered_fraction", option.frozen_covered_fraction);
  if (node["freeze_judge"]) {
    option.freeze_judge = LoadFreezeJudgeOption(node["freeze_judge"]);
  }
  return option;
}

SessionManager::SessionManager(SessionHandle& handle, const SessionManagerOption& option)
    : handle_(handle),
      option_(option),
      judge_(option.freeze_judge),
      covariance_evaluator_(handle.optimization()),
      coverage_(option.coverage_resolution, option.track_resolution) {
  CHECK_GE(option_.min_new_area, 0.0);
  CHECK_GE(option_.min_new_area_fraction, 0.0);
  CHECK_GE(option_.growth_tolerance, 0.0);
  CHECK_GE(option_.track_growth_tolerance, 0.0);
  CHECK_GE(option_.submaps_without_growth, 0);
  CHECK_GT(option_.frozen_covered_fraction, 0.0);
  CHECK_LE(option_.frozen_covered_fraction, 1.0);
}

SessionId SessionManager::Start(common::Time time, const Eigen::Affine2d& local_to_global) {
  CHECK(!fed_session_.has_value()) << "Start opens the boot session once, rotation opens the rest";
  fed_session_ = handle_.StartNewSession(time, local_to_global);
  const SessionId id = *fed_session_;
  handle_.Enqueue([this, id] {
    RebuildAfterLoadOnTask();
    for (const std::shared_ptr<SessionObserver>& observer : observers_) {
      observer->OnSessionStarted(handle_.graph(), id);
    }
  });
  return id;
}

void SessionManager::AddObserver(std::shared_ptr<SessionObserver> observer) {
  CHECK(observer != nullptr);
  observers_.push_back(std::move(observer));
}

void SessionManager::OnSubmapFinishedOnTask(const SubmapId& id) {
  const PoseGraphData& graph = handle_.graph();
  if (!option_.auto_freeze || !graph.HasSubmap(id) || graph.session(SessionOf(id)).frozen()) {
    return;
  }
  expansion_[SessionOf(id)].dirty = true;
}

void SessionManager::OnConstraintAddedOnTask(const Constraint& constraint) {
  if (!option_.auto_freeze) {
    return;
  }
  const PoseGraphData& graph = handle_.graph();
  // An INTRA edge only adds a fresh variable: it can neither anchor the session nor tighten what
  // the judge refused, so it earns no re-judge.
  const bool dirties = constraint.type != Constraint::Type::INTRA_SUBMAP;
  const auto touch = [&](const VariableId& variable) {
    if (graph.session(variable.session()).frozen()) {
      return;
    }
    ExpansionState& state = expansion_[variable.session()];
    state.dirty = state.dirty || dirties;
  };
  touch(constraint.from);
  if (constraint.to.has_value()) {
    touch(*constraint.to);
  }
}

void SessionManager::RebuildAfterLoadOnTask() {
  const PoseGraphData& graph = handle_.graph();
  coverage_.Rebuild(graph);
  expansion_.clear();
  if (!option_.auto_freeze) {
    return;
  }
  // Dirty means "rebuild the account", which exists only against a frozen layer; in the
  // bootstrap a judgement still takes an event, as it does for a session fed since boot.
  const bool dirty = AnySessionFrozen(graph);
  for (const auto& [id, session] : graph.sessions()) {
    if (!session.frozen()) {
      expansion_[id].dirty = dirty;
    }
  }
}

void SessionManager::OnOptimizedOnTask() {
  // Before Start there is no boot session, no rebuilt coverage and no state to judge on.
  if (freezing_ || !fed_session_.has_value()) {
    return;
  }
  const PoseGraphData& graph = handle_.graph();
  for (auto it = expansion_.begin(); it != expansion_.end();) {
    const bool gone = !graph.HasSession(it->first) || graph.session(it->first).frozen();
    it = gone ? expansion_.erase(it) : std::next(it);
  }
  if (!option_.auto_freeze) {
    return;
  }
  const bool any_frozen = AnySessionFrozen(graph);
  const double min_new_area =
      option_.min_new_area_fraction > 0.0
          ? std::min(option_.min_new_area, option_.min_new_area_fraction * coverage_.area())
          : option_.min_new_area;
  for (auto& [id, state] : expansion_) {
    if (!state.dirty) {
      continue;
    }
    state.dirty = false;
    if (state.phase == Phase::BOOTSTRAP) {
      if (!any_frozen) {
        if (Judge(id, state)) {
          FreezeAndMaybeRotate(id);
          break;
        }
        continue;
      }
      state.phase = Phase::LOCALIZING;
    }
    const int new_submaps = Refresh(id, state);
    if (!state.anchored) {
      LOG(INFO) << "session " << id.session_index << " unanchored";
      continue;
    }
    Step(option_, min_new_area, state, new_submaps, id == *fed_session_);
    if (state.phase != Phase::JUDGING) {
      continue;
    }
    if (Judge(id, state)) {
      FreezeAndMaybeRotate(id);
      break;
    }
  }
}

bool SessionManager::Judge(SessionId id, const ExpansionState& state) {
  ++num_judgements_;
  last_verdict_ = judge_.Judge(handle_.graph(), handle_.optimization(), covariance_evaluator_, id);
  return last_verdict_.eligible;
}

int SessionManager::Refresh(SessionId id, ExpansionState& state) {
  const PoseGraphData& graph = handle_.graph();
  state.anchored = HasFrozenLink(graph, id);
  if (!state.anchored) {
    return 0;
  }
  const double resolution = coverage_.resolution();
  const double track_resolution = coverage_.track_resolution();
  std::unordered_set<std::int64_t> own;
  std::unordered_set<std::int64_t> own_track;
  int new_submaps = 0;
  int newest = state.accounted_submap_index;
  for (const SubmapId& submap_id : graph.session(id).submap_ids) {
    const SubmapRecord& record = graph.submap(submap_id);
    if (record.submap == nullptr || !record.submap->finished()) {
      continue;
    }
    if (submap_id.submap_index > state.accounted_submap_index) {
      ++new_submaps;
      newest = std::max(newest, submap_id.submap_index);
    }
    for (const NodeId& node_id : record.node_ids) {
      const std::int64_t cell =
          CoarseCellOf(graph.node(node_id).global_pose.translation(), track_resolution);
      if (!coverage_.ContainsTrack(cell)) {
        own_track.insert(cell);
      }
    }
    const std::vector<std::int64_t> cells = ComputeCoarseFootprint(record, resolution);
    const auto covered = std::count_if(
        cells.begin(), cells.end(), [this](std::int64_t cell) { return coverage_.Contains(cell); });
    // A covered submap adds nothing: its uncovered rim is lattice mismatch, not new area.
    if (static_cast<double>(covered) >=
        option_.frozen_covered_fraction * static_cast<double>(cells.size())) {
      continue;
    }
    for (const std::int64_t cell : cells) {
      if (!coverage_.Contains(cell)) {
        own.insert(cell);
      }
    }
  }
  state.accounted_submap_index = newest;
  state.novel_area = static_cast<double>(own.size()) * resolution * resolution;
  state.novel_track = static_cast<double>(own_track.size()) * track_resolution * track_resolution;
  return new_submaps;
}

void SessionManager::FreezeFedSession(std::optional<SessionId> expected) {
  handle_.Enqueue([this, expected] {
    if (freezing_ || !fed_session_.has_value()) {
      return;
    }
    if (expected.has_value() && *expected != *fed_session_) {
      LOG(WARNING) << "freeze of session " << expected->session_index << " dropped: session "
                   << fed_session_->session_index << " is fed now";
      return;
    }
    const PoseGraphData& graph = handle_.graph();
    const std::vector<SubmapId>& submap_ids = graph.session(*fed_session_).submap_ids;
    if (std::none_of(submap_ids.begin(), submap_ids.end(),
                     [&graph](const SubmapId& id) { return IsFinished(graph.submap(id)); })) {
      LOG(WARNING) << "fed session " << fed_session_->session_index
                   << " has no finished submap, nothing to freeze";
      return;
    }
    FreezeAndMaybeRotate(*fed_session_);
  });
}

void SessionManager::FreezeAndMaybeRotate(SessionId id) {
  // Rotation belongs to the fed session alone; a floating session's freeze only notifies.
  const bool rotate = fed_session_.has_value() && id == *fed_session_;
  freezing_ = true;

  if (rotate) {
    handle_.Enqueue([this, id] {
      TrimCoveredSubmapsOnTask(id);
      const SessionId next = handle_.RotateAndFreezeFedSessionOnTask(id);
      const PoseGraphData& graph = handle_.graph();
      LOG(INFO) << "session " << id.session_index << " frozen with "
                << graph.session(id).submap_ids.size() << " submaps; "
                << graph.session(next).submap_ids.size() << " unfinished handed to session "
                << next.session_index;
      OnFrozenOnTask(id);
      fed_session_ = next;
      for (const std::shared_ptr<SessionObserver>& observer : observers_) {
        observer->OnSessionStarted(graph, next);
      }
    });
    return;
  }

  handle_.Enqueue([this, id] { TrimCoveredSubmapsOnTask(id); });
  handle_.FreezeSession(id);
  handle_.Enqueue([this, id] {
    LOG(INFO) << "session " << id.session_index << " frozen with "
              << handle_.graph().session(id).submap_ids.size() << " submaps (floating)";
    OnFrozenOnTask(id);
  });
}

void SessionManager::TrimCoveredSubmapsOnTask(SessionId id) {
  const PoseGraphData& graph = handle_.graph();
  std::vector<SubmapId> finished;
  std::vector<SubmapId> covered;
  for (const SubmapId& submap_id : graph.session(id).submap_ids) {
    const SubmapRecord& record = graph.submap(submap_id);
    if (record.submap == nullptr || !record.submap->finished()) {
      continue;
    }
    finished.push_back(submap_id);
    if (coverage_.Query(record).covered_fraction() >= option_.frozen_covered_fraction) {
      covered.push_back(submap_id);
    }
  }
  // Never the whole session: the newest pass through known area still ends somewhere.
  if (!covered.empty() && covered.size() == finished.size()) {
    covered.pop_back();
  }
  if (covered.empty()) {
    return;
  }
  LOG(INFO) << "session " << id.session_index << ": " << covered.size() << " of " << finished.size()
            << " finished submaps lie in frozen area, trimmed before the freeze";
  handle_.TrimSubmapsOnTask(covered);
}

void SessionManager::OnFrozenOnTask(SessionId id) {
  const PoseGraphData& graph = handle_.graph();
  CHECK(graph.session(id).frozen());
  for (const std::shared_ptr<SessionObserver>& observer : observers_) {
    observer->OnSessionFrozen(graph, id);
  }
  coverage_.AddSession(graph, id);
  expansion_.erase(id);
  // Edges into the session just frozen are frozen links now: a neighbour may have anchored.
  for (auto& [other, state] : expansion_) {
    state.dirty = true;
  }
  num_sessions_frozen_.fetch_add(1, std::memory_order_release);
  freezing_ = false;
}

}  // namespace evergreenslam::lifelong
