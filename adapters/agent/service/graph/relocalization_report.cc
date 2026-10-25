/**
 * @file relocalization_report.cc
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization requests and their outcomes, and backend events as agents read them.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/relocalization_report.h"

#include <algorithm>
#include <cctype>
#include <string_view>
#include <utility>

namespace evergreenslam::agent {
namespace {

constexpr size_t kRememberedRequests = 64;

std::string Lowercase(std::string_view text) {
  std::string lower(text);
  for (char& c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return lower;
}

const char* StateName(RelocalizationStatus::State state) {
  switch (state) {
    case RelocalizationStatus::State::PENDING:
      return "pending";
    case RelocalizationStatus::State::MATCHED:
      return "matched";
    case RelocalizationStatus::State::NO_MATCH:
      return "no_match";
    case RelocalizationStatus::State::SUPERSEDED:
      return "superseded";
  }
  return "pending";
}

void OptionalInt(JsonWriter& writer, std::string_view key, const std::optional<int64_t>& value) {
  writer.Key(key);
  if (value.has_value()) {
    writer.Int(*value);
  } else {
    writer.Null();
  }
}

std::optional<int64_t> Nanos(const std::optional<common::Time>& time) {
  if (!time.has_value()) {
    return std::nullopt;
  }
  return common::ToUnixNanos(*time);
}

void WriteCorrection(JsonWriter& writer,
                     const std::optional<lifelong::PoseCorrection>& correction) {
  if (correction.has_value()) {
    writer.Field("correction_m", correction->translation)
        .Field("correction_rad", correction->rotation);
  } else {
    writer.NullField("correction_m").NullField("correction_rad");
  }
}

std::optional<int64_t> SessionIndex(const std::optional<lifelong::SessionId>& id) {
  if (!id.has_value()) {
    return std::nullopt;
  }
  return id->session_index;
}

common::Time SensorNow(const lifelong::PoseGraphData& graph) {
  common::Time newest;
  for (const auto& [id, session] : graph.sessions()) {
    newest = std::max(newest, session.last_node_time);
  }
  return newest;
}

bool SamePrior(const std::optional<Eigen::Affine2d>& a, const std::optional<Eigen::Affine2d>& b) {
  if (a.has_value() != b.has_value()) {
    return false;
  }
  return !a.has_value() || a->matrix() == b->matrix();
}

}  // namespace

std::shared_ptr<RelocalizationRequests> RelocalizationRequests::Attach(
    lifelong::PoseGraph& pose_graph) {
  std::shared_ptr<RelocalizationRequests> requests(new RelocalizationRequests(pose_graph));
  requests->next_event_seq_ = pose_graph.EventsSince(0).next_seq;
  pose_graph.AddRelocalizationListener([weak = std::weak_ptr<RelocalizationRequests>(requests)](
                                           const lifelong::RelocalizationResult& result) {
    if (const std::shared_ptr<RelocalizationRequests> alive = weak.lock()) {
      alive->OnResult(result);
    }
  });
  return requests;
}

RelocalizationId RelocalizationRequests::Open(std::string kind,
                                              std::optional<lifelong::AnchorId> anchor,
                                              const std::optional<Eigen::Affine2d>& prior) {
  CatchUpOnDrops();
  RelocalizationStatus status;
  status.id = next_id_++;
  status.kind = std::move(kind);
  status.anchor = anchor;
  status.prior = prior;
  SupersedeOpen(status.id, SensorNow(pose_graph_.graph()));
  recent_.push_back(std::move(status));
  while (recent_.size() > kRememberedRequests) {
    recent_.pop_front();
  }
  return recent_.back().id;
}

std::optional<RelocalizationStatus> RelocalizationRequests::Find(RelocalizationId id) {
  CatchUpOnDrops();
  for (const RelocalizationStatus& status : recent_) {
    if (status.id == id) {
      return status;
    }
  }
  return std::nullopt;
}

std::optional<RelocalizationStatus> RelocalizationRequests::Latest() {
  CatchUpOnDrops();
  if (recent_.empty()) {
    return std::nullopt;
  }
  return recent_.back();
}

void RelocalizationRequests::OnResult(const lifelong::RelocalizationResult& result) {
  CatchUpOnDrops();
  if (recent_.empty()) {
    return;
  }
  RelocalizationStatus& open = recent_.back();
  if (open.state != RelocalizationStatus::State::PENDING || !SamePrior(open.prior, result.prior)) {
    return;
  }
  open.state = result.constraints_added > 0 ? RelocalizationStatus::State::MATCHED
                                            : RelocalizationStatus::State::NO_MATCH;
  open.candidates = result.candidates;
  open.constraints_added = result.constraints_added;
  open.correction = result.correction;
  open.requested_at = pose_graph_.graph().node(result.node_id).constant_data.time;
  open.resolved_at = result.time;
}

void RelocalizationRequests::CatchUpOnDrops() {
  const lifelong::PoseGraphEvents::Page page = pose_graph_.EventsSince(next_event_seq_);
  next_event_seq_ = page.next_seq;
  for (const lifelong::PoseGraphEvent& event : page.events) {
    if (event.kind == lifelong::PoseGraphEvent::Kind::FED_SESSION_DROPPED) {
      SupersedeOpen(std::nullopt, event.time);
    }
  }
}

void RelocalizationRequests::SupersedeOpen(std::optional<RelocalizationId> by, common::Time time) {
  for (RelocalizationStatus& status : recent_) {
    if (status.state == RelocalizationStatus::State::PENDING) {
      status.state = RelocalizationStatus::State::SUPERSEDED;
      status.superseded_by = by;
      status.resolved_at = time;
    }
  }
}

void WriteRelocalization(JsonWriter& writer, const std::optional<RelocalizationStatus>& status) {
  writer.Key("relocalization");
  if (!status.has_value()) {
    writer.Null();
    return;
  }
  writer.BeginObject().Field("id", status->id).Field("kind", status->kind);
  OptionalInt(writer, "anchor",
              status->anchor.has_value()
                  ? std::optional<int64_t>(static_cast<int64_t>(*status->anchor))
                  : std::nullopt);
  writer.Field("state", StateName(status->state))
      .Field("candidates", status->candidates)
      .Field("constraints", status->constraints_added);
  WriteCorrection(writer, status->correction);
  OptionalInt(writer, "requested_ns", Nanos(status->requested_at));
  OptionalInt(writer, "resolved_ns", Nanos(status->resolved_at));
  OptionalInt(writer, "superseded_by", status->superseded_by);
  writer.EndObject();
}

void WriteEvent(JsonWriter& writer, const lifelong::PoseGraphEvent& event) {
  using Kind = lifelong::PoseGraphEvent::Kind;
  writer.BeginObject()
      .Field("seq", event.seq)
      .Field("time_ns", common::ToUnixNanos(event.time))
      .Field("kind", Lowercase(lifelong::ToString(event.kind)));
  switch (event.kind) {
    case Kind::RELOCALIZATION:
      writer.Field("state", event.constraints_added > 0 ? "matched" : "no_match")
          .Field("candidates", event.candidates)
          .Field("constraints", event.constraints_added);
      WriteCorrection(writer, event.correction);
      break;
    case Kind::POSE_CORRECTION:
      OptionalInt(writer, "session", SessionIndex(event.session));
      WriteCorrection(writer, event.correction);
      break;
    case Kind::SESSION_FROZEN:
    case Kind::SESSION_REMOVED:
      OptionalInt(writer, "session", SessionIndex(event.session));
      break;
    case Kind::FED_SESSION_DROPPED:
      OptionalInt(writer, "session", SessionIndex(event.session));
      OptionalInt(writer, "fed_now", SessionIndex(event.fed_now));
      break;
  }
  writer.EndObject();
}

}  // namespace evergreenslam::agent
