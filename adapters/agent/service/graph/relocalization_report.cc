/**
 * @file relocalization_report.cc
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization outcomes and backend events as agents read them.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/relocalization_report.h"

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

}  // namespace

void RelocalizationRequests::Remember(lifelong::RelocalizationId id,
                                      RelocalizationRequest request) {
  std::lock_guard<std::mutex> lock(mutex_);
  requests_[id] = std::move(request);
  while (requests_.size() > kRememberedRequests) {
    requests_.erase(requests_.begin());
  }
}

std::optional<RelocalizationRequest> RelocalizationRequests::Find(
    lifelong::RelocalizationId id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = requests_.find(id);
  if (it == requests_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void WriteRelocalization(JsonWriter& writer,
                         const std::optional<lifelong::RelocalizationStatus>& status,
                         const RelocalizationRequests& requests) {
  writer.Key("relocalization");
  if (!status.has_value()) {
    writer.Null();
    return;
  }
  const std::optional<RelocalizationRequest> request = requests.Find(status->id);
  const std::string fallback_kind = status->has_prior ? "pose" : "global";
  writer.BeginObject()
      .Field("id", static_cast<int64_t>(status->id))
      .Field("kind", request.has_value() ? request->kind : fallback_kind);
  OptionalInt(writer, "anchor",
              request.has_value() && request->anchor.has_value()
                  ? std::optional<int64_t>(static_cast<int64_t>(*request->anchor))
                  : std::nullopt);
  writer.Field("state", Lowercase(lifelong::ToString(status->state)))
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
      OptionalInt(writer, "request_id", event.relocalization_id);
      writer
          .Field("state", event.outcome.has_value() ? Lowercase(lifelong::ToString(*event.outcome))
                                                    : std::string("pending"))
          .Field("candidates", event.candidates)
          .Field("constraints", event.constraints_added);
      WriteCorrection(writer, event.correction);
      OptionalInt(writer, "superseded_by", event.superseded_by);
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
