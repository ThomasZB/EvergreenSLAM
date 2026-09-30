/**
 * @file plan_report.cc
 * @author hang chen (chen@hang.plus)
 * @brief What a destructive session operation would do, and the token that locks apply to it.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/plan_report.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace evergreenslam::agent {

std::string PlanReport::Token() const {
  std::vector<lifelong::SessionId> sessions = sessions_affected;
  std::vector<lifelong::SubmapId> submaps = would_delete;
  std::vector<lifelong::AnchorId> anchors = anchors_orphaned;
  std::sort(sessions.begin(), sessions.end());
  std::sort(submaps.begin(), submaps.end());
  std::sort(anchors.begin(), anchors.end());

  std::string text = op + "|";
  for (size_t i = 0; i < sessions.size(); ++i) {
    text += (i == 0 ? "" : ",") + std::to_string(sessions[i].session_index);
  }
  text += "|";
  for (size_t i = 0; i < submaps.size(); ++i) {
    text += (i == 0 ? "" : ",") + std::to_string(submaps[i].session_id) + ":" +
            std::to_string(submaps[i].submap_index);
  }
  text += "|";
  for (size_t i = 0; i < anchors.size(); ++i) {
    text += (i == 0 ? "" : ",") + std::to_string(anchors[i]);
  }

  uint64_t hash = 14695981039346656037ull;
  for (const char c : text) {
    hash ^= static_cast<uint8_t>(c);
    hash *= 1099511628211ull;
  }
  char hex[17];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(hash));
  return hex;
}

void PlanReport::WriteFields(JsonWriter& writer) const {
  writer.OptionalField("rejection", rejection);
  writer.Key("would_delete").BeginArray();
  for (const lifelong::SubmapId& id : would_delete) {
    writer.BeginArray().Int(id.session_id).Int(id.submap_index).EndArray();
  }
  writer.EndArray();
  writer.Key("sessions_affected").BeginArray();
  for (const lifelong::SessionId& id : sessions_affected) {
    writer.Int(id.session_index);
  }
  writer.EndArray();
  writer.Key("anchors_orphaned").BeginArray();
  for (const lifelong::AnchorId id : anchors_orphaned) {
    writer.Int(static_cast<int64_t>(id));
  }
  writer.EndArray();
  writer.Field("drops_fed", drops_fed).OptionalField("note", note);
  writer.Field("at_num_solves", at_num_solves).Field("plan_token", Token());
}

}  // namespace evergreenslam::agent
