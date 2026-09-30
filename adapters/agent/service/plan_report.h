/**
 * @file plan_report.h
 * @author hang chen (chen@hang.plus)
 * @brief What a destructive session operation would do, and the token that locks apply to it.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_PLAN_REPORT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_PLAN_REPORT_H_

#include <optional>
#include <string>
#include <vector>

#include "lifelong/anchors/anchor_store.h"
#include "lifelong/ids.h"
#include "service/json_writer.h"

namespace evergreenslam::agent {

struct PlanReport {
  std::string op;
  std::optional<std::string> rejection;
  std::vector<lifelong::SubmapId> would_delete;
  std::vector<lifelong::SessionId> sessions_affected;
  std::vector<lifelong::AnchorId> anchors_orphaned;
  int at_num_solves = 0;
  // Applying replaces the fed session with a fresh one.
  bool drops_fed = false;
  std::optional<std::string> note;

  // Hex FNV-1a 64 of op|sessions|submaps|anchors, each sorted: which ids the operation touches.
  std::string Token() const;
  // Every Report field except `ok`, which the receipt carries.
  void WriteFields(JsonWriter& writer) const;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_PLAN_REPORT_H_
