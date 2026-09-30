/**
 * @file session_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /status, /sessions with plan -> apply for freeze and rm (the fed session through the
 * host's drop hook), /relocalize and /checkpoint.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <glog/logging.h>

#include <optional>
#include <string>
#include <vector>

#include "lifelong/optimization/covariance_evaluator.h"
#include "lifelong/sessions/frozen_links.h"
#include "service/backend_task.h"
#include "service/endpoints.h"
#include "service/graph_reads.h"
#include "service/http_reply.h"
#include "service/plan_report.h"

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::SessionId;

const char* Role(const PoseGraph& pose_graph, SessionId id) {
  if (pose_graph.graph().session(id).frozen()) {
    return "frozen";
  }
  return pose_graph.session_manager().fed_session() == id ? "fed" : "floating";
}

// Backend task. Solves first: the judge reads covariances, only meaningful right after a solve.
PlanReport PlanFreezeOnTask(PoseGraph& pose_graph, bool force, lifelong::FreezeVerdict& verdict) {
  pose_graph.OptimizeOnTask();
  pose_graph.RepublishActiveSessionToGlobal();
  PlanReport report;
  report.op = "freeze";
  const SessionId fed = *pose_graph.session_manager().fed_session();
  lifelong::CovarianceEvaluator evaluator(pose_graph.mutable_optimization());
  verdict = pose_graph.session_manager().freeze_judge().Judge(
      pose_graph.graph(), pose_graph.mutable_optimization(), evaluator, fed);
  const bool waived = force && verdict.rejection == lifelong::FreezeRejection::NOT_ANCHORED;
  if (!verdict.eligible && !waived) {
    report.rejection = lifelong::ToString(verdict.rejection);
  }
  report.sessions_affected = {fed};
  report.at_num_solves = NumSolves(pose_graph);
  return report;
}

constexpr char kDropsFedNote[] =
    "this is the session being mapped now: it is replaced by a fresh one; the robot's pose is lost";

// Backend task, read-only. The fed session goes through the host, which owns the frontend.
PlanReport PlanRemoveOnTask(const PoseGraph& pose_graph, SessionId id,
                            const ServiceContext& context) {
  PlanReport report;
  report.op = "rm";
  report.sessions_affected = {id};
  report.at_num_solves = NumSolves(pose_graph);
  const lifelong::PoseGraphData& graph = pose_graph.graph();
  if (!graph.HasSession(id)) {
    report.rejection = "unknown_session";
    return report;
  }
  // The queued freeze tasks would find the session gone.
  if (pose_graph.session_manager().freezing()) {
    report.rejection = "freezing";
    return report;
  }
  if (pose_graph.session_manager().fed_session() == id) {
    if (context.hooks.drop_session == nullptr) {
      report.rejection = "not_supported";
      return report;
    }
    if (context.switch_pending.load()) {
      report.rejection = "switching";
      return report;
    }
    report.drops_fed = true;
    report.note = kDropsFedNote;
  } else if (graph.session(id).frozen()) {
    report.rejection = "frozen_session";
    return report;
  }
  report.would_delete = graph.session(id).submap_ids;
  for (const lifelong::ResolvedAnchor& anchor : pose_graph.anchors().ResolveAll(graph)) {
    if (anchor.state != lifelong::AnchorState::ORPHAN &&
        lifelong::SessionOf(anchor.submap_id) == id) {
      report.anchors_orphaned.push_back(anchor.id);
    }
  }
  return report;
}

void SendPlan(httplib::Response& response, const PlanReport& report,
              const std::optional<lifelong::FreezeVerdict>& verdict) {
  JsonWriter writer = BeginReceipt(report.rejection);
  report.WriteFields(writer);
  if (verdict.has_value()) {
    writer.Key("verdict")
        .BeginObject()
        .Field("eligible", verdict->eligible)
        .Field("bootstrap", verdict->bootstrap)
        .Field("rejection", lifelong::ToString(verdict->rejection))
        .EndObject();
  }
  writer.EndObject();
  SendJson(response, writer);
}

SessionId SessionParam(const httplib::Request& request) {
  const int64_t id = ParseInt("id", RequiredParam(request, "id"));
  if (id < 0) {
    throw RequestError("bad_param", "session ids are non-negative");
  }
  return SessionId{static_cast<int>(id)};
}

void HandleStatus(ServiceContext& context, httplib::Response& response) {
  const PlaceScan scan = context.places.Scan();
  struct Reading {
    int boot_count = 0;
    int fed = 0;
    BaseAlignment alignment;
    std::string phase;
    int closures = 0;
    int num_anchors = 0;
    int num_orphans = 0;
    int num_solves = 0;
    std::optional<ScanMatch> scan_match;
  };
  PoseGraph& pose_graph = context.pose_graph;
  const Reading reading = RunOnBackend(pose_graph, [&] {
    Reading result;
    const lifelong::PoseGraphData& graph = pose_graph.graph();
    result.scan_match = ReadRobotOnTask(pose_graph, context.hooks).scan_match;
    const SessionId fed = *pose_graph.session_manager().fed_session();
    result.boot_count = pose_graph.map_manager()->boot_count();
    result.fed = fed.session_index;
    result.alignment = ReadBaseAlignmentOnTask(pose_graph);
    result.phase = SessionPhase(pose_graph, fed);
    result.closures = pose_graph.constraint_builder().num_constraints_added();
    result.num_anchors = pose_graph.anchors().size();
    for (const lifelong::ResolvedAnchor& anchor : pose_graph.anchors().ResolveAll(graph)) {
      result.num_orphans += anchor.state == lifelong::AnchorState::ORPHAN ? 1 : 0;
    }
    result.num_solves = NumSolves(pose_graph);
    return result;
  });
  const int closures_delta = reading.closures - context.closures_baseline;
  context.closures_baseline = reading.closures;

  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", reading.num_solves)
      .Field("boot_count", reading.boot_count)
      .Field("fed_session", reading.fed)
      .OptionalField("map", context.map_name.empty() ? std::nullopt
                                                     : std::optional<std::string>(context.map_name))
      .Field("has_frozen_base", reading.alignment.has_frozen_base)
      .Field("aligned_to_base", reading.alignment.aligned_to_base)
      .Field("phase", reading.phase);
  WriteScanMatch(writer, reading.scan_match);
  writer.Field("closures_delta", closures_delta)
      .Field("num_anchors", reading.num_anchors)
      .Field("num_orphans", reading.num_orphans)
      .Field("num_place_files", scan.num_place_files)
      .Key("duplicate_anchor_paths")
      .BeginArray();
  for (const std::string& path : scan.duplicate_paths) {
    writer.String(path);
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandleSessions(ServiceContext& context, httplib::Response& response) {
  PoseGraph& pose_graph = context.pose_graph;
  struct Row {
    int id = 0;
    std::string role;
    int nodes = 0;
    int submaps = 0;
    int anchors = 0;
    std::optional<std::string> phase;
    std::optional<bool> anchored;
    int64_t start_ns = 0;
    int64_t last_node_ns = 0;
  };
  struct Reading {
    int fed = 0;
    std::vector<Row> rows;
    int num_solves = 0;
  };
  const Reading reading = RunOnBackend(pose_graph, [&] {
    Reading result;
    const lifelong::PoseGraphData& graph = pose_graph.graph();
    result.fed = pose_graph.session_manager().fed_session()->session_index;
    const std::vector<lifelong::ResolvedAnchor> anchors = pose_graph.anchors().ResolveAll(graph);
    for (const auto& [id, session] : graph.sessions()) {
      Row row;
      row.id = id.session_index;
      row.role = Role(pose_graph, id);
      row.nodes = static_cast<int>(session.node_ids.size());
      row.submaps = static_cast<int>(session.submap_ids.size());
      for (const lifelong::ResolvedAnchor& anchor : anchors) {
        row.anchors += anchor.state != lifelong::AnchorState::ORPHAN &&
                               lifelong::SessionOf(anchor.submap_id) == id
                           ? 1
                           : 0;
      }
      if (!session.frozen()) {
        row.phase = SessionPhase(pose_graph, id);
        row.anchored = lifelong::HasFrozenLink(graph, id);
      }
      row.start_ns = common::ToUnixNanos(session.start_time);
      row.last_node_ns = common::ToUnixNanos(session.last_node_time);
      result.rows.push_back(std::move(row));
    }
    result.num_solves = NumSolves(pose_graph);
    return result;
  });

  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", reading.num_solves).Field("fed", reading.fed);
  writer.Key("sessions").BeginArray();
  for (const Row& row : reading.rows) {
    writer.BeginObject()
        .Field("id", row.id)
        .Field("role", row.role)
        .Field("nodes", row.nodes)
        .Field("submaps", row.submaps)
        .Field("anchors", row.anchors)
        .OptionalField("phase", row.phase);
    writer.Key("anchored");
    if (row.anchored.has_value()) {
      writer.Bool(*row.anchored);
    } else {
      writer.Null();
    }
    writer.Field("start_ns", row.start_ns).Field("last_node_ns", row.last_node_ns).EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandleFreezePlan(ServiceContext& context, const httplib::Request& request,
                      httplib::Response& response) {
  const bool force = BoolParam(request, "force", false);
  PoseGraph& pose_graph = context.pose_graph;
  lifelong::FreezeVerdict verdict;
  const PlanReport report = RunOnBackend(pose_graph, [&] {
    PlanReport result = PlanFreezeOnTask(pose_graph, force, verdict);
    // A switch would answer the old map.
    if (context.switch_pending.load()) {
      result.rejection = "switching";
    }
    return result;
  });
  SendPlan(response, report, verdict);
}

void HandleFreezeApply(ServiceContext& context, const httplib::Request& request,
                       httplib::Response& response) {
  const bool force = BoolParam(request, "force", false);
  const std::string token = RequiredParam(request, "plan_token");
  PoseGraph& pose_graph = context.pose_graph;
  struct Outcome {
    std::optional<std::string> refusal;
    SessionId fed;
    int num_solves = 0;
  };
  const Outcome outcome = RunOnBackend(pose_graph, [&] {
    lifelong::FreezeVerdict verdict;
    const PlanReport report = PlanFreezeOnTask(pose_graph, force, verdict);
    Outcome result;
    result.fed = report.sessions_affected.front();
    result.num_solves = report.at_num_solves;
    if (context.switch_pending.load()) {
      result.refusal = "switching";
    } else if (report.rejection.has_value() || report.Token() != token) {
      result.refusal = "plan_changed";
      return result;
    }
    // Queued: it runs after this task and does nothing if another session is fed by then.
    pose_graph.FreezeFedSession(result.fed);
    return result;
  });

  JsonWriter writer = BeginReceipt(outcome.refusal);
  writer.Field("at_num_solves", outcome.num_solves);
  if (outcome.refusal.has_value()) {
    writer.NullField("frozen_session").Field("pending", false);
  } else {
    writer.Field("frozen_session", outcome.fed.session_index).Field("pending", true);
  }
  writer.EndObject();
  SendJson(response, writer);
}

void HandleRemovePlan(ServiceContext& context, const httplib::Request& request,
                      httplib::Response& response) {
  const SessionId id = SessionParam(request);
  PoseGraph& pose_graph = context.pose_graph;
  const PlanReport report =
      RunOnBackend(pose_graph, [&] { return PlanRemoveOnTask(pose_graph, id, context); });
  SendPlan(response, report, std::nullopt);
}

void HandleRemoveApply(ServiceContext& context, const httplib::Request& request,
                       httplib::Response& response) {
  using Refusal = PoseGraph::DropFedSessionResult::Refusal;
  const SessionId id = SessionParam(request);
  const std::string token = RequiredParam(request, "plan_token");
  PoseGraph& pose_graph = context.pose_graph;
  struct Outcome {
    std::optional<std::string> refusal;
    PlanReport report;
    SessionId fed;
    int num_solves = 0;
  };
  Outcome outcome = RunOnBackend(pose_graph, [&] {
    Outcome result;
    result.report = PlanRemoveOnTask(pose_graph, id, context);
    if (result.report.rejection == "switching") {
      result.refusal = "switching";
    } else if (result.report.rejection.has_value() || result.report.Token() != token) {
      result.refusal = "plan_changed";
    } else if (!result.report.drops_fed) {
      pose_graph.RemoveSession(id);
      pose_graph.RepublishActiveSessionToGlobal();
    }
    result.fed = *pose_graph.session_manager().fed_session();
    result.num_solves = NumSolves(pose_graph);
    return result;
  });
  const bool drops_fed = !outcome.refusal.has_value() && outcome.report.drops_fed;
  // On the worker thread, never inside a task: the host blocks its frontend and waits on the graph.
  if (drops_fed) {
    const PoseGraph::DropFedSessionResult dropped =
        context.hooks.drop_session(SessionDropRequest{id});
    if (dropped.refusal == Refusal::NOT_FED) {
      outcome.refusal = "session_changed";
    } else if (dropped.refusal == Refusal::FREEZING) {
      outcome.refusal = "freezing";
    } else {
      CHECK(dropped.fed_now.has_value()) << "a drop hook returned success without the new fed id";
      outcome.fed = *dropped.fed_now;
    }
    outcome.num_solves = RunOnBackend(pose_graph, [&] { return NumSolves(pose_graph); });
  }

  JsonWriter writer = BeginReceipt(outcome.refusal);
  writer.Field("at_num_solves", outcome.num_solves);
  if (outcome.refusal.has_value()) {
    writer.NullField("removed").Key("anchors_orphaned").BeginArray().EndArray();
  } else {
    writer.Field("removed", id.session_index)
        .Field("fed_session", outcome.fed.session_index)
        .Key("anchors_orphaned")
        .BeginArray();
    for (const lifelong::AnchorId anchor : outcome.report.anchors_orphaned) {
      writer.Int(static_cast<int64_t>(anchor));
    }
    writer.EndArray().Field("pose_lost", drops_fed);
  }
  writer.EndObject();
  SendJson(response, writer);
}

void HandleRelocalize(ServiceContext& context, httplib::Response& response) {
  PoseGraph& pose_graph = context.pose_graph;
  const int num_solves = RunOnBackend(pose_graph, [&] {
    pose_graph.RelocalizeGlobally();
    return NumSolves(pose_graph);
  });
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", num_solves).EndObject();
  SendJson(response, writer);
}

void HandleCheckpoint(ServiceContext& context, httplib::Response& response) {
  PoseGraph& pose_graph = context.pose_graph;
  struct Reading {
    bool persisted = false;
    int num_checkpoints_written = 0;
    int num_solves = 0;
  };
  const Reading reading = RunOnBackend(pose_graph, [&] {
    const bool persisted = pose_graph.CheckpointOnTask();
    return Reading{persisted, pose_graph.map_manager()->num_checkpoints_written(),
                   NumSolves(pose_graph)};
  });
  JsonWriter writer =
      BeginReceipt(reading.persisted ? std::nullopt : std::optional<std::string>("not_persisted"));
  writer.Field("at_num_solves", reading.num_solves)
      .Field("num_checkpoints_written", reading.num_checkpoints_written)
      .EndObject();
  SendJson(response, writer);
}

}  // namespace

void RegisterSessionEndpoints(httplib::Server& server, ServiceContext& context) {
  const auto route = [&context](auto handler) {
    return TaskRoute(
        context, [&context, handler](const httplib::Request& request, httplib::Response& response) {
          handler(context, request, response);
        });
  };
  server.Get("/status", route([](ServiceContext& c, const httplib::Request&, httplib::Response& r) {
               HandleStatus(c, r);
             }));
  server.Get("/sessions", route([](ServiceContext& c, const httplib::Request&,
                                   httplib::Response& r) { HandleSessions(c, r); }));
  server.Post("/sessions/freeze/plan", route(HandleFreezePlan));
  server.Post("/sessions/freeze/apply", route(HandleFreezeApply));
  server.Post("/sessions/rm/plan", route(HandleRemovePlan));
  server.Post("/sessions/rm/apply", route(HandleRemoveApply));
  server.Post("/relocalize", route([](ServiceContext& c, const httplib::Request&,
                                      httplib::Response& r) { HandleRelocalize(c, r); }));
  server.Post("/checkpoint", route([](ServiceContext& c, const httplib::Request&,
                                      httplib::Response& r) { HandleCheckpoint(c, r); }));
}

}  // namespace evergreenslam::agent
