/**
 * @file place_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /here, /place/save, /anchors, /anchors/{id} and /init-pose.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "service/backend_task.h"
#include "service/endpoints.h"
#include "service/graph_reads.h"
#include "service/http_reply.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

std::string ToRefusal(lifelong::PoseGraph::SaveAnchorResult::Refusal refusal) {
  using Refusal = lifelong::PoseGraph::SaveAnchorResult::Refusal;
  switch (refusal) {
    case Refusal::NO_KEYFRAME:
      return "no_keyframe";
    case Refusal::NODE_GONE:
      return "node_gone";
    case Refusal::NOT_PERSISTED:
      return "not_persisted";
    case Refusal::OFFSET_NOT_FREE:
      return "offset_not_free";
    case Refusal::NONE:
      break;
  }
  return "";
}

constexpr int kClosestPlaces = 2;
// A place saved from afar is one the robot could see; beyond this the scan is too thin to trust.
constexpr double kMaxOffsetMeters = 3.0;

struct Offset {
  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
};

// `offset=dx,dy,dtheta`, robot frame (x forward, y left).
std::optional<Offset> OffsetParam(const httplib::Request& request) {
  const std::optional<std::string> text = OptionalParam(request, "offset");
  if (!text.has_value()) {
    return std::nullopt;
  }
  std::vector<double> values;
  size_t begin = 0;
  while (true) {
    const size_t comma = text->find(',', begin);
    values.push_back(ParseDouble("offset", text->substr(begin, comma - begin)));
    if (comma == std::string::npos) {
      break;
    }
    begin = comma + 1;
  }
  if (values.size() != 3) {
    throw RequestError("bad_param", "'offset' is dx,dy,dtheta: " + *text);
  }
  return Offset{values[0], values[1], utils::transform::NormalizeAngle(values[2])};
}

lifelong::AnchorId ParseAnchorId(const std::string& text) {
  const int64_t id = ParseInt("anchor", text);
  if (id <= 0) {
    throw RequestError("bad_param", "anchor ids start at 1: " + text);
  }
  return static_cast<lifelong::AnchorId>(id);
}

void WriteResolvedAnchor(JsonWriter& writer, const lifelong::ResolvedAnchor& anchor) {
  writer.Field("anchor", anchor.id)
      .Field("state", AgentState(anchor))
      .OptionalField("orphan_reason", OrphanReasonName(anchor));
  WritePose(writer, "pose", anchor.global_pose);
}

void HandleHere(ServiceContext& context, httplib::Response& response) {
  const std::vector<PlaceFile> places = context.places.Scan().Unique();
  struct Reading {
    RobotReading robot;
    std::vector<lifelong::ResolvedAnchor> resolved;
    BaseAlignment alignment;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  const Reading reading = RunOnBackend(pose_graph, [&] {
    return Reading{ReadRobotOnTask(pose_graph, context.hooks),
                   pose_graph.anchors().ResolveAll(pose_graph.graph()),
                   ReadBaseAlignmentOnTask(pose_graph), NumSolves(pose_graph)};
  });

  struct Candidate {
    PlaceDistance distance;
    lifelong::AnchorId anchor = 0;
    std::string state;
  };
  std::vector<Candidate> candidates;
  if (reading.robot.pose.has_value()) {
    const std::map<lifelong::AnchorId, lifelong::ResolvedAnchor> by_id = ById(reading.resolved);
    for (const PlaceRow& row : ResolvePlaces(places, by_id)) {
      if (!row.pose.has_value()) {
        continue;
      }
      const double distance = (row.pose->translation() - reading.robot.pose->translation()).norm();
      candidates.push_back({{row.path, distance}, row.anchor, row.state});
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    return a.distance.distance_m < b.distance.distance_m;
  });
  std::vector<PlaceDistance> distances;
  for (const Candidate& candidate : candidates) {
    distances.push_back(candidate.distance);
  }
  const std::optional<size_t> current = context.here.Update(distances);

  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", reading.num_solves);
  WritePose(writer, "robot", reading.robot.pose);
  writer.OptionalField("keyframe_age_s", reading.robot.keyframe_age_s)
      .Field("has_frozen_base", reading.alignment.has_frozen_base)
      .Field("aligned_to_base", reading.alignment.aligned_to_base);
  WriteScanMatch(writer, reading.robot.scan_match);
  writer.Key("current");
  if (current.has_value()) {
    const Candidate& place = candidates[*current];
    writer.BeginObject()
        .Field("path", place.distance.path)
        .Field("anchor", place.anchor)
        .Field("dist_m", place.distance.distance_m)
        .EndObject();
  } else {
    writer.Null();
  }
  writer.Key("closest").BeginArray();
  for (size_t i = 0; i < candidates.size() && i < kClosestPlaces; ++i) {
    writer.BeginObject()
        .Field("path", candidates[i].distance.path)
        .Field("anchor", candidates[i].anchor)
        .Field("dist_m", candidates[i].distance.distance_m)
        .Field("state", candidates[i].state)
        .EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandlePlaceSave(ServiceContext& context, const httplib::Request& request,
                     httplib::Response& response) {
  const std::string path = RequiredParam(request, "path");
  const bool keep_scan = BoolParam(request, "scan", true);
  const std::optional<Offset> offset = OffsetParam(request);
  PlaceStore::CheckNodePath(path);
  const std::string node = context.sandbox.Resolve(path).relative;
  std::optional<lifelong::AnchorId> existing = context.places.ReadBinding(node);
  // A `cp -r` copy: rebinding would move the original. A fresh id replaces the copy's place.yaml.
  if (existing.has_value()) {
    const std::vector<std::string> duplicates = context.places.Scan().duplicate_paths;
    if (std::binary_search(duplicates.begin(), duplicates.end(), node)) {
      existing.reset();
    }
  }

  struct Outcome {
    std::optional<std::string> refusal;
    std::optional<lifelong::Anchor> anchor;
    std::optional<lifelong::ResolvedAnchor> resolved;
    bool rebound = false;
    std::optional<double> keyframe_age_s;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  Outcome outcome = RunOnBackend(pose_graph, [&] {
    Outcome result;
    const RobotReading robot = ReadRobotOnTask(pose_graph, context.hooks);
    result.keyframe_age_s = robot.keyframe_age_s;
    const std::optional<lifelong::NodeId> node = pose_graph.last_ingested_node();
    // A place bound now would land in the old map.
    if (context.switch_pending.load()) {
      result.refusal = "switching";
    } else if (!node.has_value()) {
      result.refusal = "no_keyframe";
    } else if (offset.has_value() && std::hypot(offset->dx, offset->dy) > kMaxOffsetMeters) {
      result.refusal = "offset_too_far";
    } else {
      Eigen::Affine2d node_from_anchor = Eigen::Affine2d::Identity();
      if (offset.has_value()) {
        // The offset is in the robot's frame, which runs ahead of the last keyframe.
        const Eigen::Affine2d node_from_robot =
            robot.pose.has_value()
                ? Eigen::Affine2d(pose_graph.graph().node(*node).global_pose.inverse() *
                                  *robot.pose)
                : Eigen::Affine2d::Identity();
        node_from_anchor =
            node_from_robot * utils::transform::FromXYTheta(offset->dx, offset->dy, offset->dtheta);
      }
      result.rebound = existing.has_value() && pose_graph.anchors().Get(*existing).has_value();
      lifelong::PoseGraph::SaveAnchorResult saved = pose_graph.SaveAnchorOnTask(
          keep_scan, result.rebound ? existing : std::nullopt, node_from_anchor);
      result.anchor = std::move(saved.anchor);
      if (saved.refusal != lifelong::PoseGraph::SaveAnchorResult::Refusal::NONE) {
        result.refusal = ToRefusal(saved.refusal);
      } else {
        result.resolved = pose_graph.anchors().Resolve(pose_graph.graph(), result.anchor->id);
      }
    }
    result.num_solves = NumSolves(pose_graph);
    return result;
  });

  // A kill between the task and this write leaves an unreferenced anchor, never a dangling file.
  if (outcome.anchor.has_value() && !outcome.rebound &&
      !context.places.WriteBinding(node, outcome.anchor->id)) {
    outcome.refusal = "fs_error";
  }

  JsonWriter writer = BeginReceipt(outcome.refusal);
  writer.Field("at_num_solves", outcome.num_solves);
  if (outcome.anchor.has_value()) {
    writer.Field("anchor", outcome.anchor->id);
    WriteSubmapId(writer, "submap_id", outcome.anchor->submap_id);
    writer.Field("state", outcome.resolved.has_value() ? AgentState(*outcome.resolved) : "orphan");
  } else {
    writer.NullField("anchor").NullField("submap_id").NullField("state");
  }
  writer.OptionalField("keyframe_age_s", outcome.keyframe_age_s)
      .Field("rebound_existing", outcome.rebound)
      .Key("offset");
  if (offset.has_value()) {
    writer.BeginArray().Double(offset->dx).Double(offset->dy).Double(offset->dtheta).EndArray();
  } else {
    writer.Null();
  }
  writer.EndObject();
  SendJson(response, writer);
}

void HandleAnchor(ServiceContext& context, const httplib::Request& request,
                  httplib::Response& response) {
  const lifelong::AnchorId id = ParseAnchorId(request.matches[1]);
  const bool with_robot = BoolParam(request, "robot", false);
  struct Reading {
    std::optional<lifelong::ResolvedAnchor> resolved;
    RobotReading robot;
    BaseAlignment alignment;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  const Reading reading = RunOnBackend(pose_graph, [&] {
    return Reading{pose_graph.anchors().Resolve(pose_graph.graph(), id),
                   with_robot ? ReadRobotOnTask(pose_graph, context.hooks) : RobotReading{},
                   with_robot ? ReadBaseAlignmentOnTask(pose_graph) : BaseAlignment{},
                   NumSolves(pose_graph)};
  });

  JsonWriter writer = BeginReceipt(
      reading.resolved.has_value() ? std::nullopt : std::optional<std::string>("unknown_anchor"));
  writer.Field("at_num_solves", reading.num_solves);
  if (reading.resolved.has_value()) {
    WriteResolvedAnchor(writer, *reading.resolved);
    WriteSubmapId(writer, "submap_id", reading.resolved->submap_id);
    writer.Field("saved_at_ns", common::ToUnixNanos(reading.resolved->saved_at));
  } else {
    writer.Field("anchor", id);
  }
  if (with_robot) {
    WritePose(writer, "robot", reading.robot.pose);
    writer.Field("has_frozen_base", reading.alignment.has_frozen_base)
        .Field("aligned_to_base", reading.alignment.aligned_to_base);
  }
  writer.EndObject();
  SendJson(response, writer);
}

void HandleAnchors(ServiceContext& context, const httplib::Request& request,
                   httplib::Response& response) {
  const bool with_robot = BoolParam(request, "robot", false);
  struct Reading {
    std::vector<lifelong::ResolvedAnchor> resolved;
    RobotReading robot;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  const Reading reading = RunOnBackend(pose_graph, [&] {
    return Reading{pose_graph.anchors().ResolveAll(pose_graph.graph()),
                   with_robot ? ReadRobotOnTask(pose_graph, context.hooks) : RobotReading{},
                   NumSolves(pose_graph)};
  });

  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", reading.num_solves);
  if (with_robot) {
    WritePose(writer, "robot", reading.robot.pose);
  }
  writer.Key("anchors").BeginArray();
  for (const lifelong::ResolvedAnchor& anchor : reading.resolved) {
    writer.BeginObject();
    WriteResolvedAnchor(writer, anchor);
    writer.EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandleInitPose(ServiceContext& context, const httplib::Request& request,
                    httplib::Response& response) {
  const std::optional<std::string> anchor_param = OptionalParam(request, "anchor");
  std::optional<lifelong::AnchorId> anchor;
  Eigen::Affine2d given = Eigen::Affine2d::Identity();
  if (anchor_param.has_value()) {
    anchor = ParseAnchorId(*anchor_param);
  } else {
    given = utils::transform::FromXYTheta(ParseDouble("x", RequiredParam(request, "x")),
                                          ParseDouble("y", RequiredParam(request, "y")),
                                          ParseDouble("theta", RequiredParam(request, "theta")));
  }

  struct Outcome {
    std::optional<std::string> refusal;
    std::optional<Eigen::Affine2d> pose;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  const Outcome outcome = RunOnBackend(pose_graph, [&] {
    Outcome result;
    result.num_solves = NumSolves(pose_graph);
    if (anchor.has_value()) {
      const std::optional<lifelong::ResolvedAnchor> resolved =
          pose_graph.anchors().Resolve(pose_graph.graph(), *anchor);
      if (!resolved.has_value()) {
        result.refusal = "unknown_anchor";
        return result;
      }
      if (!resolved->global_pose.has_value()) {
        result.refusal = "unresolvable";
        return result;
      }
      result.pose = *resolved->global_pose;
    } else {
      result.pose = given;
    }
    pose_graph.SetInitialPose(*result.pose);
    return result;
  });

  JsonWriter writer = BeginReceipt(outcome.refusal);
  writer.Field("at_num_solves", outcome.num_solves);
  WritePose(writer, "pose", outcome.pose);
  writer.EndObject();
  SendJson(response, writer);
}

}  // namespace

void RegisterPlaceEndpoints(httplib::Server& server, ServiceContext& context) {
  server.Get("/here",
             TaskRoute(context, [&context](const httplib::Request&, httplib::Response& response) {
               HandleHere(context, response);
             }));
  server.Post("/place/save", TaskRoute(context, [&context](const httplib::Request& request,
                                                           httplib::Response& response) {
                HandlePlaceSave(context, request, response);
              }));
  server.Get("/anchors", TaskRoute(context, [&context](const httplib::Request& request,
                                                       httplib::Response& response) {
               HandleAnchors(context, request, response);
             }));
  server.Get(R"(/anchors/([^/]+))", TaskRoute(context, [&context](const httplib::Request& request,
                                                                  httplib::Response& response) {
               HandleAnchor(context, request, response);
             }));
  server.Post("/init-pose", TaskRoute(context, [&context](const httplib::Request& request,
                                                          httplib::Response& response) {
                HandleInitPose(context, request, response);
              }));
}

}  // namespace evergreenslam::agent
