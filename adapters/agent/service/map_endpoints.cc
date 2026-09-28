/**
 * @file map_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /snapshot and /view: one task copies out everything drawn or written, then files.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <glog/logging.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "common/file.h"
#include "lifelong/global_map.h"
#include "service/backend_task.h"
#include "service/endpoints.h"
#include "service/graph_reads.h"
#include "service/http_reply.h"

#if __has_include("service/render/renderer.h")
#include "service/render/renderer.h"
#define EVERGREENSLAM_AGENT_WITH_RENDERER 1
#endif

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::SessionId;

void HandleSnapshot(ServiceContext& context, httplib::Response& response) {
  const std::vector<PlaceFile> places = context.places.Scan().places;
  PoseGraph& pose_graph = context.pose_graph;
  struct Reading {
    SnapshotInput input;
    std::vector<lifelong::ResolvedAnchor> resolved;
  };
  Reading reading = RunOnBackend(pose_graph, [&] {
    Reading result;
    const lifelong::PoseGraphData& graph = pose_graph.graph();
    SnapshotInput& input = result.input;
    input.grid = lifelong::AssembleGlobalMap(graph, 0.0, /*only_finished=*/true);
    input.nodes.reserve(graph.nodes().size());
    for (const auto& [id, node] : graph.nodes()) {
      input.nodes.push_back({node.constant_data.time, node.global_pose, id.session_id});
    }
    const std::optional<SessionId> fed = pose_graph.session_manager().fed_session();
    for (const auto& [id, session] : graph.sessions()) {
      input.sessions.push_back(
          {id.session_index, session.frozen() ? "frozen" : (fed == id ? "fed" : "floating"),
           static_cast<int>(session.node_ids.size()), static_cast<int>(session.submap_ids.size())});
    }
    result.resolved = pose_graph.anchors().ResolveAll(graph);
    input.robot = ReadRobotOnTask(pose_graph, context.hooks).pose;
    if (fed.has_value()) {
      input.fed_session = fed->session_index;
    }
    input.boot_count = pose_graph.map_manager()->boot_count();
    input.num_solves = NumSolves(pose_graph);
    return result;
  });
  reading.input.generated_at = common::Now();
  reading.input.places = ResolvePlaces(places, ById(reading.resolved));

  const std::optional<SnapshotResult> written = context.snapshots.Export(reading.input);
  const bool indexed = written.has_value() &&
                       context.places.WriteIndex(reading.input.places, reading.input.num_solves);
  JsonWriter writer = BeginReceipt(indexed ? std::nullopt : std::optional<std::string>("fs_error"));
  writer.Field("at_num_solves", reading.input.num_solves);
  if (written.has_value()) {
    writer.Field("seq", written->seq).Field("dir", written->dir).Key("files").BeginArray();
    for (const std::string& file : written->files) {
      writer.String(file);
    }
    writer.EndArray();
  } else {
    writer.NullField("seq").NullField("dir").Key("files").BeginArray().EndArray();
  }
  writer.EndObject();
  SendJson(response, writer);
}

#ifdef EVERGREENSLAM_AGENT_WITH_RENDERER

constexpr size_t kMaxCustomLayers = 4;
const std::set<std::string> kLayers = {"map",    "robot",  "scan",    "trail",
                                       "places", "target", "session", "submaps"};
const std::map<std::string, std::vector<std::string>> kPresets = {
    {"map", {"map", "places"}},
    {"here", {"map", "robot", "scan"}},
    {"route", {"map", "robot", "target", "places"}},
    {"trail", {"map", "trail", "robot"}},
    {"session", {"session", "trail"}},
};

struct ViewRequest {
  std::string preset;
  std::set<std::string> layers;
  std::optional<std::string> target;
  std::optional<int> session;
  std::optional<double> ego;
};

// `target=<path>` and `session=<id>` may ride in the layer list itself.
ViewRequest ParseViewRequest(const httplib::Request& request) {
  ViewRequest view;
  view.preset = OptionalParam(request, "preset").value_or("map");
  view.target = OptionalParam(request, "target");
  if (const std::optional<std::string> session = OptionalParam(request, "session")) {
    view.session = static_cast<int>(ParseInt("session", *session));
  }
  if (const std::optional<std::string> ego = OptionalParam(request, "ego")) {
    view.ego = ParseDouble("ego", *ego);
    if (*view.ego <= 0.0) {
      throw RequestError("bad_param", "ego is a positive radius in metres");
    }
  }
  if (view.preset == "custom") {
    const std::string list = RequiredParam(request, "layers");
    std::vector<std::string> names;
    size_t begin = 0;
    while (begin <= list.size()) {
      const size_t end = std::min(list.find(',', begin), list.size());
      std::string name = list.substr(begin, end - begin);
      begin = end + 1;
      if (name.empty()) {
        continue;
      }
      const size_t equals = name.find('=');
      if (equals != std::string::npos) {
        const std::string value = name.substr(equals + 1);
        name = name.substr(0, equals);
        if (name == "target") {
          view.target = value;
        } else if (name == "session") {
          view.session = static_cast<int>(ParseInt("session", value));
        } else {
          throw RequestError("unknown_layer", "unknown layer: " + name);
        }
      }
      if (kLayers.count(name) == 0) {
        throw RequestError("unknown_layer", "unknown layer: " + name);
      }
      names.push_back(name);
    }
    view.layers.insert(names.begin(), names.end());
    if (view.layers.size() > kMaxCustomLayers) {
      throw RequestError("too_many_layers", "custom views take at most 4 layers; split them");
    }
    if (view.layers.empty()) {
      throw RequestError("bad_param", "no layers");
    }
  } else {
    const auto preset = kPresets.find(view.preset);
    if (preset == kPresets.end()) {
      throw RequestError("bad_param", "unknown preset: " + view.preset);
    }
    view.layers.insert(preset->second.begin(), preset->second.end());
  }
  if (view.layers.count("target") > 0) {
    if (!view.target.has_value()) {
      throw RequestError("bad_param", "the target layer needs target=<path>");
    }
    PlaceStore::CheckNodePath(*view.target);
  }
  return view;
}

// The `custom --layers` value that draws the same view.
std::string LayersParam(const ViewRequest& view) {
  std::string joined;
  for (const std::string& layer : view.layers) {
    std::string item = layer;
    if (layer == "target" && view.target.has_value()) {
      item += "=" + *view.target;
    } else if (layer == "session" && view.session.has_value()) {
      item += "=" + std::to_string(*view.session);
    }
    joined += (joined.empty() ? "" : ",") + item;
  }
  return joined;
}

void HandleView(ServiceContext& context, const httplib::Request& request,
                httplib::Response& response) {
  const ViewRequest view = ParseViewRequest(request);
  const auto has = [&view](const char* layer) { return view.layers.count(layer) > 0; };
  std::optional<lifelong::AnchorId> target_anchor;
  if (view.target.has_value() && has("target")) {
    context.sandbox.Resolve(*view.target);
    target_anchor = context.places.ReadBinding(*view.target);
    if (!target_anchor.has_value()) {
      JsonWriter writer = BeginReceipt(std::string("no_binding"));
      writer.Field("detail", *view.target + " holds no place.yaml").EndObject();
      SendJson(response, writer);
      return;
    }
  }
  const std::vector<PlaceFile> places = context.places.Scan().Unique();

  struct Reading {
    RenderInput input;
    std::optional<std::string> refusal;
    std::vector<lifelong::ResolvedAnchor> resolved;
    int num_solves = 0;
  };
  PoseGraph& pose_graph = context.pose_graph;
  Reading reading = RunOnBackend(pose_graph, [&] {
    Reading result;
    RenderInput& input = result.input;
    input.layers = view.layers;
    input.ego_radius_m = view.ego;
    result.num_solves = NumSolves(pose_graph);
    const lifelong::PoseGraphData& graph = pose_graph.graph();
    const SessionId fed = *pose_graph.session_manager().fed_session();
    const SessionId session = view.session.has_value() ? SessionId{*view.session} : fed;
    if (has("session") && !graph.HasSession(session)) {
      result.refusal = "unknown_session";
      return result;
    }
    if (has("session")) {
      input.grid = lifelong::AssembleGlobalMap(graph, 0.0, /*only_finished=*/true, session);
    } else if (has("map")) {
      input.grid = lifelong::AssembleGlobalMap(graph, 0.0, /*only_finished=*/true);
    }
    input.robot =
        ReadRobotOnTask(pose_graph, context.hooks, has("scan") ? &input.scan : nullptr).pose;
    if (has("trail")) {
      std::vector<std::pair<common::Time, Eigen::Vector2d>> trail;
      for (const auto& [id, node] : graph.nodes()) {
        const bool wanted = has("session")
                                ? id.session_id == session.session_index
                                : id.session_id >= pose_graph.boot_first_session().session_index;
        if (wanted) {
          trail.emplace_back(node.constant_data.time, node.global_pose.translation());
        }
      }
      std::stable_sort(trail.begin(), trail.end(),
                       [](const auto& a, const auto& b) { return a.first < b.first; });
      for (const auto& [time, xy] : trail) {
        input.trail.push_back(xy);
      }
    }
    if (has("submaps")) {
      for (const auto& [id, record] : graph.submaps()) {
        if (record.submap == nullptr || !record.submap->finished()) {
          continue;
        }
        const mapping::GridMapu8& grid = record.submap->Snapshot();
        const double x0 = grid.origin_x();
        const double y0 = grid.origin_y();
        const double x1 = x0 + grid.width() * grid.resolution();
        const double y1 = y0 + grid.height() * grid.resolution();
        input.submap_outlines.push_back({record.global_pose * Eigen::Vector2d(x0, y0),
                                         record.global_pose * Eigen::Vector2d(x1, y0),
                                         record.global_pose * Eigen::Vector2d(x1, y1),
                                         record.global_pose * Eigen::Vector2d(x0, y1)});
      }
    }
    result.resolved = pose_graph.anchors().ResolveAll(graph);
    if (target_anchor.has_value()) {
      const std::optional<lifelong::ResolvedAnchor> target =
          pose_graph.anchors().Resolve(graph, *target_anchor);
      if (!target.has_value()) {
        result.refusal = "no_binding";
      } else if (!target->global_pose.has_value()) {
        result.refusal = "unresolvable";
      }
    }
    return result;
  });
  if (!reading.refusal.has_value() && view.ego.has_value() && !reading.input.robot.has_value()) {
    reading.refusal = "no_robot_pose";
  }
  if (reading.refusal.has_value()) {
    JsonWriter writer = BeginReceipt(reading.refusal);
    writer.Field("at_num_solves", reading.num_solves).EndObject();
    SendJson(response, writer);
    return;
  }

  RenderInput& input = reading.input;
  const std::map<lifelong::AnchorId, lifelong::ResolvedAnchor> by_id = ById(reading.resolved);
  std::vector<PlaceRow> rows = ResolvePlaces(places, by_id);
  // The target gets its own highlighted marker below; numbering it too would draw it twice.
  rows.erase(std::remove_if(rows.begin(), rows.end(),
                            [&](const PlaceRow& row) {
                              return !row.pose.has_value() ||
                                     (target_anchor.has_value() && row.path == *view.target);
                            }),
             rows.end());
  if (input.robot.has_value()) {
    const Eigen::Vector2d robot = input.robot->translation();
    std::stable_sort(rows.begin(), rows.end(), [&robot](const PlaceRow& a, const PlaceRow& b) {
      return (a.pose->translation() - robot).norm() < (b.pose->translation() - robot).norm();
    });
  }
  for (size_t i = 0; i < rows.size(); ++i) {
    input.markers.push_back(
        {static_cast<int>(i + 1), rows[i].pose->translation(), rows[i].path, false});
  }
  if (target_anchor.has_value()) {
    input.markers.push_back(
        {0, by_id.at(*target_anchor).global_pose->translation(), *view.target, true});
  }

  const RenderOutput output = Render(input);
  const int seq = context.next_view_seq++;
  const std::filesystem::path file =
      std::filesystem::path(context.views_dir()) / (SequenceName(seq) + "_" + view.preset + ".png");
  std::error_code error;
  std::filesystem::create_directories(file.parent_path(), error);
  if (!common::WriteFileAtomically(file.string(),
                                   std::string(output.png.begin(), output.png.end()))) {
    LOG(WARNING) << "view not saved to " << file;
  }
  response.set_header("X-EGS-Layers", LayersParam(view));
  response.set_header("X-EGS-Legend", output.legend);
  // Relative to map_dir: a container's absolute path means nothing to a client outside it.
  response.set_header("X-EGS-View", (std::filesystem::path("views") / file.filename()).string());
  response.set_header("X-EGS-Solve", std::to_string(reading.num_solves));
  response.set_content(std::string(output.png.begin(), output.png.end()), "image/png");
}

#endif  // EVERGREENSLAM_AGENT_WITH_RENDERER

}  // namespace

void RegisterMapEndpoints(httplib::Server& server, ServiceContext& context) {
  server.Post("/snapshot",
              TaskRoute(context, [&context](const httplib::Request&, httplib::Response& response) {
                HandleSnapshot(context, response);
              }));
#ifdef EVERGREENSLAM_AGENT_WITH_RENDERER
  server.Get("/view", TaskRoute(context, [&context](const httplib::Request& request,
                                                    httplib::Response& response) {
               HandleView(context, request, response);
             }));
#endif
}

}  // namespace evergreenslam::agent
