/**
 * @file zones_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /zones: every zone.yaml with its polygon in the map frame of one solve.
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <string>
#include <utility>
#include <vector>

#include "service/backend_task.h"
#include "service/endpoints.h"
#include "service/graph_reads.h"
#include "service/http_reply.h"
#include "service/zone_store.h"

namespace evergreenslam::agent {
namespace {

void WritePolygon(JsonWriter& writer, std::string_view key, const Polygon2d& polygon) {
  writer.Key(key).BeginArray();
  for (const Eigen::Vector2d& vertex : polygon) {
    writer.BeginArray().Double(vertex.x()).Double(vertex.y()).EndArray();
  }
  writer.EndArray();
}

void HandleZones(ServiceContext& context, httplib::Response& response) {
  const ZoneScan scan = context.zones.Scan();
  if (scan.error.has_value()) {
    JsonWriter writer = BeginReceipt("fs_error");
    writer.Field("detail", *scan.error).EndObject();
    SendJson(response, writer);
    return;
  }
  const std::vector<ZoneFile>& files = scan.zones;
  struct Reading {
    std::vector<ResolvedZone> zones;
    int num_solves = 0;
  };
  lifelong::PoseGraph& pose_graph = context.pose_graph;
  const Reading reading = RunOnBackend(pose_graph, [&] {
    return Reading{ResolveZonesOnTask(pose_graph, files), NumSolves(pose_graph)};
  });

  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("at_num_solves", reading.num_solves).Key("zones").BeginArray();
  for (const ResolvedZone& zone : reading.zones) {
    writer.BeginObject()
        .Field("path", zone.path)
        .Field("kind", zone.kind)
        .OptionalField("frame", zone.frame)
        .OptionalField("frame_path", zone.frame_path);
    if (zone.anchor.has_value()) {
      writer.Field("anchor", static_cast<int64_t>(*zone.anchor));
    } else {
      writer.NullField("anchor");
    }
    writer.OptionalField("state", zone.state);
    WritePolygon(writer, "polygon", zone.polygon);
    if (zone.polygon_xy.has_value()) {
      WritePolygon(writer, "polygon_xy", *zone.polygon_xy);
    } else {
      writer.NullField("polygon_xy");
    }
    writer.OptionalField("reason", zone.reason).EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

}  // namespace

void RegisterZonesEndpoints(httplib::Server& server, ServiceContext& context) {
  server.Get("/zones",
             TaskRoute(context, [&context](const httplib::Request&, httplib::Response& response) {
               HandleZones(context, response);
             }));
}

}  // namespace evergreenslam::agent
