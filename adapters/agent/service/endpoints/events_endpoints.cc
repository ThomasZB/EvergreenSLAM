/**
 * @file events_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /events: the backend's bounded event log, paged by sequence number.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstdint>
#include <optional>
#include <string>

#include "lifelong/pose_graph_events.h"
#include "service/graph/relocalization_report.h"
#include "service/http/endpoints.h"
#include "service/http/http_reply.h"

namespace evergreenslam::agent {
namespace {

void HandleEvents(ServiceContext& context, const std::string& instance_id,
                  const httplib::Request& request, httplib::Response& response) {
  const std::optional<std::string> since_param = OptionalParam(request, "since");
  const int64_t since = since_param.has_value() ? ParseInt("since", *since_param) : 0;
  if (since < 0) {
    throw RequestError("bad_param", "'since' is a sequence number, 0 or more");
  }
  const lifelong::PoseGraphEvents::Page page = context.pose_graph.EventsSince(since);
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("instance_id", instance_id)
      .Field("first_seq", page.first_seq)
      .Field("next_seq", page.next_seq)
      .Field("lost", page.lost)
      .Key("events")
      .BeginArray();
  for (const lifelong::PoseGraphEvent& event : page.events) {
    WriteEvent(writer, event);
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

}  // namespace

void RegisterEventsEndpoints(Routes& server, ServiceContext& context,
                             const std::string& instance_id) {
  server.Get("/events", TaskRoute(context, [&context, &instance_id](const httplib::Request& request,
                                                                    httplib::Response& response) {
               HandleEvents(context, instance_id, request, response);
             }));
}

}  // namespace evergreenslam::agent
