/**
 * @file pose_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief GET /pose: the host's newest pose in the global frame, without a backend task.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <chrono>
#include <cstdint>
#include <optional>

#include "service/graph/graph_reads.h"
#include "service/http/endpoints.h"
#include "service/http/http_reply.h"

namespace evergreenslam::agent {
namespace {

struct PoseReading {
  std::optional<HostPose> sample;
  std::optional<Eigen::Affine2d> pose;
};

PoseReading ReadPose(const ServiceContext& context) {
  PoseReading reading;
  if (!context.hooks.current_pose) {
    return reading;
  }
  const uint64_t before = context.drop_seqlock.load();
  reading.sample = context.hooks.current_pose();
  const std::optional<Eigen::Affine2d> session_to_global =
      context.pose_graph.ActiveSessionToGlobal();
  const uint64_t after = context.drop_seqlock.load();
  if (reading.sample.has_value() && session_to_global.has_value() && before == after &&
      before % 2 == 0) {
    reading.pose = Eigen::Affine2d(*session_to_global * reading.sample->pose.local_pose);
  }
  return reading;
}

void HandlePose(const ServiceContext& context, httplib::Response& response) {
  const PoseReading reading = ReadPose(context);
  JsonWriter writer = BeginReceipt(std::nullopt);
  if (reading.sample.has_value()) {
    const HostPose& sample = *reading.sample;
    const double age_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count();
    writer.Field("seq", sample.seq)
        .Field("stamp_ns", common::ToUnixNanos(sample.pose.time))
        .Field("age_s", age_s);
  } else {
    writer.NullField("seq").NullField("stamp_ns").NullField("age_s");
  }
  WritePose(writer, "pose", reading.pose);
  writer.Key("velocity");
  if (reading.sample.has_value()) {
    writer.BeginObject()
        .Field("vx", reading.sample->velocity.x())
        .Field("vy", reading.sample->velocity.y())
        .Field("omega", reading.sample->velocity.z())
        .EndObject();
  } else {
    writer.Null();
  }
  writer.EndObject();
  SendJson(response, writer);
}

}  // namespace

void RegisterPoseEndpoints(Routes& server, ServiceContext& context) {
  server.ConcurrentGet("/pose",
                       Guarded([&context](const httplib::Request&, httplib::Response& response) {
                         HandlePose(context, response);
                       }));
}

}  // namespace evergreenslam::agent
