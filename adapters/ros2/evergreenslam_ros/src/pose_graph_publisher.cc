/**
 * @file pose_graph_publisher.cc
 * @author hang chen (chen@hang.plus)
 * @brief The lifelong pose graph as rviz topics: submap list, textures, markers and global map.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "pose_graph_publisher.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "laser_scan_converter.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/probability_values.h"
#include "mapping/submap.h"
#include "utils/transform/transform.h"

namespace evergreenslam::ros2 {
namespace {

using evergreenslam_msgs::msg::SubmapTexture;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr uint8_t kUnknownCell = 255;
constexpr double kTrajectoryLineWidth = 0.07;
constexpr double kConstraintLineWidth = 0.025;
// Sparse groups draw above the dense intra edges and the map, as in cartographer_ros.
constexpr double kSparseMarkerHeight = 0.1;
// rviz's billboard lines cap out at this many points per marker (cartographer_ros splits too).
constexpr size_t kMaxPointsPerMarker = 16384;

geometry_msgs::msg::Pose ToPose(const Eigen::Affine2d& pose) {
  geometry_msgs::msg::Pose out;
  out.position.x = pose.translation().x();
  out.position.y = pose.translation().y();
  const double yaw = utils::transform::GetYaw(pose);
  out.orientation.w = std::cos(0.5 * yaw);
  out.orientation.z = std::sin(0.5 * yaw);
  return out;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector2d& point) {
  geometry_msgs::msg::Point out;
  out.x = point.x();
  out.y = point.y();
  return out;
}

std_msgs::msg::ColorRGBA MakeColor(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// Golden angle hue, the same sequence the webui tints submaps with.
std_msgs::msg::ColorRGBA SessionColor(int session_index, double alpha) {
  const double hue = std::fmod(session_index * 137.508, 360.0);
  const double saturation = 0.7;
  const double lightness = 0.55;
  const double c = (1.0 - std::abs(2.0 * lightness - 1.0)) * saturation;
  const double x = c * (1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0));
  const double m = lightness - 0.5 * c;
  const int sector = static_cast<int>(hue / 60.0) % 6;
  const double table[6][3] = {{c, x, 0}, {x, c, 0}, {0, c, x}, {0, x, c}, {x, 0, c}, {c, 0, x}};
  return MakeColor(table[sector][0] + m, table[sector][1] + m, table[sector][2] + m, alpha);
}

Marker MakeMarker(const std::string& frame, const builtin_interfaces::msg::Time& stamp,
                  const std::string& ns, int id, int32_t type, double width) {
  Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  // An all-zero quaternion is an rviz warning on every marker of every cycle.
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  return marker;
}

Marker MakeDeleteAll(const std::string& frame, const builtin_interfaces::msg::Time& stamp) {
  Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.action = Marker::DELETEALL;
  marker.pose.orientation.w = 1.0;
  return marker;
}

// Splits into successive ids within the marker's namespace; a strip repeats the joint point.
void AppendSplit(MarkerArray& array, Marker marker) {
  if (marker.points.size() <= kMaxPointsPerMarker) {
    array.markers.push_back(std::move(marker));
    return;
  }
  const bool strip = marker.type == Marker::LINE_STRIP;
  const std::vector<geometry_msgs::msg::Point> points = std::move(marker.points);
  const int base_id = marker.id;
  size_t begin = 0;
  for (int chunk = 0; begin + 1 < points.size(); ++chunk) {
    // A LINE_LIST consumes pairs, so its chunk boundary stays even.
    const size_t step = strip ? kMaxPointsPerMarker : kMaxPointsPerMarker & ~size_t{1};
    const size_t end = std::min(points.size(), begin + step);
    marker.id = base_id + chunk;
    marker.points.assign(points.begin() + begin, points.begin() + end);
    array.markers.push_back(marker);
    begin = strip ? end - 1 : end;
  }
}

std::optional<Eigen::Affine2d> PoseOf(const lifelong::PoseGraphData& data,
                                      const lifelong::VariableId& id) {
  if (!data.HasVariable(id)) {
    return std::nullopt;
  }
  return id.kind == lifelong::VariableId::Kind::NODE ? data.node(id.node_id()).global_pose
                                                     : data.submap(id.submap_id()).global_pose;
}

SubmapTexture EncodeTexture(const mapping::GridMapu8& grid) {
  SubmapTexture texture;
  texture.width = grid.width();
  texture.height = grid.height();
  texture.resolution = grid.resolution();
  texture.slice_pose.position.x = grid.origin_x();
  texture.slice_pose.position.y = grid.origin_y();
  texture.slice_pose.orientation.w = 1.0;
  texture.cells.reserve(grid.data().size());
  for (const uint8_t value : grid.data()) {
    texture.cells.push_back(
        mapping::IsKnownValue(value)
            ? static_cast<uint8_t>(std::lround(100.0 * mapping::ValueToProbability(value)))
            : kUnknownCell);
  }
  return texture;
}

bool SameGrid(const nav_msgs::msg::OccupancyGrid& a, const nav_msgs::msg::OccupancyGrid& b) {
  return a.info.width == b.info.width && a.info.height == b.info.height &&
         a.info.resolution == b.info.resolution &&
         a.info.origin.position.x == b.info.origin.position.x &&
         a.info.origin.position.y == b.info.origin.position.y && a.data == b.data;
}

}  // namespace

PoseGraphPublisher::PoseGraphPublisher(rclcpp::Node& node, lifelong::PoseGraph& pose_graph,
                                       std::string map_frame, GlobalMapHook global_map_hook)
    : pose_graph_(pose_graph),
      map_frame_(std::move(map_frame)),
      global_map_hook_(std::move(global_map_hook)) {
  const rclcpp::QoS qos = rclcpp::QoS(1).transient_local();
  submap_list_publisher_ =
      node.create_publisher<evergreenslam_msgs::msg::SubmapList>("submap_list", qos);
  trajectory_publisher_ = node.create_publisher<MarkerArray>("trajectory_node_list", qos);
  constraint_publisher_ = node.create_publisher<MarkerArray>("constraint_list", qos);
  map_publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
  query_service_ = node.create_service<evergreenslam_msgs::srv::SubmapQuery>(
      "submap_query", [this](const evergreenslam_msgs::srv::SubmapQuery::Request::SharedPtr request,
                             evergreenslam_msgs::srv::SubmapQuery::Response::SharedPtr response) {
        HandleQuery(request, response);
      });
}

PoseGraphPublisher::~PoseGraphPublisher() {
  // Tasks in flight hold `this`; the timer that enqueues them is already gone.
  pose_graph_.Drain();
}

void PoseGraphPublisher::Publish() {
  if (!graph_pending_.exchange(true)) {
    pose_graph_.Enqueue([this] {
      ReadGraphOnTask();
      graph_pending_ = false;
    });
  }
  if (!map_pending_.exchange(true)) {
    pose_graph_.AssembleGlobalMapAsync([this](mapping::GridMapu8 grid) {
      PublishGlobalMapOnTask(grid);
      map_pending_ = false;
    });
  }
}

void PoseGraphPublisher::ReadGraphOnTask() {
  const lifelong::PoseGraphData& data = pose_graph_.graph();
  if (data.nodes().empty()) {
    return;
  }
  // Sensor time: a bag on sim time would put node->now() outside rviz's tf window.
  int64_t newest_nanos = 0;
  for (const auto& [id, session] : data.sessions()) {
    // A frozen session loaded from disk carries the clock of the run that recorded it.
    if (!session.frozen() && !session.node_ids.empty()) {
      newest_nanos = std::max(newest_nanos, common::ToUnixNanos(session.last_node_time));
    }
  }
  if (newest_nanos == 0) {
    for (const auto& [id, node] : data.nodes()) {
      newest_nanos = std::max(newest_nanos, common::ToUnixNanos(node.constant_data.time));
    }
  }
  if (newest_nanos == 0) {
    return;
  }
  newest_node_nanos_ = newest_nanos;
  const builtin_interfaces::msg::Time stamp = ToRosTime(common::FromUnixNanos(newest_nanos));

  // --- submap list, and the version each texture must carry -------------------------------
  evergreenslam_msgs::msg::SubmapList list;
  list.header.stamp = stamp;
  list.header.frame_id = map_frame_;
  struct Candidate {
    SubmapKey key;
    std::shared_ptr<const mapping::Submap> submap;
    int version;
    bool finished;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(data.submaps().size());
  for (const auto& [id, record] : data.submaps()) {
    // Version before the grid: a texture may then be newer than its version, never older, so a
    // viewer that re-queries on a version change never keeps a stale picture.
    const bool finished = record.submap->finished();
    const int version = record.submap->num_scans();
    evergreenslam_msgs::msg::SubmapEntry entry;
    entry.session_id = id.session_id;
    entry.submap_index = id.submap_index;
    entry.submap_version = version;
    entry.pose = ToPose(record.global_pose);
    entry.is_finished = finished;
    entry.is_frozen = data.HasSession(SessionOf(id)) && data.session(SessionOf(id)).frozen();
    list.submap.push_back(entry);
    candidates.push_back(
        {SubmapKey{id.session_id, id.submap_index}, record.submap, version, finished});
  }

  // --- trajectories -----------------------------------------------------------------------
  MarkerArray trajectories;
  trajectories.markers.push_back(MakeDeleteAll(map_frame_, stamp));
  for (const auto& [id, session] : data.sessions()) {
    if (session.node_ids.size() < 2) {
      continue;
    }
    Marker marker = MakeMarker(map_frame_, stamp, "Session " + std::to_string(id.session_index),
                               id.session_index, Marker::LINE_STRIP, kTrajectoryLineWidth);
    marker.color = SessionColor(id.session_index, session.frozen() ? 0.5 : 1.0);
    for (const lifelong::NodeId& node_id : session.node_ids) {
      if (!data.HasNode(node_id)) {
        continue;
      }
      marker.points.push_back(ToPoint(data.node(node_id).global_pose.translation()));
    }
    if (marker.points.size() < 2) {
      continue;
    }
    AppendSplit(trajectories, std::move(marker));
  }

  // --- constraints ------------------------------------------------------------------------
  MarkerArray constraints;
  constraints.markers.push_back(MakeDeleteAll(map_frame_, stamp));
  // Ids start at 0 in every namespace: a split marker takes the following ids of its own.
  Marker intra = MakeMarker(map_frame_, stamp, "Intra constraints", 0, Marker::LINE_LIST,
                            kConstraintLineWidth);
  intra.color = MakeColor(0.7, 0.7, 0.7, 0.6);
  Marker inter = MakeMarker(map_frame_, stamp, "Inter constraints", 0, Marker::LINE_LIST,
                            kConstraintLineWidth);
  inter.color = MakeColor(0.1, 0.9, 0.2, 1.0);
  inter.pose.position.z = kSparseMarkerHeight;
  Marker prior = MakeMarker(map_frame_, stamp, "Prior constraints", 0, Marker::LINE_LIST,
                            kConstraintLineWidth);
  prior.color = MakeColor(1.0, 0.75, 0.1, 1.0);
  prior.pose.position.z = kSparseMarkerHeight;
  Marker recovered = MakeMarker(map_frame_, stamp, "Recovered constraints", 0, Marker::LINE_LIST,
                                kConstraintLineWidth);
  recovered.color = MakeColor(0.9, 0.2, 0.9, 1.0);
  recovered.pose.position.z = kSparseMarkerHeight;
  Marker residual =
      MakeMarker(map_frame_, stamp, "Inter residuals", 0, Marker::LINE_LIST, kConstraintLineWidth);
  residual.color = MakeColor(1.0, 0.2, 0.2, 1.0);
  residual.pose.position.z = kSparseMarkerHeight;

  for (const lifelong::Constraint& constraint : data.constraints()) {
    const std::optional<Eigen::Affine2d> from_pose = PoseOf(data, constraint.from);
    if (!from_pose.has_value()) {
      continue;
    }
    std::optional<Eigen::Affine2d> to_pose;
    if (constraint.to.has_value()) {
      to_pose = PoseOf(data, *constraint.to);
      if (!to_pose.has_value()) {
        continue;
      }
    }
    // A PRIOR's relative_pose is absolute, so its far end is the anchor itself.
    const Eigen::Vector2d target = to_pose.has_value()
                                       ? Eigen::Vector2d(to_pose->translation())
                                       : Eigen::Vector2d(constraint.relative_pose.translation());
    Marker* marker = &intra;
    if (constraint.recovered) {
      marker = &recovered;
    } else {
      switch (constraint.type) {
        case lifelong::Constraint::Type::INTRA_SUBMAP:
          marker = &intra;
          break;
        case lifelong::Constraint::Type::INTER_SUBMAP:
          marker = &inter;
          break;
        case lifelong::Constraint::Type::PRIOR:
          marker = &prior;
          break;
      }
    }
    marker->points.push_back(ToPoint(from_pose->translation()));
    marker->points.push_back(ToPoint(target));

    const bool is_loop =
        constraint.recovered || constraint.type == lifelong::Constraint::Type::INTER_SUBMAP;
    if (is_loop && to_pose.has_value()) {
      const Eigen::Affine2d predicted = *from_pose * constraint.relative_pose;
      residual.points.push_back(ToPoint(predicted.translation()));
      residual.points.push_back(ToPoint(to_pose->translation()));
    }
  }
  for (Marker* marker : {&intra, &inter, &prior, &recovered, &residual}) {
    // An empty LINE_LIST is an rviz warning.
    if (marker->points.size() >= 2) {
      AppendSplit(constraints, std::move(*marker));
    }
  }

  // --- textures ---------------------------------------------------------------------------
  std::map<SubmapKey, std::shared_ptr<const CachedTexture>> textures;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const Candidate& candidate : candidates) {
      const auto found = textures_.find(candidate.key);
      if (found != textures_.end() && found->second->version == candidate.version) {
        textures.emplace(candidate.key, found->second);
      }
    }
  }
  for (const Candidate& candidate : candidates) {
    if (textures.count(candidate.key) != 0) {
      continue;
    }
    // Snapshot() on an unfinished submap races the frontend's InsertScan; the copy locks.
    std::optional<mapping::GridMapu8> owned;
    if (!candidate.finished) {
      owned = candidate.submap->SnapshotCopy();
    }
    const mapping::GridMapu8& grid = candidate.finished ? candidate.submap->Snapshot() : *owned;
    if (grid.width() == 0 || grid.height() == 0) {
      continue;
    }
    auto cached = std::make_shared<CachedTexture>();
    cached->version = candidate.version;
    cached->texture = EncodeTexture(grid);
    textures.emplace(candidate.key, std::move(cached));
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    textures_ = std::move(textures);
  }

  submap_list_publisher_->publish(list);
  trajectory_publisher_->publish(trajectories);
  constraint_publisher_->publish(constraints);
}

void PoseGraphPublisher::PublishGlobalMapOnTask(const mapping::GridMapu8& grid) {
  if (grid.width() == 0 || grid.height() == 0) {
    return;
  }
  const int64_t nanos = newest_node_nanos_.load();
  if (nanos == 0) {
    return;
  }
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = ToRosTime(common::FromUnixNanos(nanos));
  message.header.frame_id = map_frame_;
  message.info.resolution = grid.resolution();
  message.info.width = grid.width();
  message.info.height = grid.height();
  message.info.origin.position.x = grid.origin_x();
  message.info.origin.position.y = grid.origin_y();
  message.info.origin.orientation.w = 1.0;
  message.data.resize(static_cast<size_t>(grid.width()) * grid.height());
  for (size_t i = 0; i < message.data.size(); ++i) {
    const uint8_t value = grid.data()[i];
    message.data[i] =
        mapping::IsKnownValue(value)
            ? static_cast<int8_t>(std::lround(100.0 * mapping::ValueToProbability(value)))
            : static_cast<int8_t>(-1);
  }
  if (last_map_.has_value() && SameGrid(*last_map_, message)) {
    return;
  }
  map_publisher_->publish(message);
  if (global_map_hook_ != nullptr) {
    global_map_hook_(grid);
  }
  last_map_ = std::move(message);
}

void PoseGraphPublisher::HandleQuery(
    const evergreenslam_msgs::srv::SubmapQuery::Request::SharedPtr request,
    evergreenslam_msgs::srv::SubmapQuery::Response::SharedPtr response) const {
  std::shared_ptr<const CachedTexture> cached;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto found = textures_.find(SubmapKey{request->session_id, request->submap_index});
    if (found != textures_.end()) {
      cached = found->second;
    }
  }
  if (cached == nullptr) {
    response->submap_version = 0;
    response->error_message = "no texture for submap (" + std::to_string(request->session_id) +
                              ", " + std::to_string(request->submap_index) +
                              "): unknown, trimmed or still empty";
    return;
  }
  response->error_message.clear();
  response->submap_version = cached->version;
  response->texture = cached->texture;
}

}  // namespace evergreenslam::ros2
