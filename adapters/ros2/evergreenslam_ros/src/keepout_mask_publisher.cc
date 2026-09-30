/**
 * @file keepout_mask_publisher.cc
 * @author hang chen (chen@hang.plus)
 * @brief The agent's keep-out zones as a nav2 KeepoutFilter mask (nav_msgs/OccupancyGrid).
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "keepout_mask_publisher.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace evergreenslam::ros2 {
namespace {

constexpr double kMaskResolution = 0.05;
constexpr double kMaskMarginM = 0.5;
// 4 MB per build, and the build runs on the backend thread every graph tick.
constexpr double kMaxMaskCells = 4e6;
constexpr double kMaxMaskResolution = 0.8;
constexpr int8_t kKeepout = 100;

bool Inside(const agent::Polygon2d& polygon, const Eigen::Vector2d& p) {
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
        p.x() < a.x() + (p.y() - a.y()) / (b.y() - a.y()) * (b.x() - a.x())) {
      inside = !inside;
    }
  }
  return inside;
}

bool SameMask(const nav_msgs::msg::OccupancyGrid& a, const nav_msgs::msg::OccupancyGrid& b) {
  return a.info.width == b.info.width && a.info.height == b.info.height &&
         a.info.resolution == b.info.resolution &&
         a.info.origin.position.x == b.info.origin.position.x &&
         a.info.origin.position.y == b.info.origin.position.y && a.data == b.data;
}

}  // namespace

std::optional<nav_msgs::msg::OccupancyGrid> BuildKeepoutMask(
    const std::vector<agent::Polygon2d>& polygons, double resolution, double margin_m,
    double max_resolution, double max_cells) {
  Eigen::Vector2d min = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d max = -min;
  for (const agent::Polygon2d& polygon : polygons) {
    for (const Eigen::Vector2d& p : polygon) {
      min = min.cwiseMin(p);
      max = max.cwiseMax(p);
    }
  }
  if (!(min.x() <= max.x())) {
    return ClearingKeepoutMask();
  }
  min -= Eigen::Vector2d::Constant(margin_m);
  max += Eigen::Vector2d::Constant(margin_m);
  if (!min.allFinite() || !max.allFinite()) {
    return std::nullopt;
  }
  const auto cells = [&](double r) {
    return std::ceil((max.x() - min.x()) / r) * std::ceil((max.y() - min.y()) / r);
  };
  while (cells(resolution) > max_cells) {
    resolution *= 2.0;
    if (resolution > max_resolution) {
      return std::nullopt;
    }
  }
  const int width = std::max(1, static_cast<int>(std::ceil((max.x() - min.x()) / resolution)));
  const int height = std::max(1, static_cast<int>(std::ceil((max.y() - min.y()) / resolution)));
  nav_msgs::msg::OccupancyGrid mask;
  mask.info.resolution = static_cast<float>(resolution);
  mask.info.origin.orientation.w = 1.0;
  mask.info.width = width;
  mask.info.height = height;
  mask.info.origin.position.x = min.x();
  mask.info.origin.position.y = min.y();
  mask.data.assign(static_cast<size_t>(width) * height, 0);
  for (const agent::Polygon2d& polygon : polygons) {
    Eigen::Vector2d lo = polygon.front();
    Eigen::Vector2d hi = polygon.front();
    for (const Eigen::Vector2d& p : polygon) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
    const int col0 = std::max(0, static_cast<int>(std::floor((lo.x() - min.x()) / resolution)));
    const int col1 =
        std::min(width - 1, static_cast<int>(std::floor((hi.x() - min.x()) / resolution)));
    const int row0 = std::max(0, static_cast<int>(std::floor((lo.y() - min.y()) / resolution)));
    const int row1 =
        std::min(height - 1, static_cast<int>(std::floor((hi.y() - min.y()) / resolution)));
    for (int row = row0; row <= row1; ++row) {
      for (int col = col0; col <= col1; ++col) {
        const Eigen::Vector2d centre = min + resolution * Eigen::Vector2d(col + 0.5, row + 0.5);
        if (Inside(polygon, centre)) {
          mask.data[static_cast<size_t>(row) * width + col] = kKeepout;
        }
      }
    }
  }
  return mask;
}

nav_msgs::msg::OccupancyGrid ClearingKeepoutMask() {
  nav_msgs::msg::OccupancyGrid mask;
  mask.info.resolution = static_cast<float>(kMaskResolution);
  mask.info.origin.orientation.w = 1.0;
  mask.info.width = 1;
  mask.info.height = 1;
  mask.data.assign(1, 0);
  return mask;
}

std::optional<nav_msgs::msg::OccupancyGrid> NextKeepoutMask(
    const agent::BaseAlignment& alignment,
    const std::optional<std::vector<agent::ResolvedZone>>& zones,
    const nav_msgs::msg::OccupancyGrid& last) {
  // Positions are in the fed session's own frame until it links to the frozen base.
  if ((alignment.has_frozen_base && !alignment.aligned_to_base) || !zones.has_value()) {
    return std::nullopt;
  }
  std::vector<agent::Polygon2d> polygons;
  for (const agent::ResolvedZone& zone : *zones) {
    if (zone.IsActiveKeepout()) {
      polygons.push_back(*zone.polygon_xy);
    }
  }
  std::optional<nav_msgs::msg::OccupancyGrid> mask =
      BuildKeepoutMask(polygons, kMaskResolution, kMaskMarginM, kMaxMaskResolution, kMaxMaskCells);
  if (!mask.has_value()) {
    LOG_EVERY_N(WARNING, 100) << "keep-out zones span too much for one mask: previous mask kept";
    return std::nullopt;
  }
  if (SameMask(last, *mask)) {
    return std::nullopt;
  }
  return mask;
}

KeepoutMaskPublisher::KeepoutMaskPublisher(rclcpp::Node& node, lifelong::PoseGraph& pose_graph,
                                           const agent::ZoneStore& zones, std::string map_frame,
                                           const std::string& topic)
    : pose_graph_(pose_graph),
      zones_(zones),
      map_frame_(std::move(map_frame)),
      last_mask_(ClearingKeepoutMask()) {
  // nav2's costmap filters subscribe transient_local + reliable; a volatile mask never arrives.
  publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::QoS(1).transient_local().reliable());
  // A new map (or a map switch): nav2 must not keep the previous map's zones.
  nav_msgs::msg::OccupancyGrid clear = last_mask_;
  clear.header.frame_id = map_frame_;
  clear.header.stamp = node.now();
  clear.info.map_load_time = clear.header.stamp;
  publisher_->publish(clear);
}

KeepoutMaskPublisher::~KeepoutMaskPublisher() {
  // Tasks in flight hold `this` and `zones_`; the timer that enqueues them is already gone.
  pose_graph_.Drain();
}

void KeepoutMaskPublisher::Publish() {
  if (pending_.exchange(true)) {
    return;
  }
  pose_graph_.Enqueue([this, scan = zones_.Scan()] {
    PublishOnTask(scan);
    pending_ = false;
  });
}

void KeepoutMaskPublisher::PublishOnTask(const agent::ZoneScan& scan) {
  std::optional<std::vector<agent::ResolvedZone>> zones;
  if (!scan.error.has_value()) {
    zones = agent::ResolveZonesOnTask(pose_graph_, scan.zones);
  }
  std::optional<nav_msgs::msg::OccupancyGrid> mask =
      NextKeepoutMask(agent::ReadBaseAlignmentOnTask(pose_graph_), zones, last_mask_);
  if (!mask.has_value()) {
    return;
  }
  mask->header.frame_id = map_frame_;
  const std::optional<common::Time> newest = agent::NewestNodeTime(pose_graph_.graph());
  if (newest.has_value()) {
    mask->header.stamp = rclcpp::Time(common::ToUnixNanos(*newest));
  }
  mask->info.map_load_time = mask->header.stamp;
  publisher_->publish(*mask);
  last_mask_ = std::move(*mask);
}

}  // namespace evergreenslam::ros2
