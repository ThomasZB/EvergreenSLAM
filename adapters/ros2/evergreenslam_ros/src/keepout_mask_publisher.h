/**
 * @file keepout_mask_publisher.h
 * @author hang chen (chen@hang.plus)
 * @brief The agent's keep-out zones as a nav2 KeepoutFilter mask (nav_msgs/OccupancyGrid).
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_ROS2_KEEPOUT_MASK_PUBLISHER_H_
#define EVERGREENSLAM_ADAPTERS_ROS2_KEEPOUT_MASK_PUBLISHER_H_

#include <atomic>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "lifelong/pose_graph.h"
#include "service/graph_reads.h"
#include "service/zone_store.h"

namespace evergreenslam::ros2 {

// Cells whose centres lie inside any polygon are 100 (lethal to nav2's KeepoutFilter), the rest 0.
// Bounds: the polygons' bounding boxes plus `margin_m`; no polygon gives a 1x1 grid of 0. The
// resolution doubles until the grid fits `max_cells`; nullopt when even `max_resolution` does not.
std::optional<nav_msgs::msg::OccupancyGrid> BuildKeepoutMask(
    const std::vector<agent::Polygon2d>& polygons, double resolution, double margin_m,
    double max_resolution, double max_cells);

nav_msgs::msg::OccupancyGrid ClearingKeepoutMask();

// The mask one graph tick publishes; nullopt leaves nav2's current mask in place: positions are
// untrusted (frozen base, not aligned), the scan was incomplete, the mask is too large, or it
// equals `last`. `zones` is nullopt for an incomplete scan.
std::optional<nav_msgs::msg::OccupancyGrid> NextKeepoutMask(
    const agent::BaseAlignment& alignment,
    const std::optional<std::vector<agent::ResolvedZone>>& zones,
    const nav_msgs::msg::OccupancyGrid& last);

// Reads the graph only from backend tasks. Contract: adapters/agent/API.md "Keep-out mask".
class KeepoutMaskPublisher {
 public:
  KeepoutMaskPublisher(rclcpp::Node& node, lifelong::PoseGraph& pose_graph,
                       const agent::ZoneStore& zones, std::string map_frame,
                       const std::string& topic);
  ~KeepoutMaskPublisher();

  KeepoutMaskPublisher(const KeepoutMaskPublisher&) = delete;
  KeepoutMaskPublisher& operator=(const KeepoutMaskPublisher&) = delete;

  // Executor thread: scans memory/ for zone.yaml, then enqueues one resolve-and-publish task
  // unless one is in flight.
  void Publish();

 private:
  void PublishOnTask(const agent::ZoneScan& scan);

  lifelong::PoseGraph& pose_graph_;
  const agent::ZoneStore& zones_;
  const std::string map_frame_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  std::atomic<bool> pending_{false};

  // Backend task only (set in the constructor before any task exists).
  nav_msgs::msg::OccupancyGrid last_mask_;
};

}  // namespace evergreenslam::ros2

#endif  // EVERGREENSLAM_ADAPTERS_ROS2_KEEPOUT_MASK_PUBLISHER_H_
