/**
 * @file pose_graph_publisher.h
 * @author hang chen (chen@hang.plus)
 * @brief The lifelong pose graph as rviz topics: submap list, textures, markers and global map.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_ROS2_POSE_GRAPH_PUBLISHER_H_
#define EVERGREENSLAM_ADAPTERS_ROS2_POSE_GRAPH_PUBLISHER_H_

#include <atomic>
#include <cstdint>
#include <evergreenslam_msgs/msg/submap_list.hpp>
#include <evergreenslam_msgs/msg/submap_texture.hpp>
#include <evergreenslam_msgs/srv/submap_query.hpp>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mapping/grid_mapping/grid_map.h"

namespace evergreenslam::lifelong {
class PoseGraph;
}  // namespace evergreenslam::lifelong

namespace evergreenslam::ros2 {

// Reads the graph only from backend tasks; the executor thread enqueues and answers queries from
// a texture cache those tasks refresh.
class PoseGraphPublisher {
 public:
  using GlobalMapHook = std::function<void(const mapping::GridMapu8&)>;

  PoseGraphPublisher(rclcpp::Node& node, lifelong::PoseGraph& pose_graph, std::string map_frame,
                     GlobalMapHook global_map_hook = nullptr);
  ~PoseGraphPublisher();

  PoseGraphPublisher(const PoseGraphPublisher&) = delete;
  PoseGraphPublisher& operator=(const PoseGraphPublisher&) = delete;

  // Executor thread: enqueues one graph read and one global map assembly, if none is in flight.
  void Publish();

 private:
  using SubmapKey = std::pair<int, int>;  // session index, submap index
  struct CachedTexture {
    int version = 0;
    evergreenslam_msgs::msg::SubmapTexture texture;
  };

  void ReadGraphOnTask();
  void PublishGlobalMapOnTask(const mapping::GridMapu8& grid);
  void HandleQuery(const evergreenslam_msgs::srv::SubmapQuery::Request::SharedPtr request,
                   evergreenslam_msgs::srv::SubmapQuery::Response::SharedPtr response) const;

  lifelong::PoseGraph& pose_graph_;
  const std::string map_frame_;
  const GlobalMapHook global_map_hook_;

  rclcpp::Publisher<evergreenslam_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraint_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Service<evergreenslam_msgs::srv::SubmapQuery>::SharedPtr query_service_;

  std::atomic<bool> graph_pending_{false};
  std::atomic<bool> map_pending_{false};
  // Written by the graph task, read by the map task, which carries no time of its own.
  std::atomic<int64_t> newest_node_nanos_{0};

  mutable std::mutex mutex_;
  std::map<SubmapKey, std::shared_ptr<const CachedTexture>> textures_;

  // Backend task only.
  std::optional<nav_msgs::msg::OccupancyGrid> last_map_;
};

}  // namespace evergreenslam::ros2

#endif  // EVERGREENSLAM_ADAPTERS_ROS2_POSE_GRAPH_PUBLISHER_H_
