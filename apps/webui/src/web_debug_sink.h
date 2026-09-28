/**
 * @file web_debug_sink.h
 * @author hang chen (chen@hang.plus)
 * @brief Serves the pipeline's intermediates to a browser over HTTP.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_APPS_WEBUI_WEB_DEBUG_SINK_H_
#define EVERGREENSLAM_APPS_WEBUI_WEB_DEBUG_SINK_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "debug/debug_sink.h"

namespace evergreenslam::lifelong {
class PoseGraph;
}  // namespace evergreenslam::lifelong

namespace evergreenslam::webui {

struct WebDebugSinkOption {
  int port = 8080;
  std::string www_dir = EVERGREENSLAM_WEBUI_WWW_DIR;
  // Wall clock, not scan time: a replay may run at any speed.
  double map_period_seconds = 0.2;
  bool serve_submaps = true;
};

// Latest wins, depth one; nothing here blocks the publishing thread.
class WebDebugSink : public debug::DebugSink {
 public:
  using Option = WebDebugSinkOption;

  explicit WebDebugSink(const Option& option = Option());
  ~WebDebugSink() override;

  void PublishScanMatch(common::Time time,
                        const std::unordered_map<std::string, Eigen::Affine2d>& poses,
                        const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
                        double score) override;

  void PublishGlobalMap(const mapping::GridMapu8& grid_map);

  // Non-const because enqueueing is a mutation; the read itself happens on a backend task.
  void PublishPoseGraph(lifelong::PoseGraph& pose_graph);

  // After a map switch: drops the old map's graph, submap textures and trajectory; open pages
  // reload, since they cache textures by the same {session, index} the new map reuses.
  void Reset();

  void WaitForever();

  int port() const { return option_.port; }

 private:
  struct Impl;
  using SubmapKey = std::pair<int, int>;  // session index, submap index

  void CopyGrid(const mapping::GridMapu8& grid_map, const Eigen::Affine2d& grid_pose);
  static std::vector<uint8_t> EncodeGrid(const mapping::GridMapu8& grid_map);

  Option option_;
  std::unique_ptr<Impl> impl_;
  std::thread server_thread_;

  std::mutex mutex_;
  std::condition_variable frame_ready_;
  // Held without its closing brace: the sequence numbers are stamped on at send time.
  std::string latest_frame_;
  uint64_t stream_seq_ = 0;
  // One byte per cell, 0..100 occupancy and 255 unknown.
  std::vector<uint8_t> map_blob_;
  uint64_t map_seq_ = 0;
  // Stamped next to map_seq so the page places the raster with the copy it belongs to.
  Eigen::Affine2d map_pose_ = Eigen::Affine2d::Identity();
  std::vector<uint8_t> global_map_blob_;
  uint64_t global_map_seq_ = 0;
  std::string graph_json_;
  uint64_t graph_seq_ = 0;
  std::map<SubmapKey, std::shared_ptr<const std::vector<uint8_t>>> submap_blobs_;
  std::vector<double> trajectory_;
  uint64_t epoch_ = 0;
  std::chrono::steady_clock::time_point last_map_copy_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> graph_request_pending_{false};
};

}  // namespace evergreenslam::webui

#endif  // EVERGREENSLAM_APPS_WEBUI_WEB_DEBUG_SINK_H_
