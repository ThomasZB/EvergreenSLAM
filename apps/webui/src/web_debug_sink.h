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
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "debug/debug_sink.h"

namespace evergreenslam::webui {

struct WebDebugSinkOption {
  int port = 8080;
  std::string www_dir = EVERGREENSLAM_WEBUI_WWW_DIR;
  // Wall clock, not scan time: this bounds how often a human sees a new map, and a replay may
  // run at any speed. The grid copy is the one part of publishing that is not cheap.
  double map_period_seconds = 0.2;
};

// Latest wins with a depth of one: a browser that cannot keep up misses frames, it never slows
// the pipeline down. Nothing here blocks the publishing thread.
class WebDebugSink : public debug::DebugSink {
 public:
  using Option = WebDebugSinkOption;

  explicit WebDebugSink(const Option& option = Option());
  ~WebDebugSink() override;

  void PublishScanMatch(common::Time time,
                        const std::unordered_map<std::string, Eigen::Affine2d>& poses,
                        const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
                        double score) override;

  // The run is over but the map is still worth looking at. Returns when the process is
  // interrupted, so callers that just want the server torn down can skip it.
  void WaitForever();

  int port() const { return option_.port; }

 private:
  struct Impl;

  void CopyGrid(const mapping::GridMapu8& grid_map);

  Option option_;
  std::unique_ptr<Impl> impl_;
  std::thread server_thread_;

  std::mutex mutex_;
  std::condition_variable frame_ready_;
  std::string latest_frame_;
  uint64_t frame_seq_ = 0;
  // Decoded to one byte per cell, 0..100 occupancy and 255 unknown, so the raw cell encoding in
  // probability_values.h stays the only definition of itself.
  std::vector<uint8_t> map_blob_;
  uint64_t map_seq_ = 0;
  // Kept here rather than accumulated in the browser so that opening the page late, or after the
  // run has ended, still shows the whole track.
  std::vector<double> trajectory_;
  std::chrono::steady_clock::time_point last_map_copy_;
  std::atomic<bool> stopping_{false};
};

}  // namespace evergreenslam::webui

#endif  // EVERGREENSLAM_APPS_WEBUI_WEB_DEBUG_SINK_H_
