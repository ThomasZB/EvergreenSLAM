/**
 * @file web_debug_sink.cc
 * @author hang chen (chen@hang.plus)
 * @brief Serves the pipeline's intermediates to a browser over HTTP.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "web_debug_sink.h"

#include <glog/logging.h>

#include <cmath>
#include <cstring>
#include <sstream>
#include <type_traits>

#include "httplib.h"
#include "mapping/grid_mapping/probability_values.h"
#include "utils/transform/transform.h"

namespace evergreenslam::webui {
namespace {

constexpr char kMapMagic[4] = {'E', 'G', 'M', '1'};
constexpr size_t kMapHeaderSize = 4 + 2 * sizeof(int32_t) + 3 * sizeof(double);
constexpr uint8_t kUnknownCell = 255;

template <typename T>
void AppendLittleEndian(std::vector<uint8_t>& blob, const T value) {
  static_assert(std::is_trivially_copyable<T>::value);
  // Every platform this runs on is little endian, and so is the DataView call on the other side.
  const size_t offset = blob.size();
  blob.resize(offset + sizeof(T));
  std::memcpy(blob.data() + offset, &value, sizeof(T));
}

void AppendPose(std::ostringstream& out, const Eigen::Affine2d& pose) {
  out << '[' << pose.translation().x() << ',' << pose.translation().y() << ','
      << utils::transform::GetYaw(pose) << ']';
}

}  // namespace

struct WebDebugSink::Impl {
  httplib::Server server;
};

WebDebugSink::WebDebugSink(const Option& option)
    : option_(option), impl_(std::make_unique<Impl>()) {
  httplib::Server& server = impl_->server;
  server.set_mount_point("/", option_.www_dir);

  server.Get("/map", [this](const httplib::Request&, httplib::Response& response) {
    std::vector<uint8_t> blob;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      blob = map_blob_;
    }
    if (blob.empty()) {
      response.status = 204;
      return;
    }
    response.set_content(reinterpret_cast<const char*>(blob.data()), blob.size(),
                         "application/octet-stream");
  });

  server.Get("/trajectory", [this](const httplib::Request&, httplib::Response& response) {
    std::vector<double> trajectory;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      trajectory = trajectory_;
    }
    if (trajectory.empty()) {
      response.status = 204;
      return;
    }
    response.set_content(reinterpret_cast<const char*>(trajectory.data()),
                         trajectory.size() * sizeof(double), "application/octet-stream");
  });

  server.Get("/events", [this](const httplib::Request&, httplib::Response& response) {
    response.set_chunked_content_provider(
        "text/event-stream", [this, sent = uint64_t{0}](size_t, httplib::DataSink& sink) mutable {
          std::string frame;
          {
            std::unique_lock<std::mutex> lock(mutex_);
            // Times out rather than waiting forever so a finished run still lets the handler
            // notice a closed socket and a shutting down server.
            frame_ready_.wait_for(lock, std::chrono::milliseconds(500),
                                  [this, sent] { return stopping_ || frame_seq_ > sent; });
            if (stopping_) {
              return false;
            }
            if (frame_seq_ > sent) {
              sent = frame_seq_;
              frame = latest_frame_;
            }
          }
          // Outside the lock, always. This is a blocking socket write, and a browser that has
          // stopped reading fills the send buffer and parks here; holding the mutex through that
          // would park the SLAM thread in PublishScanMatch behind it.
          if (frame.empty()) {
            return sink.write(": keepalive\n\n", 13);
          }
          const std::string message = "data: " + frame + "\n\n";
          return sink.write(message.data(), message.size());
        });
  });

  server_thread_ = std::thread([this] {
    if (!impl_->server.listen("0.0.0.0", option_.port)) {
      LOG(ERROR) << "webui could not bind port " << option_.port;
    }
  });
  impl_->server.wait_until_ready();
  LOG(INFO) << "webui on http://localhost:" << option_.port << " serving " << option_.www_dir;
}

WebDebugSink::~WebDebugSink() {
  stopping_ = true;
  frame_ready_.notify_all();
  impl_->server.stop();
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void WebDebugSink::PublishScanMatch(common::Time time,
                                    const std::unordered_map<std::string, Eigen::Affine2d>& poses,
                                    const sensor::PointCloud& point_cloud,
                                    const mapping::GridMapu8& grid_map, double score) {
  const auto now = std::chrono::steady_clock::now();
  const bool copy_map =
      map_seq_ == 0 ||
      std::chrono::duration<double>(now - last_map_copy_).count() >= option_.map_period_seconds;

  std::ostringstream out;
  out.precision(9);
  out << "{\"time\":" << common::ToUnixSeconds(time) << ",\"score\":" << score << ",\"poses\":{";
  bool first = true;
  for (const auto& [name, pose] : poses) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << '"' << name << "\":";
    AppendPose(out, pose);
  }
  // Sensor frame, so the page can draw it under any of the poses above and show what the
  // refinement actually moved.
  out << "},\"scan\":[";
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (i != 0) {
      out << ',';
    }
    out << point_cloud[i].point.x() << ',' << point_cloud[i].point.y();
  }
  out << "]";

  if (copy_map) {
    CopyGrid(grid_map);
    last_map_copy_ = now;
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    out << ",\"map_seq\":" << map_seq_ << ",\"traj_len\":" << trajectory_.size() / 2 << '}';
    latest_frame_ = out.str();
    ++frame_seq_;
    const auto matched = poses.find("matched");
    if (matched != poses.end()) {
      trajectory_.push_back(matched->second.translation().x());
      trajectory_.push_back(matched->second.translation().y());
    }
  }
  frame_ready_.notify_all();
}

void WebDebugSink::CopyGrid(const mapping::GridMapu8& grid_map) {
  std::vector<uint8_t> blob;
  blob.reserve(kMapHeaderSize + grid_map.data().size());
  blob.insert(blob.end(), kMapMagic, kMapMagic + sizeof(kMapMagic));
  AppendLittleEndian(blob, static_cast<int32_t>(grid_map.width()));
  AppendLittleEndian(blob, static_cast<int32_t>(grid_map.height()));
  AppendLittleEndian(blob, grid_map.resolution());
  AppendLittleEndian(blob, grid_map.origin_x());
  AppendLittleEndian(blob, grid_map.origin_y());
  for (const uint8_t value : grid_map.data()) {
    blob.push_back(mapping::IsKnownValue(value) ? static_cast<uint8_t>(std::lround(
                                                      100.0 * mapping::ValueToProbability(value)))
                                                : kUnknownCell);
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  map_blob_ = std::move(blob);
  ++map_seq_;
}

void WebDebugSink::WaitForever() {
  LOG(INFO) << "webui still serving on http://localhost:" << option_.port << ", ctrl-c to stop";
  server_thread_.join();
}

}  // namespace evergreenslam::webui
