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
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <type_traits>
#include <utility>

#include "httplib.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/probability_values.h"
#include "mapping/submap.h"
#include "pose_graph_view.h"
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

std::string PoseJson(const Eigen::Affine2d& pose) {
  std::ostringstream out;
  out.precision(9);
  AppendPose(out, pose);
  return out.str();
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

  server.Get("/global_map", [this](const httplib::Request&, httplib::Response& response) {
    std::vector<uint8_t> blob;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      blob = global_map_blob_;
    }
    if (blob.empty()) {
      response.status = 204;
      return;
    }
    response.set_content(reinterpret_cast<const char*>(blob.data()), blob.size(),
                         "application/octet-stream");
  });

  server.Get("/graph", [this](const httplib::Request&, httplib::Response& response) {
    std::string json;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      json = graph_json_;
    }
    if (json.empty()) {
      response.status = 204;
      return;
    }
    response.set_content(json, "application/json");
  });

  server.Get("/submap", [this](const httplib::Request& request, httplib::Response& response) {
    // 404, not 204: the page retries a 204, and a submap that is gone is gone.
    std::shared_ptr<const std::vector<uint8_t>> blob;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      const auto found = submap_blobs_.find({std::atoi(request.get_param_value("s").c_str()),
                                             std::atoi(request.get_param_value("i").c_str())});
      if (found != submap_blobs_.end()) {
        blob = found->second;
      }
    }
    if (blob == nullptr) {
      response.status = 404;
      return;
    }
    response.set_content(reinterpret_cast<const char*>(blob->data()), blob->size(),
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
            // Times out, or the handler never notices a closed socket or a shutting down server.
            frame_ready_.wait_for(lock, std::chrono::milliseconds(500),
                                  [this, sent] { return stopping_ || stream_seq_ > sent; });
            if (stopping_) {
              return false;
            }
            // Stamped here, not baked into the frame, so a map published after the last
            // scan still reaches a streaming page.
            if (stream_seq_ > sent && !latest_frame_.empty()) {
              sent = stream_seq_;
              frame = latest_frame_ + ",\"map_seq\":" + std::to_string(map_seq_) +
                      ",\"map_pose\":" + PoseJson(map_pose_) +
                      ",\"global_map_seq\":" + std::to_string(global_map_seq_) +
                      ",\"graph_seq\":" + std::to_string(graph_seq_) +
                      ",\"traj_len\":" + std::to_string(trajectory_.size() / 2) + "}";
            }
          }
          // Outside the lock, always: a blocking write to a stalled browser would park the
          // SLAM thread behind it.
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
  const auto grid_entry = poses.find("grid");
  const Eigen::Affine2d grid_pose =
      grid_entry != poses.end() ? grid_entry->second : Eigen::Affine2d::Identity();
  // A matching submap handoff moves the grid frame; the raster must follow in the same frame.
  const bool grid_moved = !(grid_pose.matrix().array() == map_pose_.matrix().array()).all();
  const bool copy_map =
      map_seq_ == 0 || grid_moved ||
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
  // Sensor frame, so the page can draw it under any of the poses above.
  out << "},\"scan\":[";
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (i != 0) {
      out << ',';
    }
    out << point_cloud[i].point.x() << ',' << point_cloud[i].point.y();
  }
  out << "]";

  if (copy_map) {
    CopyGrid(grid_map, grid_pose);
    last_map_copy_ = now;
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = out.str();
    ++stream_seq_;
    const auto matched = poses.find("matched");
    if (matched != poses.end()) {
      trajectory_.push_back(matched->second.translation().x());
      trajectory_.push_back(matched->second.translation().y());
    }
  }
  frame_ready_.notify_all();
}

std::vector<uint8_t> WebDebugSink::EncodeGrid(const mapping::GridMapu8& grid_map) {
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
  return blob;
}

void WebDebugSink::CopyGrid(const mapping::GridMapu8& grid_map, const Eigen::Affine2d& grid_pose) {
  std::vector<uint8_t> blob = EncodeGrid(grid_map);
  const std::lock_guard<std::mutex> lock(mutex_);
  map_blob_ = std::move(blob);
  map_pose_ = grid_pose;
  ++map_seq_;
}

void WebDebugSink::PublishGlobalMap(const mapping::GridMapu8& grid_map) {
  std::vector<uint8_t> blob = EncodeGrid(grid_map);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    global_map_blob_ = std::move(blob);
    ++global_map_seq_;
    ++stream_seq_;
  }
  frame_ready_.notify_all();
}

void WebDebugSink::PublishPoseGraph(lifelong::PoseGraph& pose_graph) {
  if (graph_request_pending_) {
    return;
  }
  graph_request_pending_ = true;

  // All work runs on the backend task thread; the calling (scan) thread returns at once.
  pose_graph.Enqueue([this, &pose_graph] {
    std::string json = SerializePoseGraph(pose_graph);

    // Collect finished submaps (the shared_ptr keeps them readable even if the trimmer drops them).
    std::vector<std::pair<SubmapKey, std::shared_ptr<const mapping::Submap>>> finished;
    if (option_.serve_submaps) {
      for (const auto& [id, record] : pose_graph.graph().submaps()) {
        if (record.submap->finished()) {
          finished.emplace_back(SubmapKey{id.session_id, id.submap_index}, record.submap);
        }
      }
    }

    // Check which blobs are already cached (brief lock).
    std::map<SubmapKey, std::shared_ptr<const std::vector<uint8_t>>> blobs;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& [key, submap] : finished) {
        const auto found = submap_blobs_.find(key);
        if (found != submap_blobs_.end()) {
          blobs.emplace(key, found->second);
        }
      }
    }

    // Encode new submap blobs (no lock held).
    for (const auto& [key, submap] : finished) {
      if (blobs.count(key) != 0) {
        continue;
      }
      const mapping::GridMapu8& snapshot = submap->Snapshot();
      if (snapshot.width() == 0 || snapshot.height() == 0) {
        continue;
      }
      blobs.emplace(key, std::make_shared<const std::vector<uint8_t>>(EncodeGrid(snapshot)));
    }

    // Publish.
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      graph_json_ = std::move(json);
      submap_blobs_ = std::move(blobs);
      ++graph_seq_;
      ++stream_seq_;
    }
    frame_ready_.notify_all();
    graph_request_pending_ = false;
  });
}

void WebDebugSink::WaitForever() {
  LOG(INFO) << "webui still serving on http://localhost:" << option_.port << ", ctrl-c to stop";
  server_thread_.join();
}

}  // namespace evergreenslam::webui
