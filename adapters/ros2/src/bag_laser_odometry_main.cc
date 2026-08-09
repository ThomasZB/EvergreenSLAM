/**
 * @file bag_laser_odometry_main.cc
 * @author hang chen (chen@hang.plus)
 * @brief Replays a rosbag2 through the local trajectory builder and scores it against a reference
 *        trajectory taken from the bag itself.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <vector>

#include "laser_scan_converter.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/local_trajectory_builder.h"
#include "mapping/local_trajectory_builder_option.h"
#include "utils/transform/transform.h"
#ifdef EVERGREENSLAM_WITH_WEBUI
#include "web_debug_sink.h"
#endif

namespace {

using evergreenslam::common::Time;
// Time is a std::chrono alias, so ADL looks in std and never finds common's stream operator.
using evergreenslam::common::operator<<;
using evergreenslam::utils::transform::GetYaw;
using evergreenslam::utils::transform::NormalizeAngle;

struct Args {
  std::string bag;
  std::string config;
  std::string scan_topic = "/scan";
  std::string odom_topic;
  std::string base_frame = "base_link";
  std::string reference_frame;
  std::string out_prefix = "/tmp/evergreenslam_run";
  int max_scans = 0;
  // 0 disables the viewer entirely, which is the configuration the reported numbers are for.
  int webui_port = 0;
  // Replay pacing as a multiple of real time; 0 is as fast as the machine allows. Only useful
  // with the viewer, so --webui_port defaults it to 1.
  double speed = 0.0;
  // Draws the map at the reference poses instead of the estimated ones. A reference that is
  // itself drifting smears its own map, which is how to tell which of the two is wrong.
  bool map_from_reference = false;
  // Fallback for bags recorded without a transform tree.
  bool has_fixed_extrinsic = false;
  Eigen::Affine2d base_from_laser = Eigen::Affine2d::Identity();
};

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << key << std::endl;
        std::exit(2);
      }
      return argv[++i];
    };
    if (key == "--bag") {
      args.bag = value();
    } else if (key == "--config") {
      args.config = value();
    } else if (key == "--scan_topic") {
      args.scan_topic = value();
    } else if (key == "--odom_topic") {
      args.odom_topic = value();
    } else if (key == "--base_frame") {
      args.base_frame = value();
    } else if (key == "--reference_frame") {
      args.reference_frame = value();
    } else if (key == "--out_prefix") {
      args.out_prefix = value();
    } else if (key == "--max_scans") {
      args.max_scans = std::stoi(value());
    } else if (key == "--webui_port") {
      args.webui_port = std::stoi(value());
      if (args.speed == 0.0) {
        args.speed = 1.0;
      }
    } else if (key == "--speed") {
      args.speed = std::stod(value());
    } else if (key == "--map_from_reference") {
      args.map_from_reference = true;
    } else if (key == "--base_from_laser") {
      const std::string text = value();
      double x = 0.0;
      double y = 0.0;
      double theta = 0.0;
      if (std::sscanf(text.c_str(), "%lf,%lf,%lf", &x, &y, &theta) != 3) {
        std::cerr << "--base_from_laser wants x,y,theta" << std::endl;
        std::exit(2);
      }
      args.base_from_laser = evergreenslam::utils::transform::FromXYTheta(x, y, theta);
      args.has_fixed_extrinsic = true;
    } else {
      std::cerr << "unknown argument " << key << std::endl;
      std::exit(2);
    }
  }
  if (args.bag.empty()) {
    std::cerr << "usage: bag_laser_odometry --bag <dir> [--config <yaml>] [--scan_topic <t>]\n"
              << "       [--base_frame <f>] [--reference_frame <f>] [--odom_topic <t>]\n"
              << "       [--base_from_laser x,y,theta] [--out_prefix <p>] [--max_scans <n>]\n"
              << "       [--webui_port <n>] [--speed <x>] [--map_from_reference]" << std::endl;
    std::exit(2);
  }
  return args;
}

template <typename Message>
Message Deserialize(const rosbag2_storage::SerializedBagMessage& message) {
  Message value;
  const rclcpp::SerializedMessage serialized(*message.serialized_data);
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &value);
  return value;
}

double YawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct StampedPose {
  Time time;
  Eigen::Affine2d pose = Eigen::Affine2d::Identity();
};

Eigen::Affine2d InterpolateAt(const std::vector<StampedPose>& poses, Time time) {
  const auto it = std::lower_bound(poses.begin(), poses.end(), time,
                                   [](const StampedPose& pose, Time t) { return pose.time < t; });
  if (it == poses.begin()) {
    return poses.front().pose;
  }
  if (it == poses.end()) {
    return poses.back().pose;
  }
  const StampedPose& before = *(it - 1);
  const StampedPose& after = *it;
  const double span = evergreenslam::common::ToSeconds(after.time - before.time);
  if (span <= 0.0) {
    return after.pose;
  }
  const double factor = evergreenslam::common::ToSeconds(time - before.time) / span;
  return evergreenslam::utils::transform::Interpolate(before.pose, after.pose, factor);
}

// Enough of tf2 to walk a recorded tree offline. ROS 1 bags carry no /tf_static, so the sensor
// extrinsic and the reference trajectory both come from the same time indexed edges.
class TfTree {
 public:
  static std::string Normalize(const std::string& frame) {
    return frame.empty() || frame[0] != '/' ? frame : frame.substr(1);
  }

  void Add(const std::string& parent, const std::string& child, Time time,
           const Eigen::Affine2d& pose) {
    Edge& edge = edges_[Normalize(child)];
    edge.parent = Normalize(parent);
    edge.samples.push_back({time, pose});
  }

  void Finalize() {
    for (auto& [child, edge] : edges_) {
      std::sort(edge.samples.begin(), edge.samples.end(),
                [](const StampedPose& a, const StampedPose& b) { return a.time < b.time; });
    }
  }

  std::optional<Eigen::Affine2d> Lookup(const std::string& target, const std::string& source,
                                        Time time) const {
    const std::string goal = Normalize(target);
    std::string current = Normalize(source);
    Eigen::Affine2d result = Eigen::Affine2d::Identity();
    for (int hop = 0; hop < 32; ++hop) {
      if (current == goal) {
        return result;
      }
      const auto it = edges_.find(current);
      if (it == edges_.end() || it->second.samples.empty()) {
        return std::nullopt;
      }
      result = InterpolateAt(it->second.samples, time) * result;
      current = it->second.parent;
    }
    return std::nullopt;
  }

  bool empty() const { return edges_.empty(); }

 private:
  struct Edge {
    std::string parent;
    std::vector<StampedPose> samples;
  };
  std::map<std::string, Edge> edges_;
};

void WritePgm(const evergreenslam::mapping::ProbabilityGrid& grid, const std::string& path) {
  const evergreenslam::mapping::GridMapu8 snapshot = grid.ToSnapshot();
  if (snapshot.width() == 0 || snapshot.height() == 0) {
    return;
  }
  std::ofstream out(path, std::ios::binary);
  out << "P5\n" << snapshot.width() << " " << snapshot.height() << "\n255\n";
  // Rows bottom up, so the image is not mirrored against the world frame.
  for (int y = snapshot.height() - 1; y >= 0; --y) {
    for (int x = 0; x < snapshot.width(); ++x) {
      const uint8_t value = snapshot.GetValue(x, y);
      const uint8_t pixel =
          evergreenslam::mapping::IsKnownValue(value)
              ? static_cast<uint8_t>(255.0 *
                                     (1.0 - evergreenslam::mapping::ValueToProbability(value)))
              : 128;
      out.put(static_cast<char>(pixel));
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = ParseArgs(argc, argv);

  evergreenslam::mapping::LocalTrajectoryBuilderOption option;
  if (!args.config.empty()) {
    option = evergreenslam::mapping::LoadLocalTrajectoryBuilderOptionFromFile(args.config);
  }
  std::cout << option << std::endl;

  // Pass one: the transform tree and any odometry, both small next to the scans.
  TfTree tf_tree;
  std::vector<StampedPose> odometry;
  {
    rosbag2_cpp::Reader reader;
    reader.open(args.bag);
    while (reader.has_next()) {
      const auto message = reader.read_next();
      if (message->topic_name == "/tf" || message->topic_name == "/tf_static") {
        const auto tf = Deserialize<tf2_msgs::msg::TFMessage>(*message);
        for (const auto& transform : tf.transforms) {
          tf_tree.Add(transform.header.frame_id, transform.child_frame_id,
                      evergreenslam::ros2::FromRosTime(transform.header.stamp),
                      evergreenslam::utils::transform::FromXYTheta(
                          transform.transform.translation.x, transform.transform.translation.y,
                          YawOf(transform.transform.rotation)));
        }
      } else if (!args.odom_topic.empty() && message->topic_name == args.odom_topic) {
        const auto odom = Deserialize<nav_msgs::msg::Odometry>(*message);
        odometry.push_back({evergreenslam::ros2::FromRosTime(odom.header.stamp),
                            evergreenslam::utils::transform::FromXYTheta(
                                odom.pose.pose.position.x, odom.pose.pose.position.y,
                                YawOf(odom.pose.pose.orientation))});
      }
    }
  }
  tf_tree.Finalize();
  std::sort(odometry.begin(), odometry.end(),
            [](const StampedPose& a, const StampedPose& b) { return a.time < b.time; });
  std::cout << "tf tree: " << (tf_tree.empty() ? "empty" : "loaded")
            << ", odometry poses: " << odometry.size() << std::endl;

  evergreenslam::mapping::LocalTrajectoryBuilder builder(option);
  builder.StartNewSession(0);

#ifdef EVERGREENSLAM_WITH_WEBUI
  std::shared_ptr<evergreenslam::webui::WebDebugSink> web_debug_sink;
  if (args.webui_port > 0) {
    evergreenslam::webui::WebDebugSinkOption webui_option;
    webui_option.port = args.webui_port;
    web_debug_sink = std::make_shared<evergreenslam::webui::WebDebugSink>(webui_option);
    builder.SetDebugSink(web_debug_sink);
  }
#endif

  evergreenslam::mapping::ProbabilityGrid global_grid(option.active_map_option.resolution);
  const evergreenslam::mapping::CastRaysMapping inserter(option.active_map_option.inserter_option);

  std::ofstream trajectory(args.out_prefix + "_trajectory.csv");
  trajectory << "stamp_ns,est_x,est_y,est_theta,ref_x,ref_y,ref_theta\n";
  trajectory.precision(12);

  std::chrono::steady_clock::time_point replay_start_wall;
  Time replay_start_bag;

  int num_scans = 0;
  int num_keyframes = 0;
  int num_scored = 0;
  int num_missing_extrinsic = 0;
  bool have_first = false;
  Eigen::Affine2d first_estimate = Eigen::Affine2d::Identity();
  Eigen::Affine2d first_reference = Eigen::Affine2d::Identity();
  Eigen::Affine2d previous_estimate = Eigen::Affine2d::Identity();
  Eigen::Affine2d previous_reference = Eigen::Affine2d::Identity();
  Eigen::Affine2d last_estimate = Eigen::Affine2d::Identity();
  double sum_squared_absolute = 0.0;
  double sum_squared_relative = 0.0;
  double worst_absolute = 0.0;
  double reference_path_length = 0.0;
  double estimate_path_length = 0.0;
  double final_absolute = 0.0;
  double final_yaw_error = 0.0;
  bool logged_extrinsic = false;

  rosbag2_cpp::Reader reader;
  reader.open(args.bag);
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name != args.scan_topic) {
      continue;
    }
    const auto scan = Deserialize<sensor_msgs::msg::LaserScan>(*message);
    const Time time = evergreenslam::ros2::FromRosTime(scan.header.stamp);

    Eigen::Affine2d base_from_laser = args.base_from_laser;
    if (!args.has_fixed_extrinsic &&
        TfTree::Normalize(scan.header.frame_id) != TfTree::Normalize(args.base_frame)) {
      const std::optional<Eigen::Affine2d> looked_up =
          tf_tree.Lookup(args.base_frame, scan.header.frame_id, time);
      if (!looked_up.has_value()) {
        ++num_missing_extrinsic;
        continue;
      }
      base_from_laser = *looked_up;
    }
    if (!logged_extrinsic) {
      std::cout << "extrinsic " << args.base_frame << " <- " << scan.header.frame_id << " = ["
                << base_from_laser.translation().x() << ", " << base_from_laser.translation().y()
                << ", " << GetYaw(base_from_laser) << "]" << std::endl;
      logged_extrinsic = true;
    }

    const evergreenslam::ros2::TimedScan timed_scan =
        evergreenslam::ros2::FromLaserScan(scan, base_from_laser);
    if (num_scans < 3) {
      std::cout << "scan " << num_scans << " stamp " << time << " beams " << scan.ranges.size()
                << " valid " << timed_scan.point_cloud.size() << std::endl;
    }
    if (timed_scan.point_cloud.empty()) {
      continue;
    }

    if (args.speed > 0.0) {
      if (num_scans == 0) {
        replay_start_wall = std::chrono::steady_clock::now();
        replay_start_bag = time;
      }
      const double elapsed = evergreenslam::common::ToSeconds(time - replay_start_bag) / args.speed;
      std::this_thread::sleep_until(replay_start_wall + std::chrono::duration<double>(elapsed));
    }

    const auto insertion = builder.AddScan(timed_scan.time, timed_scan.point_cloud);
    ++num_scans;
    const Eigen::Affine2d estimate = builder.local_pose();
    last_estimate = estimate;

    if (insertion.has_value()) {
      ++num_keyframes;
      if (!args.map_from_reference) {
        inserter.Insert(
            estimate.translation(),
            evergreenslam::sensor::TransformPointCloud(insertion->node.point_cloud, estimate),
            global_grid);
      }
    }

    std::optional<Eigen::Affine2d> reference;
    if (!args.reference_frame.empty()) {
      reference = tf_tree.Lookup(args.reference_frame, args.base_frame, time);
    } else if (!odometry.empty()) {
      reference = InterpolateAt(odometry, time);
    }
    if (!reference.has_value()) {
      continue;
    }
    const Eigen::Affine2d reference_pose = *reference;

    if (!have_first) {
      first_estimate = estimate;
      first_reference = reference_pose;
      previous_estimate = estimate;
      previous_reference = reference_pose;
      have_first = true;
    }

    const Eigen::Affine2d estimate_relative = first_estimate.inverse() * estimate;
    const Eigen::Affine2d reference_relative = first_reference.inverse() * reference_pose;
    if (args.map_from_reference && insertion.has_value()) {
      inserter.Insert(reference_relative.translation(),
                      evergreenslam::sensor::TransformPointCloud(insertion->node.point_cloud,
                                                                 reference_relative),
                      global_grid);
    }
    const double absolute =
        (estimate_relative.translation() - reference_relative.translation()).norm();
    sum_squared_absolute += absolute * absolute;
    worst_absolute = std::max(worst_absolute, absolute);
    final_absolute = absolute;
    final_yaw_error = NormalizeAngle(GetYaw(estimate_relative) - GetYaw(reference_relative));

    const Eigen::Affine2d estimate_step = previous_estimate.inverse() * estimate;
    const Eigen::Affine2d reference_step = previous_reference.inverse() * reference_pose;
    const double relative = (estimate_step.translation() - reference_step.translation()).norm();
    sum_squared_relative += relative * relative;
    reference_path_length += reference_step.translation().norm();
    estimate_path_length += estimate_step.translation().norm();
    ++num_scored;

    previous_estimate = estimate;
    previous_reference = reference_pose;

    trajectory << evergreenslam::common::ToUnixNanos(time) << ","
               << estimate_relative.translation().x() << "," << estimate_relative.translation().y()
               << "," << GetYaw(estimate_relative) << "," << reference_relative.translation().x()
               << "," << reference_relative.translation().y() << "," << GetYaw(reference_relative)
               << "\n";

    if (args.max_scans > 0 && num_scans >= args.max_scans) {
      break;
    }
  }

  WritePgm(global_grid, args.out_prefix + "_map.pgm");

  const int status = [&]() -> int {
    std::cout << "\n=== result ===" << std::endl;
    std::cout << "scans: " << num_scans << "  keyframes: " << num_keyframes
              << "  skipped for missing extrinsic: " << num_missing_extrinsic << std::endl;
    if (num_scans == 0) {
      std::cerr << "no usable scan on topic " << args.scan_topic << std::endl;
      return 1;
    }
    if (num_scored == 0) {
      std::cout << "no reference trajectory, distance from start to end: "
                << (first_estimate.inverse() * last_estimate).translation().norm() << " m"
                << std::endl;
      return 0;
    }
    std::cout << "scored scans: " << num_scored << std::endl;
    std::cout << "path length estimated " << estimate_path_length << " m, reference "
              << reference_path_length << " m" << std::endl;
    std::cout << "absolute error rms " << std::sqrt(sum_squared_absolute / num_scored)
              << " m, worst " << worst_absolute << " m, final " << final_absolute
              << " m, final yaw " << final_yaw_error << " rad" << std::endl;
    std::cout << "per scan relative error rms " << std::sqrt(sum_squared_relative / num_scored)
              << " m" << std::endl;
    std::cout << "final drift " << 100.0 * final_absolute / std::max(reference_path_length, 1e-9)
              << " % of reference path" << std::endl;
    return 0;
  }();

#ifdef EVERGREENSLAM_WITH_WEBUI
  if (web_debug_sink != nullptr) {
    web_debug_sink->WaitForever();
  }
#endif
  return status;
}
