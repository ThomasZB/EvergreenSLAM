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
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/local_trajectory_builder.h"
#include "mapping/local_trajectory_builder_option.h"
#include "utils/transform/transform.h"
#ifdef EVERGREENSLAM_WITH_WEBUI
#include "web_debug_sink.h"
#endif

namespace {

using evergreenslam::common::Time;
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
  int webui_port = 0;
  double speed = 0.0;
  bool map_from_reference = false;
  bool has_fixed_extrinsic = false;
  Eigen::Affine2d base_from_laser = Eigen::Affine2d::Identity();
  bool with_backend = true;
  std::string map_dir;
  // Where the run starts in the loaded map's frame; without it a reboot seeds from last_pose.pb.
  std::optional<Eigen::Affine2d> initial_pose;
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
    } else if (key == "--no_backend") {
      args.with_backend = false;
    } else if (key == "--map_dir") {
      args.map_dir = value();
    } else if (key == "--initial_pose") {
      const std::string text = value();
      double x = 0.0;
      double y = 0.0;
      double theta = 0.0;
      if (std::sscanf(text.c_str(), "%lf,%lf,%lf", &x, &y, &theta) != 3) {
        std::cerr << "--initial_pose wants x,y,theta" << std::endl;
        std::exit(2);
      }
      args.initial_pose = evergreenslam::utils::transform::FromXYTheta(x, y, theta);
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
              << "       [--webui_port <n>] [--speed <x>] [--map_from_reference]\n"
              << "       [--no_backend] [--map_dir <dir>] [--initial_pose x,y,theta]" << std::endl;
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

void WritePgm(const evergreenslam::mapping::GridMapu8& snapshot, const std::string& path) {
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

  std::unique_ptr<evergreenslam::lifelong::PoseGraph> backend;
  if (args.with_backend) {
    evergreenslam::lifelong::PoseGraphOption backend_option;
    if (!args.config.empty()) {
      backend_option = evergreenslam::lifelong::LoadPoseGraphOptionFromFile(args.config);
    }
    backend = std::make_unique<evergreenslam::lifelong::PoseGraph>(backend_option, args.map_dir);
  }
  // Start needs the bag's clock, not ours.
  bool backend_started = false;

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

    if (backend != nullptr && !backend_started) {
      backend->Start(timed_scan.time, args.initial_pose);
      backend_started = true;
    }

    const auto matching = builder.AddScan(timed_scan.time, timed_scan.point_cloud);
    ++num_scans;
    const Eigen::Affine2d estimate = builder.local_pose();
    last_estimate = estimate;

    if (matching != nullptr && matching->insertion_result != nullptr) {
      const auto& insertion = *matching->insertion_result;
      ++num_keyframes;
      if (!args.map_from_reference) {
        inserter.Insert(
            estimate.translation(),
            evergreenslam::sensor::TransformPointCloud(insertion.node.point_cloud, estimate),
            global_grid);
      }
      if (backend != nullptr) {
        backend->AddInsertionResult(insertion);
#ifdef EVERGREENSLAM_WITH_WEBUI
        if (web_debug_sink != nullptr && num_keyframes % 30 == 0) {
          web_debug_sink->PublishGlobalMap(backend->AssembleGlobalMap());
          web_debug_sink->PublishPoseGraph(*backend);
        }
#endif
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
    if (args.map_from_reference && matching != nullptr && matching->insertion_result != nullptr) {
      inserter.Insert(reference_relative.translation(),
                      evergreenslam::sensor::TransformPointCloud(
                          matching->insertion_result->node.point_cloud, reference_relative),
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

  WritePgm(global_grid.ToSnapshot(), args.out_prefix + "_map.pgm");

  double backend_rms = -1.0;
  double backend_worst = 0.0;
  int backend_scored = 0;
  if (backend != nullptr && backend_started) {
    backend->Finish();
    WritePgm(backend->AssembleGlobalMap(), args.out_prefix + "_global_map.pgm");
#ifdef EVERGREENSLAM_WITH_WEBUI
    if (web_debug_sink != nullptr) {
      web_debug_sink->PublishGlobalMap(backend->AssembleGlobalMap());
      web_debug_sink->PublishPoseGraph(*backend);
    }
#endif
    std::ofstream constraints_csv(args.out_prefix + "_constraints.csv");
    constraints_csv << "type,from_kind,from_session,from_index,to_kind,to_session,to_index,"
                       "rel_x,rel_y,rel_theta,from_x,from_y,to_x,to_y,err_trans,err_rot\n";
    constraints_csv.precision(12);
    const auto pose_of = [&](const evergreenslam::lifelong::VariableId& id) -> Eigen::Affine2d {
      return id.kind == evergreenslam::lifelong::VariableId::Kind::NODE
                 ? backend->graph().node(id.node_id()).global_pose
                 : backend->graph().submap(id.submap_id()).global_pose;
    };
    for (const auto& constraint : backend->graph().constraints()) {
      const char* type =
          constraint.type == evergreenslam::lifelong::Constraint::Type::INTRA_SUBMAP
              ? "INTRA"
              : (constraint.type == evergreenslam::lifelong::Constraint::Type::PRIOR ? "PRIOR"
                                                                                     : "INTER");
      const Eigen::Affine2d from_pose = pose_of(constraint.from);
      const Eigen::Affine2d to_pose =
          constraint.to.has_value() ? pose_of(*constraint.to) : Eigen::Affine2d::Identity();
      const Eigen::Affine2d predicted = constraint.to.has_value()
                                            ? Eigen::Affine2d(from_pose * constraint.relative_pose)
                                            : constraint.relative_pose;
      const Eigen::Affine2d target = constraint.to.has_value() ? to_pose : from_pose;
      const Eigen::Affine2d error = Eigen::Affine2d(predicted.inverse() * target);
      const auto kind_of = [](const evergreenslam::lifelong::VariableId& id) {
        return id.kind == evergreenslam::lifelong::VariableId::Kind::NODE ? "node" : "submap";
      };
      constraints_csv << type << "," << kind_of(constraint.from) << ","
                      << constraint.from.session_id << "," << constraint.from.index << ",";
      if (constraint.to.has_value()) {
        constraints_csv << kind_of(*constraint.to) << "," << constraint.to->session_id << ","
                        << constraint.to->index << ",";
      } else {
        constraints_csv << ",,,";
      }
      constraints_csv << constraint.relative_pose.translation().x() << ","
                      << constraint.relative_pose.translation().y() << ","
                      << GetYaw(constraint.relative_pose) << "," << from_pose.translation().x()
                      << "," << from_pose.translation().y() << "," << to_pose.translation().x()
                      << "," << to_pose.translation().y() << "," << error.translation().norm()
                      << "," << std::abs(NormalizeAngle(GetYaw(error))) << "\n";
    }

    std::ofstream nodes_csv(args.out_prefix + "_nodes.csv");
    nodes_csv << "stamp_ns,x,y,theta\n";
    nodes_csv.precision(12);

    bool have_first_node = false;
    Eigen::Affine2d first_node_estimate = Eigen::Affine2d::Identity();
    Eigen::Affine2d first_node_reference = Eigen::Affine2d::Identity();
    double sum_squared = 0.0;
    for (const auto& [node_id, node] : backend->graph().nodes()) {
      const Time node_time = node.constant_data.time;
      nodes_csv << evergreenslam::common::ToUnixNanos(node_time) << ","
                << node.global_pose.translation().x() << "," << node.global_pose.translation().y()
                << "," << GetYaw(node.global_pose) << "\n";
      std::optional<Eigen::Affine2d> reference;
      if (!args.reference_frame.empty()) {
        reference = tf_tree.Lookup(args.reference_frame, args.base_frame, node_time);
      } else if (!odometry.empty()) {
        reference = InterpolateAt(odometry, node_time);
      }
      if (!reference.has_value()) {
        continue;
      }
      if (!have_first_node) {
        first_node_estimate = node.global_pose;
        first_node_reference = *reference;
        have_first_node = true;
      }
      const Eigen::Affine2d estimate_relative =
          Eigen::Affine2d(first_node_estimate.inverse() * node.global_pose);
      const Eigen::Affine2d reference_relative =
          Eigen::Affine2d(first_node_reference.inverse() * *reference);
      const double error =
          (estimate_relative.translation() - reference_relative.translation()).norm();
      sum_squared += error * error;
      backend_worst = std::max(backend_worst, error);
      ++backend_scored;
    }
    if (backend_scored > 0) {
      backend_rms = std::sqrt(sum_squared / backend_scored);
    }
  }

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

  if (backend != nullptr && backend_started) {
    std::cout << "\n=== lifelong backend ===" << std::endl;
    std::cout << "nodes " << backend->graph().nodes().size() << "  submaps "
              << backend->graph().submaps().size() << "  constraints "
              << backend->graph().constraints().size() << std::endl;
    std::cout << "loop matches attempted " << backend->constraint_builder().num_matches_attempted()
              << ", accepted " << backend->constraint_builder().num_constraints_added()
              << std::endl;
    std::cout << "sessions frozen " << backend->session_manager().num_sessions_frozen()
              << " (last verdict: " << backend->session_manager().last_verdict()
              << ")  submaps trimmed " << backend->trimmer().num_submaps_trimmed() << std::endl;
    if (backend->map_manager() != nullptr) {
      std::cout << "checkpoints written " << backend->map_manager()->num_checkpoints_written()
                << " to " << backend->map_manager()->directory() << std::endl;
    }
    if (backend_scored > 0) {
      std::cout << "optimized absolute error rms " << backend_rms << " m, worst " << backend_worst
                << " m over " << backend_scored << " nodes (odometry baseline rms above)"
                << std::endl;
    }
  }

#ifdef EVERGREENSLAM_WITH_WEBUI
  if (web_debug_sink != nullptr) {
    web_debug_sink->WaitForever();
  }
#endif
  return status;
}
