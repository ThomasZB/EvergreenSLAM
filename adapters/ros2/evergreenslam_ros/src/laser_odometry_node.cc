/**
 * @file laser_odometry_node.cc
 * @author hang chen (chen@hang.plus)
 * @brief Live ROS 2 node: LaserScan in, odometry, tf and an occupancy grid out.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

#include "laser_scan_converter.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "mapping/local_trajectory_builder.h"
#include "mapping/local_trajectory_builder_option.h"
#include "pose_graph_publisher.h"
#include "utils/transform/transform.h"
#ifdef EVERGREENSLAM_WITH_WEBUI
#include "web_debug_sink.h"
#endif

namespace evergreenslam::ros2 {
namespace {

geometry_msgs::msg::Quaternion YawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  return q;
}

double QuaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class LaserOdometryNode : public rclcpp::Node {
 public:
  LaserOdometryNode() : Node("laser_odometry") {
    const std::string config = declare_parameter<std::string>("config", "");
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "scan");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    map_publish_period_ = declare_parameter<double>("map_publish_period", 1.0);

    mapping::LocalTrajectoryBuilderOption option;
    if (!config.empty()) {
      option = mapping::LoadLocalTrajectoryBuilderOptionFromFile(config);
    }
    builder_ = std::make_unique<mapping::LocalTrajectoryBuilder>(option);

    const bool lifelong = declare_parameter<bool>("lifelong", true);
    const std::string map_dir = declare_parameter<std::string>("map_dir", "");
    if (lifelong) {
      lifelong::PoseGraphOption backend_option;
      if (!config.empty()) {
        backend_option = lifelong::LoadPoseGraphOptionFromFile(config);
      }
      backend_ = std::make_unique<lifelong::PoseGraph>(backend_option, map_dir);
      map_frame_ = declare_parameter<std::string>("map_frame", "map");
    }

#ifdef EVERGREENSLAM_WITH_WEBUI
    const int webui_port = declare_parameter<int>("webui_port", 0);
    if (webui_port > 0) {
      webui::WebDebugSinkOption webui_option;
      webui_option.port = webui_port;
      web_debug_sink_ = std::make_shared<webui::WebDebugSink>(webui_option);
      builder_->SetDebugSink(web_debug_sink_);
    }
#endif

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (backend_ == nullptr) {
      // With the backend on, PoseGraphPublisher owns "map"; two publishers would fight over it.
      map_publisher_ =
          create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
    }
    // A RELIABLE subscriber gets nothing from a BEST_EFFORT publisher; SensorDataQoS takes both.
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { HandleScan(*scan); });
    if (backend_ != nullptr) {
      initial_pose_subscription_ =
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "initialpose", 10,
              [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
                HandleInitialPose(*message);
              });

      PoseGraphPublisher::GlobalMapHook hook;
#ifdef EVERGREENSLAM_WITH_WEBUI
      if (web_debug_sink_ != nullptr) {
        hook = [this](const mapping::GridMapu8& grid) { web_debug_sink_->PublishGlobalMap(grid); };
      }
#endif
      pose_graph_publisher_ =
          std::make_unique<PoseGraphPublisher>(*this, *backend_, map_frame_, std::move(hook));
      graph_timer_ = create_wall_timer(std::chrono::duration<double>(map_publish_period_), [this] {
        // Start() writes the graph on this thread; a task enqueued before it would race it.
        if (!backend_started_) {
          return;
        }
        pose_graph_publisher_->Publish();
#ifdef EVERGREENSLAM_WITH_WEBUI
        // Must run on the executor thread, not a backend task; it enqueues its own.
        if (web_debug_sink_ != nullptr) {
          web_debug_sink_->PublishPoseGraph(*backend_);
        }
#endif
      });
    }
  }

 private:
  void HandleScan(const sensor_msgs::msg::LaserScan& scan) {
    if (!ResolveExtrinsic(scan.header.frame_id)) {
      return;
    }
    const TimedScan timed_scan = FromLaserScan(scan, base_from_laser_);
    if (timed_scan.point_cloud.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "scan has no finite range");
      return;
    }

    if (backend_ != nullptr && !backend_started_) {
      // Boot on the sensor clock, not ours.
      backend_->Start(timed_scan.time);
      backend_started_ = true;
    }

    const auto matching = builder_->AddScan(timed_scan.time, timed_scan.point_cloud);
    if (matching != nullptr && matching->insertion_result != nullptr && backend_ != nullptr) {
      backend_->AddInsertionResult(*matching->insertion_result);
    }
    Publish(ToRosTime(timed_scan.time), builder_->local_pose());
  }

  void HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& message) {
    if (!backend_started_) {
      RCLCPP_WARN(get_logger(), "initial pose ignored: no scan has booted the backend yet");
      return;
    }
    const auto& pose = message.pose.pose;
    backend_->SetInitialPose(utils::transform::FromXYTheta(pose.position.x, pose.position.y,
                                                           QuaternionToYaw(pose.orientation)));
  }

  bool ResolveExtrinsic(const std::string& laser_frame) {
    if (extrinsic_resolved_) {
      return true;
    }
    if (laser_frame == base_frame_) {
      extrinsic_resolved_ = true;
      return true;
    }
    try {
      const auto transform =
          tf_buffer_->lookupTransform(base_frame_, laser_frame, tf2::TimePointZero);
      base_from_laser_ = utils::transform::FromXYTheta(
          transform.transform.translation.x, transform.transform.translation.y,
          QuaternionToYaw(transform.transform.rotation));
      extrinsic_resolved_ = true;
      return true;
    } catch (const tf2::TransformException& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s -> %s: %s",
                           base_frame_.c_str(), laser_frame.c_str(), exception.what());
      return false;
    }
  }

  void Publish(const builtin_interfaces::msg::Time& stamp, const Eigen::Affine2d& pose) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.orientation = YawToQuaternion(utils::transform::GetYaw(pose));
    odom_publisher_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.translation().x();
      transform.transform.translation.y = pose.translation().y();
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);

      if (backend_ != nullptr) {
        // map -> odom carries the optimizer's correction, so odometry never jumps.
        // ActiveSessionToGlobal, not a graph read: threaded, the graph belongs to the consumer.
        const std::optional<Eigen::Affine2d> session_to_global = backend_->ActiveSessionToGlobal();
        const Eigen::Affine2d map_from_odom =
            session_to_global.has_value() ? *session_to_global : Eigen::Affine2d::Identity();
        geometry_msgs::msg::TransformStamped map_transform;
        map_transform.header.stamp = stamp;
        map_transform.header.frame_id = map_frame_;
        map_transform.child_frame_id = odom_frame_;
        map_transform.transform.translation.x = map_from_odom.translation().x();
        map_transform.transform.translation.y = map_from_odom.translation().y();
        map_transform.transform.rotation = YawToQuaternion(utils::transform::GetYaw(map_from_odom));
        tf_broadcaster_->sendTransform(map_transform);
      }
    }

    // With the backend on, the graph timer owns "map" and the pose graph topics.
    if (backend_ != nullptr) {
      return;
    }
    const rclcpp::Time now(stamp);
    if (last_map_publish_.nanoseconds() != 0 &&
        (now - last_map_publish_).seconds() < map_publish_period_) {
      return;
    }
    last_map_publish_ = now;
    PublishMap(stamp);
  }

  void PublishMap(const builtin_interfaces::msg::Time& stamp) {
    // Not matching_submap(): that one stops growing halfway through its life.
    const auto& submaps = builder_->active_map().submaps();
    if (submaps.empty()) {
      return;
    }
    const auto& submap = submaps.back();
    const mapping::GridMapu8& grid = submap->Snapshot();
    if (grid.width() == 0 || grid.height() == 0) {
      return;
    }
    PublishGrid(stamp, grid, odom_frame_, submap->local_pose());
  }

  // `grid_pose`: the grid's frame expressed in `frame`.
  void PublishGrid(const builtin_interfaces::msg::Time& stamp, const mapping::GridMapu8& grid,
                   const std::string& frame,
                   const Eigen::Affine2d& grid_pose = Eigen::Affine2d::Identity()) {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = frame;
    message.info.resolution = grid.resolution();
    message.info.width = grid.width();
    message.info.height = grid.height();
    const Eigen::Vector2d origin = grid_pose * Eigen::Vector2d(grid.origin_x(), grid.origin_y());
    message.info.origin.position.x = origin.x();
    message.info.origin.position.y = origin.y();
    message.info.origin.orientation = YawToQuaternion(utils::transform::GetYaw(grid_pose));
    message.data.resize(static_cast<size_t>(grid.width()) * grid.height());
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const uint8_t value = grid.GetValue(x, y);
        message.data[static_cast<size_t>(y) * grid.width() + x] =
            mapping::IsKnownValue(value)
                ? static_cast<int8_t>(std::lround(100.0 * mapping::ValueToProbability(value)))
                : static_cast<int8_t>(-1);
      }
    }
    map_publisher_->publish(message);
  }

#ifdef EVERGREENSLAM_WITH_WEBUI
  // Declared first so it is destroyed last: the builder holds a raw pointer to it.
  std::shared_ptr<webui::WebDebugSink> web_debug_sink_;
#endif
  std::unique_ptr<mapping::LocalTrajectoryBuilder> builder_;
  std::unique_ptr<lifelong::PoseGraph> backend_;
  bool backend_started_ = false;
  std::string map_frame_ = "map";
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  double map_publish_period_ = 1.0;

  Eigen::Affine2d base_from_laser_ = Eigen::Affine2d::Identity();
  bool extrinsic_resolved_ = false;
  rclcpp::Time last_map_publish_{0, 0, RCL_ROS_TIME};

  // After backend_: it holds a reference to it and drains the queue on destruction. The timer
  // comes last so it stops firing before the publisher goes.
  std::unique_ptr<PoseGraphPublisher> pose_graph_publisher_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};

}  // namespace evergreenslam::ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<evergreenslam::ros2::LaserOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
