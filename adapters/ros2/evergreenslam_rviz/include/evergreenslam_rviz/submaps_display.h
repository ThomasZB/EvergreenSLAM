/**
 * @file submaps_display.h
 * @author hang chen (chen@hang.plus)
 * @brief rviz display of the pose graph's submaps, one texture per submap.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#ifndef EVERGREENSLAM_RVIZ_SUBMAPS_DISPLAY_H_
#define EVERGREENSLAM_RVIZ_SUBMAPS_DISPLAY_H_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "evergreenslam_msgs/msg/submap_list.hpp"
#include "evergreenslam_msgs/srv/submap_query.hpp"
#include "evergreenslam_rviz/drawable_submap.h"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace evergreenslam_rviz {

// Qt does not allow a private nested QObject, so the per-session checkbox group lives here.
struct Session : public QObject {
  Q_OBJECT

 public:
  Session(std::unique_ptr<::rviz_common::properties::BoolProperty> property,
          bool pose_markers_enabled);

  std::unique_ptr<::rviz_common::properties::BoolProperty> visibility;
  std::unique_ptr<::rviz_common::properties::BoolProperty> pose_markers_visibility;
  std::map<int, std::unique_ptr<DrawableSubmap>> submaps;

 private Q_SLOTS:
  void AllEnabledToggled();
  void PoseMarkersEnabledToggled();
};

// Draws the map as the collection of submaps it is: one alpha blended texture each, at the pose
// the backend last optimized it to.
class SubmapsDisplay : public ::rviz_common::Display {
  Q_OBJECT

 public:
  SubmapsDisplay();
  ~SubmapsDisplay() override;

  SubmapsDisplay(const SubmapsDisplay&) = delete;
  SubmapsDisplay& operator=(const SubmapsDisplay&) = delete;

 private Q_SLOTS:
  void Reset();
  void UpdateTopic();
  void AllEnabledToggled();
  void PoseMarkersEnabledToggled();

 private:
  // Called by rviz and therefore not in the repo's naming style.
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  void Subscribe();
  void Unsubscribe();
  void CreateClient();
  void ProcessMessage(const ::evergreenslam_msgs::msg::SubmapList& msg);

  ::rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::Subscription<::evergreenslam_msgs::msg::SubmapList>::SharedPtr subscription_;
  rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedPtr client_;

  // Shared with the subscription callback, which may still be running on the executor thread
  // when this display is already gone; it must not touch the display itself.
  struct Inbox {
    std::mutex mutex;
    ::evergreenslam_msgs::msg::SubmapList::ConstSharedPtr latest;
  };
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();

  Ogre::SceneNode* map_node_ = nullptr;
  std::map<int, std::unique_ptr<Session>> sessions_;

  ::rviz_common::properties::StringProperty* topic_property_;
  ::rviz_common::properties::StringProperty* submap_query_service_property_;
  ::rviz_common::properties::StringProperty* map_frame_property_;
  ::rviz_common::properties::StringProperty* tracking_frame_property_;
  ::rviz_common::properties::FloatProperty* fade_out_start_distance_in_meters_;
  ::rviz_common::properties::Property* sessions_category_;
  ::rviz_common::properties::BoolProperty* visibility_all_enabled_;
  ::rviz_common::properties::BoolProperty* pose_markers_all_enabled_;
};

}  // namespace evergreenslam_rviz

#endif  // EVERGREENSLAM_RVIZ_SUBMAPS_DISPLAY_H_
