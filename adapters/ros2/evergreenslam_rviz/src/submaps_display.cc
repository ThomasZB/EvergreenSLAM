/**
 * @file submaps_display.cc
 * @author hang chen (chen@hang.plus)
 * @brief rviz display of the pose graph's submaps, one texture per submap.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#include "evergreenslam_rviz/submaps_display.h"

#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <exception>
#include <set>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"

namespace evergreenslam_rviz {

namespace {

constexpr int kMaxOnGoingRequestsPerSession = 6;
constexpr char kResourceGroup[] = "evergreenslam_rviz";
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kDefaultTopic[] = "submap_list";
constexpr char kDefaultSubmapQueryServiceName[] = "submap_query";
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultTrackingFrame[] = "base_link";
constexpr float kSubmapPoseAxesLength = 0.3f;
constexpr float kSubmapPoseAxesRadius = 0.06f;

// The material and the shaders are parsed once per process, however many displays are added.
void RegisterOgreMedia() {
  static std::once_flag once;
  std::call_once(once, [] {
    const std::string materials =
        ament_index_cpp::get_package_share_directory("evergreenslam_rviz") + kMaterialsDirectory;
    auto& resource_group_manager = Ogre::ResourceGroupManager::getSingleton();
    resource_group_manager.addResourceLocation(materials, "FileSystem", kResourceGroup);
    resource_group_manager.addResourceLocation(materials + "/glsl120", "FileSystem",
                                               kResourceGroup);
    resource_group_manager.addResourceLocation(materials + "/scripts", "FileSystem",
                                               kResourceGroup);
    resource_group_manager.initialiseResourceGroup(kResourceGroup);
  });
}

}  // namespace

Session::Session(std::unique_ptr<::rviz_common::properties::BoolProperty> property,
                 const bool pose_markers_enabled)
    : visibility(std::move(property)) {
  ::QObject::connect(visibility.get(), SIGNAL(changed()), this, SLOT(AllEnabledToggled()));
  pose_markers_visibility = std::make_unique<::rviz_common::properties::BoolProperty>(
      QString("Submap Pose Markers"), pose_markers_enabled,
      QString("Toggles the submap pose markers of this session."), visibility.get());
  ::QObject::connect(pose_markers_visibility.get(), SIGNAL(changed()), this,
                     SLOT(PoseMarkersEnabledToggled()));
}

void Session::AllEnabledToggled() {
  const bool visible = visibility->getBool();
  for (auto& submap : submaps) {
    submap.second->set_visibility(visible);
  }
}

void Session::PoseMarkersEnabledToggled() {
  const bool visible = pose_markers_visibility->getBool();
  for (auto& submap : submaps) {
    submap.second->set_pose_markers_visibility(visible);
  }
}

SubmapsDisplay::SubmapsDisplay() {
  topic_property_ = new ::rviz_common::properties::StringProperty(
      "Topic", kDefaultTopic, "Latched submap list to subscribe to.", this, SLOT(UpdateTopic()));
  submap_query_service_property_ = new ::rviz_common::properties::StringProperty(
      "Submap query service", kDefaultSubmapQueryServiceName, "Submap query service to connect to.",
      this, SLOT(Reset()));
  map_frame_property_ = new ::rviz_common::properties::StringProperty(
      "Map frame", kDefaultMapFrame, "Frame the submap poses are expressed in.", this);
  tracking_frame_property_ = new ::rviz_common::properties::StringProperty(
      "Tracking frame", kDefaultTrackingFrame, "Tracking frame, used for fading out submaps.",
      this);
  sessions_category_ = new ::rviz_common::properties::Property(
      "Submaps", QVariant(), "List of all submaps, organized by session.", this);
  visibility_all_enabled_ = new ::rviz_common::properties::BoolProperty(
      "All", true, "Whether submaps from all sessions should be displayed or not.",
      sessions_category_, SLOT(AllEnabledToggled()), this);
  pose_markers_all_enabled_ = new ::rviz_common::properties::BoolProperty(
      "All Submap Pose Markers", true, "Whether submap pose markers should be displayed or not.",
      sessions_category_, SLOT(PoseMarkersEnabledToggled()), this);
  fade_out_start_distance_in_meters_ = new ::rviz_common::properties::FloatProperty(
      "Fade-out distance", 1.f,
      "Distance in meters in z-direction beyond which submaps will start to fade out.", this);
  RegisterOgreMedia();
}

SubmapsDisplay::~SubmapsDisplay() {
  Unsubscribe();
  client_.reset();
  sessions_.clear();
  if (map_node_ != nullptr) {
    scene_manager_->destroySceneNode(map_node_);
  }
}

void SubmapsDisplay::onInitialize() {
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  map_node_ = scene_node_->createChildSceneNode();
  CreateClient();
  Subscribe();
}

void SubmapsDisplay::onEnable() {
  Subscribe();
  // Display::onEnableChanged cascades setVisible(true) through every scene node under ours.
  for (const auto& session : sessions_) {
    for (const auto& submap : session.second->submaps) {
      submap.second->ReapplyVisibility();
    }
  }
}

void SubmapsDisplay::onDisable() { Unsubscribe(); }

void SubmapsDisplay::reset() {
  Display::reset();
  Unsubscribe();
  {
    const std::lock_guard<std::mutex> lock(inbox_->mutex);
    inbox_->latest.reset();
  }
  sessions_.clear();
  client_.reset();
  CreateClient();
  Subscribe();
}

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::UpdateTopic() {
  Unsubscribe();
  Subscribe();
}

void SubmapsDisplay::CreateClient() {
  const auto node = rviz_ros_node_.lock();
  if (node == nullptr) {
    return;
  }
  // An edited property can be any string at all, and an invalid name throws out of the Qt slot
  // that changed it, which takes rviz down with it.
  try {
    client_ = node->get_raw_node()->create_client<::evergreenslam_msgs::srv::SubmapQuery>(
        submap_query_service_property_->getStdString());
    setStatus(::rviz_common::properties::StatusProperty::Ok, "Service", "OK");
  } catch (const rclcpp::exceptions::InvalidServiceNameError& e) {
    setStatus(::rviz_common::properties::StatusProperty::Error, "Service", e.what());
  }
}

void SubmapsDisplay::Subscribe() {
  const auto node = rviz_ros_node_.lock();
  if (node == nullptr || !isEnabled() || topic_property_->getStdString().empty()) {
    return;
  }
  try {
    // The publisher latches the list, so a display added long after the node started still gets
    // the current state - which only a transient local subscription receives.
    subscription_ =
        node->get_raw_node()->create_subscription<::evergreenslam_msgs::msg::SubmapList>(
            topic_property_->getStdString(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [inbox = inbox_](::evergreenslam_msgs::msg::SubmapList::ConstSharedPtr msg) {
              const std::lock_guard<std::mutex> lock(inbox->mutex);
              inbox->latest = std::move(msg);
            });
    setStatus(::rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    setStatus(::rviz_common::properties::StatusProperty::Error, "Topic", e.what());
  }
}

void SubmapsDisplay::Unsubscribe() { subscription_.reset(); }

void SubmapsDisplay::ProcessMessage(const ::evergreenslam_msgs::msg::SubmapList& msg) {
  // Versions only ever grow, so a version that went backwards means the node was restarted and
  // everything drawn so far belongs to a map that no longer exists.
  for (const auto& entry : msg.submap) {
    const auto session = sessions_.find(entry.session_id);
    if (session == sessions_.end()) {
      continue;
    }
    const auto submap = session->second->submaps.find(entry.submap_index);
    if (submap != session->second->submaps.end() &&
        submap->second->version() > entry.submap_version) {
      sessions_.clear();
      break;
    }
  }

  std::set<std::pair<int, int>> listed_submaps;
  std::set<int> listed_sessions;
  for (const auto& entry : msg.submap) {
    listed_submaps.insert({entry.session_id, entry.submap_index});
    listed_sessions.insert(entry.session_id);
    if (sessions_.count(entry.session_id) == 0) {
      sessions_.emplace(
          entry.session_id,
          std::make_unique<Session>(
              std::make_unique<::rviz_common::properties::BoolProperty>(
                  QString("Session %1").arg(entry.session_id), visibility_all_enabled_->getBool(),
                  QString("List of all submaps in Session %1. The checkbox controls whether all "
                          "submaps in this session should be displayed or not.")
                      .arg(entry.session_id),
                  sessions_category_),
              pose_markers_all_enabled_->getBool()));
    }
    Session& session = *sessions_[entry.session_id];
    if (session.submaps.count(entry.submap_index) == 0) {
      try {
        session.submaps.emplace(
            entry.submap_index,
            std::make_unique<DrawableSubmap>(
                entry.session_id, entry.submap_index, context_, map_node_, session.visibility.get(),
                session.visibility->getBool(), session.pose_markers_visibility->getBool(),
                kSubmapPoseAxesLength, kSubmapPoseAxesRadius));
      } catch (const std::exception& e) {
        // An exception out of update() would take rviz down; a broken install is a status line.
        setStatus(::rviz_common::properties::StatusProperty::Error, "Submaps", e.what());
        continue;
      }
    }
    session.submaps.at(entry.submap_index)->Update(entry);
  }

  for (auto it = sessions_.begin(); it != sessions_.end();) {
    if (listed_sessions.count(it->first) == 0) {
      it = sessions_.erase(it);
    } else {
      ++it;
    }
  }
  for (const auto& session : sessions_) {
    auto& submaps = session.second->submaps;
    for (auto it = submaps.begin(); it != submaps.end();) {
      if (listed_submaps.count({session.first, it->first}) == 0) {
        it = submaps.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void SubmapsDisplay::update(const float /*wall_dt*/, const float /*ros_dt*/) {
  ::evergreenslam_msgs::msg::SubmapList::ConstSharedPtr msg;
  {
    const std::lock_guard<std::mutex> lock(inbox_->mutex);
    msg.swap(inbox_->latest);
  }
  if (msg != nullptr) {
    ProcessMessage(*msg);
  }

  std::string query_error;
  for (const auto& session : sessions_) {
    int num_ongoing_requests = 0;
    for (const auto& submap : session.second->submaps) {
      submap.second->ApplyFetchedTexture(client_);
      if (submap.second->QueryInProgress()) {
        ++num_ongoing_requests;
      }
      if (query_error.empty() && !submap.second->last_error().empty()) {
        query_error = submap.second->last_error();
      }
    }
    // Newest submaps first: they are the ones still changing.
    for (auto it = session.second->submaps.rbegin();
         it != session.second->submaps.rend() &&
         num_ongoing_requests < kMaxOnGoingRequestsPerSession;
         ++it) {
      if (it->second->MaybeFetchTexture(client_)) {
        ++num_ongoing_requests;
      }
    }
  }

  if (query_error.empty()) {
    setStatus(::rviz_common::properties::StatusProperty::Ok, "Submap query", "OK");
  } else {
    setStatus(::rviz_common::properties::StatusProperty::Warn, "Submap query",
              QString::fromStdString(query_error));
  }

  Ogre::Vector3 map_position;
  Ogre::Quaternion map_orientation;
  if (!context_->getFrameManager()->getTransform(map_frame_property_->getStdString(), map_position,
                                                 map_orientation)) {
    setStatus(::rviz_common::properties::StatusProperty::Warn, "Transform",
              QString("No transform from [%1] to the fixed frame.")
                  .arg(map_frame_property_->getString()));
    return;
  }
  setStatus(::rviz_common::properties::StatusProperty::Ok, "Transform", "OK");
  map_node_->setPosition(map_position);
  map_node_->setOrientation(map_orientation);

  Ogre::Vector3 tracking_position;
  Ogre::Quaternion tracking_orientation;
  if (context_->getFrameManager()->getTransform(tracking_frame_property_->getStdString(),
                                                tracking_position, tracking_orientation)) {
    const double tracking_z = tracking_position.z - map_position.z;
    for (const auto& session : sessions_) {
      for (const auto& submap : session.second->submaps) {
        submap.second->SetAlpha(tracking_z, fade_out_start_distance_in_meters_->getFloat());
      }
    }
  }
  context_->queueRender();
}

void SubmapsDisplay::AllEnabledToggled() {
  const bool visible = visibility_all_enabled_->getBool();
  for (const auto& session : sessions_) {
    session.second->visibility->setBool(visible);
  }
}

void SubmapsDisplay::PoseMarkersEnabledToggled() {
  const bool visible = pose_markers_all_enabled_->getBool();
  for (const auto& session : sessions_) {
    session.second->pose_markers_visibility->setBool(visible);
  }
}

}  // namespace evergreenslam_rviz

PLUGINLIB_EXPORT_CLASS(evergreenslam_rviz::SubmapsDisplay, ::rviz_common::Display)
