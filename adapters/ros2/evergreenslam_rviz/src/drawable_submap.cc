/**
 * @file drawable_submap.cc
 * @author hang chen (chen@hang.plus)
 * @brief One submap in the scene: its pose, its texture and the query that keeps it current.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#include "evergreenslam_rviz/drawable_submap.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "utils/transform/transform.h"

namespace evergreenslam_rviz {

namespace {

constexpr std::chrono::milliseconds kMinQueryDelay(250);
constexpr std::chrono::seconds kQueryTimeout(10);
constexpr float kAlphaUpdateThreshold = 0.2f;

const Ogre::ColourValue kSubmapIdColor(Ogre::ColourValue::Red);
const Ogre::Vector3 kSubmapIdPosition(0.f, 0.f, 0.3f);
constexpr float kSubmapIdCharHeight = 0.2f;

Eigen::Affine2d ToAffine2d(const geometry_msgs::msg::Pose& pose) {
  const auto& q = pose.orientation;
  const double yaw = std::atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z));
  return ::evergreenslam::utils::transform::FromXYTheta(pose.position.x, pose.position.y, yaw);
}

Ogre::Quaternion ToOgre(const double yaw) {
  return Ogre::Quaternion(std::cos(0.5 * yaw), 0., 0., std::sin(0.5 * yaw));
}

}  // namespace

DrawableSubmap::DrawableSubmap(const int session_id, const int submap_index,
                               ::rviz_common::DisplayContext* const display_context,
                               Ogre::SceneNode* const map_node,
                               ::rviz_common::properties::Property* const submap_category,
                               const bool visible, const bool pose_markers_visible,
                               const float pose_axes_length, const float pose_axes_radius)
    : session_id_(session_id),
      submap_index_(submap_index),
      display_context_(display_context),
      submap_node_(map_node->createChildSceneNode()),
      submap_id_text_node_(submap_node_->createChildSceneNode()),
      ogre_slice_(session_id, submap_index, display_context->getSceneManager(), submap_node_),
      pose_axes_(display_context->getSceneManager(), submap_node_, pose_axes_length,
                 pose_axes_radius),
      pose_markers_visible_(pose_markers_visible),
      submap_id_text_(QString("%1/%2").arg(session_id).arg(submap_index).toStdString()),
      last_query_time_(std::chrono::steady_clock::time_point::min()) {
  // The Qt parent of the checkbox is the submap category, which outlives this object, so the
  // property is owned here and destroyed with the submap it stands for.
  visibility_ = std::make_unique<::rviz_common::properties::BoolProperty>(
      "" /* title */, visible, "" /* description */, submap_category, SLOT(ToggleVisibility()),
      this);
  submap_id_text_.setCharacterHeight(kSubmapIdCharHeight);
  submap_id_text_.setColor(kSubmapIdColor);
  submap_id_text_.setTextAlignment(::rviz_rendering::MovableText::H_CENTER,
                                   ::rviz_rendering::MovableText::V_ABOVE);
  submap_id_text_node_->setPosition(kSubmapIdPosition);
  submap_id_text_node_->attachObject(&submap_id_text_);
  set_pose_markers_visibility(pose_markers_visible_);
  ToggleVisibility();
}

DrawableSubmap::~DrawableSubmap() {
  display_context_->getSceneManager()->destroySceneNode(submap_id_text_node_);
  display_context_->getSceneManager()->destroySceneNode(submap_node_);
}

void DrawableSubmap::Update(const ::evergreenslam_msgs::msg::SubmapEntry& entry) {
  metadata_version_ = entry.submap_version;
  pose_ = ToAffine2d(entry.pose);
  pose_z_ = entry.pose.position.z;
  submap_node_->setPosition(static_cast<float>(pose_.translation().x()),
                            static_cast<float>(pose_.translation().y()),
                            static_cast<float>(pose_z_));
  submap_node_->setOrientation(ToOgre(::evergreenslam::utils::transform::GetYaw(pose_)));
  visibility_->setName(QString("%1.%2").arg(submap_index_).arg(metadata_version_));
  visibility_->setDescription(QString("Toggle visibility of this individual submap.<br><br>"
                                      "Session %1, submap %2, version %3, %4, %5")
                                  .arg(session_id_)
                                  .arg(submap_index_)
                                  .arg(metadata_version_)
                                  .arg(entry.is_finished ? "finished" : "open")
                                  .arg(entry.is_frozen ? "frozen" : "active"));
  display_context_->queueRender();
}

bool DrawableSubmap::MaybeFetchTexture(
    const rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedPtr& client) {
  if (query_ != nullptr || texture_version_ == metadata_version_ ||
      failed_version_ == metadata_version_ || client == nullptr || !client->service_is_ready()) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_query_time_ + kMinQueryDelay > now) {
    return false;
  }
  last_query_time_ = now;
  query_ = std::make_shared<Query>();
  auto request = std::make_shared<::evergreenslam_msgs::srv::SubmapQuery::Request>();
  request->session_id = session_id_;
  request->submap_index = submap_index_;
  auto query = query_;
  // The response lands on whatever thread spins the rviz node, so it only fills in the shared
  // Query; ApplyFetchedTexture picks it up on the render thread.
  const auto future = client->async_send_request(
      std::move(request),
      [query](rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedFuture response) {
        const auto result = response.get();
        const std::lock_guard<std::mutex> lock(query->mutex);
        query->done = true;
        query->version = result->submap_version;
        query->succeeded = result->error_message.empty();
        if (query->succeeded) {
          query->texture = result->texture;
        } else {
          query->error = result->error_message;
        }
      });
  query_request_id_ = future.request_id;
  return true;
}

void DrawableSubmap::ApplyFetchedTexture(
    const rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedPtr& client) {
  if (query_ == nullptr) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(query_->mutex);
    if (query_->done) {
      if (query_->succeeded && ogre_slice_.Update(query_->texture)) {
        texture_version_ = query_->version;
        last_error_.clear();
        ToggleVisibility();
        display_context_->queueRender();
      } else {
        failed_version_ = metadata_version_;
        last_error_ = query_->succeeded ? "malformed texture" : query_->error;
      }
      query_.reset();
      return;
    }
  }
  if (last_query_time_ + kQueryTimeout < std::chrono::steady_clock::now()) {
    if (client != nullptr) {
      client->remove_pending_request(query_request_id_);
    }
    query_.reset();
  }
}

void DrawableSubmap::SetAlpha(const double current_tracking_z,
                              const float fade_out_start_distance_in_meters) {
  const float fade_out_distance_in_meters = 2.f * fade_out_start_distance_in_meters;
  const double distance_z = std::abs(pose_z_ - current_tracking_z);
  const double fade_distance = std::max(distance_z - fade_out_start_distance_in_meters, 0.);
  // A zero distance means no fading, not 0 / 0.
  const float target_alpha =
      fade_out_distance_in_meters <= 0.f
          ? 1.f
          : static_cast<float>(std::max(0., 1. - fade_distance / fade_out_distance_in_meters));
  if (std::abs(target_alpha - current_alpha_) > kAlphaUpdateThreshold || target_alpha == 0.f ||
      target_alpha == 1.f) {
    current_alpha_ = target_alpha;
  }
  ogre_slice_.SetAlpha(current_alpha_);
}

void DrawableSubmap::set_pose_markers_visibility(const bool visibility) {
  pose_markers_visible_ = visibility;
  submap_id_text_node_->setVisible(visibility);
  pose_axes_.getSceneNode()->setVisible(visibility);
}

void DrawableSubmap::ReapplyVisibility() {
  set_pose_markers_visibility(pose_markers_visible_);
  ToggleVisibility();
}

void DrawableSubmap::ToggleVisibility() {
  ogre_slice_.SetVisible(visibility_->getBool());
  display_context_->queueRender();
}

}  // namespace evergreenslam_rviz
