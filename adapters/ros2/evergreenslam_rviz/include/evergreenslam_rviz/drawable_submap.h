/**
 * @file drawable_submap.h
 * @author hang chen (chen@hang.plus)
 * @brief One submap in the scene: its pose, its texture and the query that keeps it current.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#ifndef EVERGREENSLAM_RVIZ_DRAWABLE_SUBMAP_H_
#define EVERGREENSLAM_RVIZ_DRAWABLE_SUBMAP_H_

#include <OgreSceneNode.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "evergreenslam_msgs/msg/submap_entry.hpp"
#include "evergreenslam_msgs/msg/submap_texture.hpp"
#include "evergreenslam_msgs/srv/submap_query.hpp"
#include "evergreenslam_rviz/ogre_slice.h"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

namespace evergreenslam_rviz {

// Everything needed to draw one submap. Every member runs on the render thread; only the service
// response lands elsewhere, and it only writes the shared Query below.
class DrawableSubmap : public QObject {
  Q_OBJECT

 public:
  DrawableSubmap(int session_id, int submap_index, ::rviz_common::DisplayContext* display_context,
                 Ogre::SceneNode* map_node, ::rviz_common::properties::Property* submap_category,
                 bool visible, bool pose_markers_visible, float pose_axes_length,
                 float pose_axes_radius);
  ~DrawableSubmap() override;

  DrawableSubmap(const DrawableSubmap&) = delete;
  DrawableSubmap& operator=(const DrawableSubmap&) = delete;

  void Update(const ::evergreenslam_msgs::msg::SubmapEntry& entry);

  // Sends a texture query if the version moved on and no query is in flight; returns whether one
  // was sent.
  bool MaybeFetchTexture(
      const rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedPtr& client);

  // Hands an arrived texture to Ogre, or drops a query that never came back.
  void ApplyFetchedTexture(
      const rclcpp::Client<::evergreenslam_msgs::srv::SubmapQuery>::SharedPtr& client);

  bool QueryInProgress() const { return query_ != nullptr; }

  void SetAlpha(double current_tracking_z, float fade_out_start_distance_in_meters);

  int version() const { return metadata_version_; }
  // Empty unless the last query for the current version failed.
  const std::string& last_error() const { return last_error_; }
  void set_visibility(bool visibility) { visibility_->setBool(visibility); }
  void set_pose_markers_visibility(bool visibility);
  // Pushes the stored checkbox state back into the scene after something else changed it.
  void ReapplyVisibility();

 private Q_SLOTS:
  void ToggleVisibility();

 private:
  struct Query {
    std::mutex mutex;
    bool done = false;
    bool succeeded = false;
    int version = -1;
    std::string error;
    ::evergreenslam_msgs::msg::SubmapTexture texture;
  };

  const int session_id_;
  const int submap_index_;
  ::rviz_common::DisplayContext* const display_context_;
  Ogre::SceneNode* const submap_node_;
  Ogre::SceneNode* const submap_id_text_node_;
  OgreSlice ogre_slice_;
  ::rviz_rendering::Axes pose_axes_;
  bool pose_markers_visible_;
  ::rviz_rendering::MovableText submap_id_text_;
  std::unique_ptr<::rviz_common::properties::BoolProperty> visibility_;

  Eigen::Affine2d pose_ = Eigen::Affine2d::Identity();
  double pose_z_ = 0.;
  int metadata_version_ = -1;
  int texture_version_ = -1;
  // The version whose query failed; not retried until the list moves the version on.
  int failed_version_ = -1;
  std::string last_error_;
  float current_alpha_ = 0.f;
  std::shared_ptr<Query> query_;
  int64_t query_request_id_ = 0;
  std::chrono::steady_clock::time_point last_query_time_;
};

}  // namespace evergreenslam_rviz

#endif  // EVERGREENSLAM_RVIZ_DRAWABLE_SUBMAP_H_
