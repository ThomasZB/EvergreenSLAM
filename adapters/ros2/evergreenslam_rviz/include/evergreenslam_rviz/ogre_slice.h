/**
 * @file ogre_slice.h
 * @author hang chen (chen@hang.plus)
 * @brief The textured quad one submap is drawn as.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#ifndef EVERGREENSLAM_RVIZ_OGRE_SLICE_H_
#define EVERGREENSLAM_RVIZ_OGRE_SLICE_H_

#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>

#include <string>

#include "evergreenslam_msgs/msg/submap_texture.hpp"

namespace evergreenslam_rviz {

// The Ogre side of one submap. Every member is expected to be called from the render thread.
class OgreSlice {
 public:
  // Throws std::runtime_error when the package's material is not loaded.
  OgreSlice(int session_id, int submap_index, Ogre::SceneManager* scene_manager,
            Ogre::SceneNode* submap_node);
  ~OgreSlice();

  OgreSlice(const OgreSlice&) = delete;
  OgreSlice& operator=(const OgreSlice&) = delete;

  // False when the texture is malformed (empty, or cells not width * height).
  bool Update(const evergreenslam_msgs::msg::SubmapTexture& texture);

  void SetAlpha(float alpha);

  void SetVisible(bool visible);

 private:
  const std::string identifier_;
  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const slice_node_;
  Ogre::ManualObject* const manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};

}  // namespace evergreenslam_rviz

#endif  // EVERGREENSLAM_RVIZ_OGRE_SLICE_H_
