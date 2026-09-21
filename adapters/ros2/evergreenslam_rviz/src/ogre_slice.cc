/**
 * @file ogre_slice.cc
 * @author hang chen (chen@hang.plus)
 * @brief The textured quad one submap is drawn as.
 * @version 0.1
 * @date 2026-09-16
 *
 * @copyright Copyright (c) 2026
 *
 */

// Ported from cartographer_rviz, Copyright 2016 The Cartographer Authors, Apache License 2.0.

#include "evergreenslam_rviz/ogre_slice.h"

#include <OgreGpuProgramParams.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace evergreenslam_rviz {

namespace {

constexpr char kManualObjectPrefix[] = "EvergreenSlamSubmapObject";
constexpr char kSourceMaterialName[] = "evergreenslam_rviz/Submap";
constexpr char kMaterialPrefix[] = "EvergreenSlamSubmapMaterial";
constexpr char kTexturePrefix[] = "EvergreenSlamSubmapTexture";

constexpr uint8_t kUnknownCell = 255;
// Alpha ramps from 0 at even odds to 1 once the probability is this far across.
constexpr double kFullyOpaqueProbabilitySpan = 0.7;

// Ogre names are global to the process; a serial keeps two displays showing the same submap
// (two robots, or a restarted node) from colliding.
std::string SliceIdentifier(const int session_id, const int submap_index) {
  static std::atomic<int> serial{0};
  return std::to_string(serial++) + "-" + std::to_string(session_id) + "-" +
         std::to_string(submap_index);
}

}  // namespace

OgreSlice::OgreSlice(const int session_id, const int submap_index,
                     Ogre::SceneManager* const scene_manager, Ogre::SceneNode* const submap_node)
    : identifier_(SliceIdentifier(session_id, submap_index)),
      scene_manager_(scene_manager),
      slice_node_(submap_node->createChildSceneNode()),
      manual_object_(scene_manager_->createManualObject(kManualObjectPrefix + identifier_)) {
  material_ = Ogre::MaterialManager::getSingleton().getByName(kSourceMaterialName);
  if (!material_) {
    scene_manager_->destroyManualObject(manual_object_);
    scene_manager_->destroySceneNode(slice_node_);
    throw std::runtime_error(std::string("material ") + kSourceMaterialName +
                             " not found: is ogre_media installed with the package?");
  }
  material_ = material_->clone(kMaterialPrefix + identifier_, true, "General");
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthBias(-1.f, 0.f);
  material_->setDepthWriteEnabled(false);
  slice_node_->attachObject(manual_object_);
}

OgreSlice::~OgreSlice() {
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.reset();
  }
  scene_manager_->destroySceneNode(slice_node_);
  scene_manager_->destroyManualObject(manual_object_);
}

bool OgreSlice::Update(const evergreenslam_msgs::msg::SubmapTexture& texture) {
  if (texture.width <= 0 || texture.height <= 0 ||
      texture.cells.size() !=
          static_cast<size_t>(texture.width) * static_cast<size_t>(texture.height)) {
    return false;
  }
  slice_node_->setPosition(texture.slice_pose.position.x, texture.slice_pose.position.y,
                           texture.slice_pose.position.z);
  slice_node_->setOrientation(Ogre::Quaternion::IDENTITY);

  // Ogre's loadRawData refuses a two channel texture, so the intensity goes in R, the alpha in G
  // and B stays 0; the fragment shader reads exactly those two channels.
  std::vector<uint8_t> rgb;
  rgb.reserve(texture.cells.size() * 3);
  for (const uint8_t cell : texture.cells) {
    if (cell == kUnknownCell) {
      rgb.push_back(0);
      rgb.push_back(0);
      rgb.push_back(0);
      continue;
    }
    const double probability = std::min<double>(cell, 100) / 100.0;
    const double alpha =
        std::min(1.0, std::abs(probability - 0.5) * 2.0 / kFullyOpaqueProbabilitySpan);
    rgb.push_back(static_cast<uint8_t>(std::lround(255.0 * (1.0 - probability))));
    rgb.push_back(static_cast<uint8_t>(std::lround(255.0 * alpha)));
    rgb.push_back(0);
  }

  const float metric_width = static_cast<float>(texture.resolution * texture.width);
  const float metric_height = static_cast<float>(texture.resolution * texture.height);
  manual_object_->clear();
  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  // Cell (0, 0) sits at the slice node and the first uploaded row is v = 0, so the texture is
  // upright when the quad's y = 0 edge carries v = 0.
  manual_object_->position(0.f, 0.f, 0.f);
  manual_object_->textureCoord(0.f, 0.f);
  manual_object_->position(metric_width, 0.f, 0.f);
  manual_object_->textureCoord(1.f, 0.f);
  manual_object_->position(0.f, metric_height, 0.f);
  manual_object_->textureCoord(0.f, 1.f);
  manual_object_->position(metric_width, metric_height, 0.f);
  manual_object_->textureCoord(1.f, 1.f);
  manual_object_->end();

  Ogre::DataStreamPtr pixel_stream(new Ogre::MemoryDataStream(rgb.data(), rgb.size()));
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.reset();
  }
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      kTexturePrefix + identifier_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, texture.width, texture.height, Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D, 0);

  Ogre::Pass* const pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_SOURCE_ALPHA, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const texture_unit = pass->getNumTextureUnitStates() > 0
                                                   ? pass->getTextureUnitState(0)
                                                   : pass->createTextureUnitState();
  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);
  return true;
}

void OgreSlice::SetAlpha(const float alpha) {
  material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant(
      "u_alpha", alpha);
}

void OgreSlice::SetVisible(const bool visible) { slice_node_->setVisible(visible); }

}  // namespace evergreenslam_rviz
