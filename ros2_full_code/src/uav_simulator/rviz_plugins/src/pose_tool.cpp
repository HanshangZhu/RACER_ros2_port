// ROS 2 port of rviz_plugins/pose_tool.cpp.

#include "pose_tool.h"

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace {
rviz_rendering::ViewportProjectionFinder g_projection_finder;
}

#include <cmath>

namespace rviz_plugins {

Pose3DTool::Pose3DTool() : rviz_common::Tool() {
}

Pose3DTool::~Pose3DTool() {
  delete arrow_;
  for (auto* a : arrow_array_) delete a;
}

void Pose3DTool::onInitialize() {
  arrow_ = new rviz_rendering::Arrow(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate() {
  setStatus("Click and drag left mouse to set position/orientation; "
            "right-drag to raise/lower Z.");
  state_ = Position;
}

void Pose3DTool::deactivate() {
  arrow_->getSceneNode()->setVisible(false);
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  int flags = 0;
  static double initz{0.0}, prevz{0.0}, prevangle{0.0};
  const double z_scale = 50;
  const double z_interval = 0.5;
  Ogre::Quaternion orient_x(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

  if (event.leftDown()) {
    auto hit = g_projection_finder.getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
    if (hit.first) {
      pos_ = hit.second;
      arrow_->setPosition(pos_);
      state_ = Orientation;
      flags |= Render;
    }
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == Orientation) {
      auto hit = g_projection_finder.getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);
      if (hit.first) {
        const Ogre::Vector3& cur_pos = hit.second;
        double angle = std::atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setOrientation(orient_x);
        if (event.right()) state_ = Height;
        initz = pos_.z;
        prevz = event.y;
        prevangle = angle;
        flags |= Render;
      }
    }
    if (state_ == Height) {
      double z = event.y;
      double dz = z - prevz;
      prevz = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition(pos_);
      for (auto* a : arrow_array_) delete a;
      arrow_array_.clear();
      int cnt = static_cast<int>(std::ceil(std::fabs(initz - pos_.z) / z_interval));
      for (int k = 0; k < cnt; k++) {
        auto* marker = new rviz_rendering::Arrow(scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        marker->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        marker->getSceneNode()->setVisible(true);
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = initz - ((initz - pos_.z > 0) ? 1 : -1) * k * z_interval;
        marker->setPosition(arr_pos);
        marker->setOrientation(
            Ogre::Quaternion(Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z) * orient_x);
        arrow_array_.push_back(marker);
      }
      flags |= Render;
    }
  } else if (event.leftUp()) {
    if (state_ == Orientation || state_ == Height) {
      for (auto* a : arrow_array_) delete a;
      arrow_array_.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished | Render);
    }
  }
  return flags;
}

}  // namespace rviz_plugins
