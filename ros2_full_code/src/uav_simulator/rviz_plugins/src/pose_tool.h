// ROS 2 port of rviz_plugins/pose_tool.h — base class for the Goal3DTool.
//
// Adapted from Willow Garage's PoseTool (BSD-3). Upstream rviz2's
// rviz_default_plugins::tools::PoseTool was not reused because this tool
// drags out a THIRD dimension via right-mouse-drag and emits an arrow array
// at intermediate heights, which the stock PoseTool does not support.

#pragma once

#include <OgreVector3.h>

#include <QCursor>

#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <vector>

namespace rviz_rendering {
class Arrow;
}

namespace rviz_plugins {

class Pose3DTool : public rviz_common::Tool {
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  rviz_rendering::Arrow* arrow_{nullptr};
  std::vector<rviz_rendering::Arrow*> arrow_array_;

  enum State { Position, Orientation, Height };
  State state_{Position};

  Ogre::Vector3 pos_;
};

}  // namespace rviz_plugins
