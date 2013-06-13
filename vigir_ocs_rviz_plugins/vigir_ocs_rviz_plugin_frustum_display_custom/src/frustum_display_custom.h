/*
 * FrustumDisplayCustom declarion.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_frustum_DISPLAY_H
#define RVIZ_frustum_DISPLAY_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <flor_perception_msgs/RaycastRequest.h>
#include <flor_ocs_msgs/OCSRaycastRequest.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include "raycast_utils.h"

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz
{
class Axes;
class RenderPanel;
}

namespace rviz
{

/**
 * \class FrustumDisplayCustom
 * \brief Uses the window mouse information to create a selection marker
 */
class FrustumDisplayCustom: public Display
{
Q_OBJECT
public:
  FrustumDisplayCustom();
  virtual ~FrustumDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void fixedFrameChanged();
  virtual void reset();

  void clear();

public Q_SLOTS:
  void setFrustum( const float &fovy_rad, const float &aspect_ratio, const float& near, const float& far );

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateAlpha();

protected:
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

  std::vector<Ogre::ManualObject*> manual_objects_;

  Ogre::Frustum frustum_;

};

} // namespace rviz

#endif


