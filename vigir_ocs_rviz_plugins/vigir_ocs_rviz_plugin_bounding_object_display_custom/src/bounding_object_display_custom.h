/*
 * BoundingObjectDisplayCustom declaration.
 *
 * Author: Felipe Bacim.
 *
 * help with selection of robot parts 
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

#ifndef RVIZ_BOUNDING_OBJECT_DISPLAY_H
#define RVIZ_BOUNDING_OBJECT_DISPLAY_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Axes;
class RenderPanel;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class VectorProperty;
class StringProperty;
class QuaternionProperty;
}

namespace rviz
{

/**
 * \class BoundingObjectDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class BoundingObjectDisplayCustom: public Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  BoundingObjectDisplayCustom();
  virtual ~BoundingObjectDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void fixedFrameChanged();
  virtual void reset();

  void clear();

  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
  virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);

private Q_SLOTS:
  void updateObjectProperties();
  void updateTopic();
  void setRenderPanel(rviz::RenderPanel*);
  void updateName();

protected:
  void setPose();
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void updatePose( const geometry_msgs::PoseStamped::ConstPtr& pose );

  void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

  float time_since_last_transform_;

  RosTopicProperty* topic_property_;
  FloatProperty* alpha_property_;
  VectorProperty* scale_property_;
  ColorProperty* color_property_;
  VectorProperty* offset_property_;
  VectorProperty* position_property_;
  StringProperty* name_property_;
  StringProperty* type_property_;
  QuaternionProperty* rotation_property_;

  geometry_msgs::Pose pose_;

  ros::NodeHandle nh_;

  Ogre::SceneNode* bounding_object_;
  Ogre::MaterialPtr bounding_object_material_;

  ros::Subscriber pose_sub_;

  std::vector<RenderPanel*> render_panel_list_;
  RenderPanel* render_panel_; // this is the active render panel

  bool initialized_;
};

} // namespace rviz

#endif


