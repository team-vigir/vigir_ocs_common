/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#ifndef JOINT_MARKER_DISPLAY_CUSTOM_H
#define JOINT_MARKER_DISPLAY_CUSTOM_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"

#include <joint_visual_display_custom.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

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

#include "robot_state_manager.h"

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
 * \class JointMarkerDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class JointMarkerDisplayCustom: public Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  JointMarkerDisplayCustom();
  virtual ~JointMarkerDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void fixedFrameChanged();
  virtual void reset();

  void clear();

  void setArrowDirection(std::string joint_name,int direction);
  void setJointColor(QColor color,std::string joint_name);
  void setJointAlpha(float alpha,std::string joint_name);

  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);
  virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation);

private Q_SLOTS:
  void updateWidthAndScale();
  void updateTopic();
  void setRenderPanel(rviz::RenderPanel*);
  void updateName();
  void updateRobotDescription();
  void updateGhost();

protected:
  void setPose();
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void calculateGhostTransform(std::string link_name, std::string joint_name, Ogre::Vector3& position, Ogre::Quaternion& orientation);
  void processMessage( const sensor_msgs::JointState::ConstPtr& msg );
  void processPelvisEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose);
  // The object for urdf model
  boost::shared_ptr<urdf::Model> robot_model_;

  //
  std::string robot_description_;

  RosTopicProperty* topic_property_;
  FloatProperty * width_property_,* scale_property_;
  StringProperty* name_property_;
  StringProperty *robot_description_property_;
  BoolProperty * is_ghost_property_;


  ros::NodeHandle nh_;

  ros::Subscriber joint_states_sub_;

  boost::shared_ptr<JointVisualCustom> visual_;

  std::vector<RenderPanel*> render_panel_list_;
  RenderPanel* render_panel_; // this is the active render panel

  ros::Subscriber ghost_root_sub_;
  geometry_msgs::Pose ghost_root_pose_;

  bool initialized_;
  bool is_ghost_;
};

} // namespace rviz

#endif // JOINT_MARKER_DISPLAY_CUSTOM_H
