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

  void processDistQuery( const std_msgs::Float64::ConstPtr& distance );
  void processOCSDistQuery( const flor_ocs_msgs::OCSRaycastRequest::ConstPtr& request );

Q_SIGNALS:
  void newSelection( Ogre::Vector3 );
  void setContext( int );
  void setSelectionRay( Ogre::Ray );

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  void updateRobotDescription();
  void createMarker(int, int, int, int);
  void createMarker(bool, int, int);
  void createROISelection(bool,int,int);
  void resetSelection();
  void setRenderPanel(rviz::RenderPanel*);
  void setMarkerScale(float);
  void setMarkerPosition(float, float, float);
  void queryPosition( int, int, Ogre::Vector3& );
  void queryContext( int, int );

  void raycastRequest(bool, int, int);
  void raycastRequestROI(bool, int, int);

protected:
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

  void publishRayRequest(Ogre::Vector3 origin, Ogre::Vector3 direction);
  void publishOCSRayRequest(int mode, Ogre::Vector3 origin, Ogre::Vector3 direction);

  Ogre::Vector3 calculateRaycastPosition(double distance);

  //bool has_new_transforms_;      ///< Callback sets this to tell our update function it needs to update the transforms

  float time_since_last_transform_;

  ros::NodeHandle nh_;
  ros::Publisher raycast_query_pub_;
  ros::Publisher ocs_raycast_query_pub_;
  ros::Subscriber raycast_query_sub_;
  ros::Subscriber ocs_raycast_query_sub_;
  
  Ogre::SceneNode* ground_;
  Ogre::SceneNode* selection_marker_;
  Ogre::SceneNode* roi_marker_final_;
  Ogre::SceneNode* roi_marker_box_;

  RenderPanel* render_panel_;

  RayCastUtils* raycast_utils_;

  Ogre::Vector3 selection_position_;
  Ogre::Vector3 selection_position_roi_;

  bool initialized_;

  float marker_scale_;

  enum
  {
      RAYCAST_SELECTION,
      RAYCAST_SELECTION_ROI
  } RaycastRequestMode;

  int raycast_request_mode_;

  Ogre::Ray last_ray_;

  bool ray_initialized_;
};

} // namespace rviz

#endif

