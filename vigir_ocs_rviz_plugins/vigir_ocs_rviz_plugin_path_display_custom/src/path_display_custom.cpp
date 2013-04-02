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

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreMatrix4.h>
#include <OGRE/OgreMovableObject.h>
#include "OGRE/OgreEntity.h"

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"

#include "path_display_custom.h"

namespace rviz
{

PathDisplayCustom::PathDisplayCustom()
{
  color_property_ = new ColorProperty( "Color", QColor( 25, 255, 0 ),
                                       "Color to draw the path.", this );

  alpha_property_ = new FloatProperty( "Alpha", 1.0,
                                       "Amount of transparency to apply to the path.", this );

  buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                             "Number of paths to display.",
                                             this, SLOT( updateBufferLength() ));
  buffer_length_property_->setMin( 1 );
}

PathDisplayCustom::~PathDisplayCustom()
{
  destroyObjects();
}

void PathDisplayCustom::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void PathDisplayCustom::reset()
{
  MFDClass::reset();
  updateBufferLength();
}

void PathDisplayCustom::destroyObjects()
{
  for( size_t i = 0; i < manual_objects_.size(); i++ )
  {
    Ogre::ManualObject* manual_object = manual_objects_[ i ];
    if( manual_object )
    {
      manual_object->clear();
      scene_manager_->destroyManualObject( manual_object );
    }
  }
  for( size_t i = 0; i < waypoint_markers_.size(); i++ )
  	this->scene_node_->removeChild(waypoint_markers_[i]);
  waypoint_markers_.clear();
}

void PathDisplayCustom::updateBufferLength()
{
  destroyObjects();

  int buffer_length = buffer_length_property_->getInt();
  QColor color = color_property_->getColor();

  manual_objects_.resize( buffer_length );
  for( size_t i = 0; i < manual_objects_.size(); i++ )
  {
    Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
    manual_object->setDynamic( true );
    scene_node_->attachObject( manual_object );

    manual_objects_[ i ] = manual_object;
  }
}

bool validateFloats( const nav_msgs::Path& msg )
{
  bool valid = true;
  valid = valid && validateFloats( msg.poses );
  return valid;
}

void PathDisplayCustom::processMessage( const nav_msgs::Path::ConstPtr& msg )
{
  Ogre::ManualObject* manual_object = manual_objects_[ messages_received_ % buffer_length_property_->getInt() ];
  manual_object->clear();

  if( !validateFloats( *msg ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  Ogre::Matrix4 transform( orientation );
  transform.setTrans( position );

//  scene_node_->setPosition( position );
//  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  uint32_t num_points = msg->poses.size();
  manual_object->estimateVertexCount( num_points );
  manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
  for( uint32_t i=0; i < num_points; ++i)
  {
    const geometry_msgs::Point& pos = msg->poses[ i ].pose.position;
    Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
    manual_object->position( xpos.x, xpos.y, xpos.z );
    manual_object->colour( color );

    int index = waypoint_markers_.size();
    std::ostringstream convert;
	convert << index;
    Ogre::Entity* lEntity = this->scene_manager_->createEntity(std::string("waypoint marker ")+convert.str(), Ogre::SceneManager::PT_CUBE);
    waypoint_markers_.push_back(this->scene_node_->createChildSceneNode());
    waypoint_markers_[index]->attachObject(lEntity);
    waypoint_markers_[index]->setPosition(xpos.x, xpos.y, xpos.z);
    waypoint_markers_[index]->scale(0.001f,0.001f,0.001f);
  }

  manual_object->end();
}

void PathDisplayCustom::transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose)
{
    tf::Quaternion bt_orientation(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Vector3 bt_position(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);

    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), pose.header.frame_id);
    tf::Stamped<tf::Pose> pose_out;

    try
    {
        context_->getFrameManager()->getTFClient()->transformPose( target_frame.c_str(), pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", pose.header.frame_id.c_str(), target_frame.c_str(), e.what());
        return;
    }

    bt_position = pose_out.getOrigin();
    bt_orientation = pose_out.getRotation();

    pose.pose.position.x = bt_position.x();
    pose.pose.position.y = bt_position.y();
    pose.pose.position.z = bt_position.z();
    pose.pose.orientation.x = bt_orientation.x();
    pose.pose.orientation.y = bt_orientation.y();
    pose.pose.orientation.z = bt_orientation.z();
    pose.pose.orientation.w = bt_orientation.w();

    pose.header.frame_id = target_frame;
}

} // namespace rviz

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS( rviz::PathDisplayCustom, rviz::Display )
