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

#include <GL/gl.h>

#include "path_display_custom.h"

namespace rviz
{

PathDisplayCustom::PathDisplayCustom() :
    z_offset (0.5f)
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

void PathDisplayCustom::update( float wall_dt, float ros_dt )
{
    //std::cout << "Path display update" << std::endl;
    draw(last_msg_);
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

void PathDisplayCustom::draw(nav_msgs::Path path)
{
    Ogre::ManualObject* manual_object = manual_objects_[ messages_received_ % buffer_length_property_->getInt() ];
    manual_object->clear();

    for( size_t i = 0; i < waypoint_markers_.size(); i++ )
    {
        this->scene_manager_->destroyEntity((Ogre::Entity*)waypoint_markers_[i]->getAttachedObject(0));
        this->scene_node_->removeChild(waypoint_markers_[i]);
    }
    waypoint_markers_.clear();

    /*
    Ogre::Vector3 positiono(0,0,0);
    Ogre::Quaternion orientationo(0,0,0,1);

    if( !context_->getFrameManager()->getTransform( path.header, positiono, orientationo ))
    {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", path.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    }

    std::cout << "FrameManager->getTransform" << std::endl;
    std::cout << "  position:    " << positiono.x << ", " << positiono.y << ", " << positiono.z << std::endl;
    std::cout << "  orientation: " << orientationo.x << ", " << orientationo.y << ", " << orientationo.z << ", " << orientationo.w << std::endl;
    */

    Ogre::Vector3 position(0,0,0);
    Ogre::Quaternion orientation = Ogre::Quaternion::IDENTITY;

    transform(path.header.frame_id,fixed_frame_.toUtf8().constData(),position,orientation);

    //std::cout << "after transform" << std::endl;
    //std::cout << "  position:    " << position.x << ", " << position.y << ", " << position.z << std::endl;
    //std::cout << "  orientation: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;

    //scene_node_->setPosition( position );
    //scene_node_->setOrientation( orientation );

    Ogre::Matrix4 transform( orientation );
    transform.setTrans( position );

    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();

    uint32_t num_points = path.poses.size();
    manual_object->estimateVertexCount( num_points );
    manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );

    manual_object->position( 0, 0, 0 ); // will only work if fixed frame is pelvis
    manual_object->colour( color );

    glLineWidth(4.0f);

    static int counter = 0;

    for( uint32_t i=0; i < num_points; ++i)
    {
        const geometry_msgs::Point& pos = path.poses[ i ].pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z+z_offset );
        manual_object->position( xpos.x, xpos.y, xpos.z );
        manual_object->colour( color );

        int index = waypoint_markers_.size();
        std::ostringstream convert;
        convert << counter++ << "." << index;
        Ogre::Entity* lEntity = this->scene_manager_->createEntity(std::string("waypoint marker ")+convert.str(), Ogre::SceneManager::PT_CUBE);
        lEntity->setUserAny(Ogre::Any(std::string("waypoint marker ")+convert.str()));
        waypoint_markers_.push_back(this->scene_node_->createChildSceneNode());
        waypoint_markers_[index]->attachObject(lEntity);
        waypoint_markers_[index]->setPosition(xpos.x, xpos.y, xpos.z-z_offset/2.0f);
        waypoint_markers_[index]->scale(0.001f,0.001f,z_offset/100.0f);
    }

    manual_object->end();
    glLineWidth(1.0f);
}

void PathDisplayCustom::processMessage( const nav_msgs::Path::ConstPtr& msg )
{
    if( !validateFloats( *msg ))
    {
        setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
        return;
    }

    last_msg_ = *msg;
    //std::cout << "drawing" << std::endl;
    //draw(last_msg_);
}

void PathDisplayCustom::transform(const std::string& from_frame, const std::string& to_frame, Ogre::Vector3& position, Ogre::Quaternion& orientation)
{
    tf::Quaternion bt_orientation(orientation.x,orientation.y,orientation.z,orientation.w);
    tf::Vector3 bt_position(position.x,position.y,position.z);

    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), from_frame);
    tf::Stamped<tf::Pose> pose_out;

    try
    {
        context_->getFrameManager()->getTFClient()->transformPose( to_frame, pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", from_frame.c_str(), to_frame.c_str(), e.what());
        return;
    }

    bt_position = pose_out.getOrigin();
    bt_orientation = pose_out.getRotation();

    position.x = bt_position.x();
    position.y = bt_position.y();
    position.z = bt_position.z();
    orientation.x = bt_orientation.x();
    orientation.y = bt_orientation.y();
    orientation.z = bt_orientation.z();
    orientation.w = bt_orientation.w();
}

} // namespace rviz

