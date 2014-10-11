
/*
 * JointMarkerDisplayCustom class implementation.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreFrustum.h>

#include <urdf/model.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"
#include "robot_display_custom.h"

#include "joint_marker_display_custom.h"

namespace rviz
{

JointMarkerDisplayCustom::JointMarkerDisplayCustom()
    : Display()
    , initialized_(false)
{
    name_property_ = new StringProperty( "Name", "",
                                         "Name of the object.", this , SLOT(updateName()));

    width_property_ =
            new rviz::FloatProperty( "Width", 0.02,
                                     "Width to draw circle",
                                     this, SLOT( updateWidthAndScale() ));

    scale_property_ =
            new rviz::FloatProperty( "Scale", 1.0,
                                     "Scale to draw circle",
                                     this, SLOT( updateWidthAndScale() ));

    topic_property_ = new RosTopicProperty( "Topic", "",
                                            QString::fromStdString( ros::message_traits::datatype<sensor_msgs::JointState>() ),
                                            "geometry_msgs::PoseStamped topic to subscribe to.",
                                            this, SLOT( updateTopic() ));

    robot_description_property_ =
            new rviz::StringProperty( "Robot Description", "robot_description",
                                      "Name of the parameter to search for to load the robot description.",
                                      this, SLOT( updateRobotDescription() ) );

}

JointMarkerDisplayCustom::~JointMarkerDisplayCustom()
{
}

void JointMarkerDisplayCustom::updateName()
{

}

void JointMarkerDisplayCustom::onInitialize()
{
    initialized_ = true;
    context_->getSceneManager()->addRenderQueueListener(this);
}

void JointMarkerDisplayCustom::updateWidthAndScale()
{
    float width = width_property_->getFloat();
    float scale = scale_property_->getFloat();

    if(visual_)
    {
        visual_->setWidth( width );
        visual_->setScale( scale );
    }
}

void JointMarkerDisplayCustom::updateTopic()
{
    unsubscribe();
    subscribe();
}

void JointMarkerDisplayCustom::updateRobotDescription()
{
    if( isEnabled() )
    {
        load();
        context_->queueRender();  //??
    }
}

void JointMarkerDisplayCustom::load()
{
    // get robot_description
    std::string content;
    if (!update_nh_.getParam( robot_description_property_->getStdString(), content) )
    {
        std::string loc;
        if( update_nh_.searchParam( robot_description_property_->getStdString(), loc ))
        {
            update_nh_.getParam( loc, content );
        }
        else
        {
            clear();

            setStatus( rviz::StatusProperty::Error, "URDF",
                       "Parameter [" + robot_description_property_->getString()
                       + "] does not exist, and was not found by searchParam()" );
            ROS_ERROR("URDF not found");
            return;
        }
    }

    if( content.empty() )
    {
        clear();
        setStatus( rviz::StatusProperty::Error, "URDF", "URDF is empty" );
        ROS_ERROR("URDF is empty");
        return;
    }

    if( content == robot_description_ )
    {
        return;
    }

    robot_description_ = content;


    robot_model_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!robot_model_->initString(content))
    {
        ROS_ERROR("Unable to parse URDF description!");
        setStatus( rviz::StatusProperty::Error, "URDF", "Unable to parse robot model description!");
        return;
    }
    setStatus(rviz::StatusProperty::Ok, "URDF", "Robot model parserd Ok");

    visual_.reset(new JointVisualCustom(context_->getSceneManager(), scene_node_, robot_model_));
}

void JointMarkerDisplayCustom::subscribe()
{
    if ( !isEnabled() )
    {
        return;
    }

    if( !topic_property_->getTopic().isEmpty() )
    {
        try
        {
            joint_states_sub_ = nh_.subscribe( topic_property_->getTopicStd(), 1, &JointMarkerDisplayCustom::processMessage, this );
            setStatus( StatusProperty::Ok, "Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
        }
    }
}

void JointMarkerDisplayCustom::unsubscribe()
{
    joint_states_sub_.shutdown();
}

void JointMarkerDisplayCustom::onEnable()
{
    load();
    subscribe();
}

void JointMarkerDisplayCustom::onDisable()
{
    unsubscribe();
}

void JointMarkerDisplayCustom::preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt )
{
    context_->queueRender();
}

void JointMarkerDisplayCustom::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{

}

void JointMarkerDisplayCustom::renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& skipThisInvocation)
{

}

void JointMarkerDisplayCustom::renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, bool& repeatThisInvocation)
{

}

void JointMarkerDisplayCustom::setArrowDirection(std::string joint_name,int direction)
{
    if(visual_) visual_->setArrowDirection(joint_name, direction);
}

void JointMarkerDisplayCustom::setJointColor(QColor color,std::string joint_name)
{
    if(visual_) visual_->setJointColor(color.redF(),color.greenF(),color.blueF(),joint_name);
}

void JointMarkerDisplayCustom::setJointAlpha(float alpha,std::string joint_name)
{
    if(visual_) visual_ ->setJointAlpha(alpha,joint_name);
}


void JointMarkerDisplayCustom::update( float wall_dt, float ros_dt )
{
}

void JointMarkerDisplayCustom::fixedFrameChanged()
{
    //has_new_transforms_ = true;
    context_->queueRender();
}

void JointMarkerDisplayCustom::clear()
{
}

void JointMarkerDisplayCustom::reset()
{
    Display::reset();
    //has_new_transforms_ = true;
}

void JointMarkerDisplayCustom::setRenderPanel( rviz::RenderPanel* rp )
{
    if(std::find(render_panel_list_.begin(),render_panel_list_.end(),rp) != render_panel_list_.end())
    {
        this->render_panel_ = rp;
        this->render_panel_->getRenderWindow()->addListener( this );
    }
    else
    {
        this->render_panel_list_.push_back(rp);
        this->render_panel_ = rp;
        this->render_panel_->getRenderWindow()->addListener( this );
    }
}

// This is our callback to handle an incoming message.
void JointMarkerDisplayCustom::processMessage( const sensor_msgs::JointState::ConstPtr& msg )
{
    if(!visual_)
        return;

    std::vector<std::string> joints;
    int joint_num = msg->name.size();

    for (int i = 0; i < joint_num; ++i)
    {
        std::string joint_name = msg->name[i];

        const urdf::Joint* joint = robot_model_->getJoint(joint_name).get();
        if(!joint)
            ROS_ERROR("null joint");
        int joint_type = joint->type;
        if ( joint_type == urdf::Joint::REVOLUTE )
        {
            // we expect that parent_link_name equals to frame_id.
            std::string parent_link_name = joint->child_link_name;
            Ogre::Quaternion orientation;
            Ogre::Vector3 position;

            // Here we call the rviz::FrameManager to get the transform from the
            // fixed frame to the frame in the header of this Effort message.  If
            // it fails, we can't do anything else so we return.
            if( !context_->getFrameManager()->getTransform( parent_link_name,
                                                            ros::Time(),
                                                            //msg->header.stamp, // ???
                                                            position, orientation ))
            {
                ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                           parent_link_name.c_str(), qPrintable( fixed_frame_) );
                continue;
            }
            ;
            tf::Vector3 axis_joint(joint->axis.x, joint->axis.y, joint->axis.z);
            tf::Vector3 axis_z(0,0,1);
            tf::Quaternion axis_rotation(tf::tfCross(axis_joint, axis_z), tf::tfAngle(axis_joint, axis_z));
            if ( std::isnan(axis_rotation.x()) ||
                 std::isnan(axis_rotation.y()) ||
                 std::isnan(axis_rotation.z()) ) axis_rotation = tf::Quaternion::getIdentity();

            tf::Quaternion axis_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
            tf::Quaternion axis_rot = axis_orientation * axis_rotation;
            Ogre::Quaternion joint_orientation(Ogre::Real(axis_rot.w()), Ogre::Real(axis_rot.x()), Ogre::Real(axis_rot.y()), Ogre::Real(axis_rot.z()));
            visual_->setFramePosition( joint_name, position );
            visual_->setFrameOrientation( joint_name, joint_orientation );
        }
    }

    // Now set or update the contents of the chosen visual.
    float scale = scale_property_->getFloat();
    float width = width_property_->getFloat();
    visual_->setWidth( width );
    visual_->setScale( scale );
    visual_->setMessage( msg );
}

} // namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::JointMarkerDisplayCustom, rviz::Display )



