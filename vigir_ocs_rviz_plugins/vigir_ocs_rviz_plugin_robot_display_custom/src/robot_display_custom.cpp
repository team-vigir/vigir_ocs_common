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

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "robot_custom.h"
#include "tf_link_updater_custom.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/color_property.h"

#include "robot_display_custom.h"

namespace rviz
{

void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                RobotDisplayCustom* display )
{
    display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

RobotDisplayCustom::RobotDisplayCustom()
    : Display()
    , has_new_transforms_( false )
    , time_since_last_transform_( 0.0f )
{
    visual_enabled_property_ = new Property( "Visual Enabled", true,
                                             "Whether to display the visual representation of the robot.",
                                             this, SLOT( updateVisualVisible() ));

    collision_enabled_property_ = new Property( "Collision Enabled", false,
                                                "Whether to display the collision representation of the robot.",
                                                this, SLOT( updateCollisionVisible() ));

    update_rate_property_ = new FloatProperty( "Update Interval", 0,
                                               "Interval at which to update the links, in seconds. "
                                               " 0 means to update every update cycle.",
                                               this );
    update_rate_property_->setMin( 0 );

    alpha_property_ = new FloatProperty( "Alpha", 1,
                                         "Amount of transparency to apply to the links.",
                                         this, SLOT( updateAlpha() ));
    alpha_property_->setMin( 0.0 );
    alpha_property_->setMax( 1.0 );

    robot_description_property_ = new StringProperty( "Robot Description", "robot_description",
                                                      "Name of the parameter to search for to load the robot description.",
                                                      this, SLOT( updateRobotDescription() ));

    tf_prefix_property_ = new StringProperty( "TF Prefix", "",
                                              "Robot Model normally assumes the link name is the same as the tf frame name. "
                                              " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
                                              this, SLOT( updateTfPrefix() ));

    color_property_ = new ColorProperty( "Color", QColor( 127, 127, 127 ),
                                         "Color to draw the path.", this, SLOT( updateColor() ) );
}

RobotDisplayCustom::~RobotDisplayCustom()
{
    delete robot_;
}

RobotCustom* RobotDisplayCustom::getRobotCustom()
{
    return robot_;
}

void RobotDisplayCustom::onInitialize()
{
    robot_ = new RobotCustom( scene_node_, context_, "Robot: " + getName().toStdString(), this );

    updateVisualVisible();
    updateCollisionVisible();
    updateAlpha();
    updateColor();

    // subscribe to the topic to load all templates
    link_color_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSLinkColor>( "/link_color", 5, &RobotDisplayCustom::processLinkColorChange, this );
}

void RobotDisplayCustom::updateAlpha()
{
    //ROS_ERROR("UPDATING ALPHA TO %f",alpha_property_->getFloat());
    robot_->setAlpha( alpha_property_->getFloat() );
    context_->queueRender();
}

void RobotDisplayCustom::updateColor()
{
    robot_->setRobotColor( color_property_->getColor() );
    context_->queueRender();
}

void RobotDisplayCustom::updateRobotDescription()
{
    if( isEnabled() )
    {
        load();
        context_->queueRender();
    }
}

void RobotDisplayCustom::updateVisualVisible()
{
    robot_->setVisualVisible( visual_enabled_property_->getValue().toBool() );
    context_->queueRender();
}

void RobotDisplayCustom::updateCollisionVisible()
{
    robot_->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
    context_->queueRender();
}

void RobotDisplayCustom::updateTfPrefix()
{
    clearStatuses();
    context_->queueRender();
}

void RobotDisplayCustom::load()
{
    std::string content;
    if( !update_nh_.getParam( robot_description_property_->getStdString(), content ))
    {
        std::string loc;
        if( update_nh_.searchParam( robot_description_property_->getStdString(), loc ))
        {
            update_nh_.getParam( loc, content );
        }
        else
        {
            clear();
            setStatus( StatusProperty::Error, "URDF",
                       "Parameter [" + robot_description_property_->getString()
                       + "] does not exist, and was not found by searchParam()" );
            return;
        }
    }

    if( content.empty() )
    {
        clear();
        setStatus( StatusProperty::Error, "URDF", "URDF is empty" );
        return;
    }

    if( content == robot_description_ )
    {
        return;
    }

    robot_description_ = content;

    TiXmlDocument doc;
    doc.Parse( robot_description_.c_str() );
    if( !doc.RootElement() )
    {
        clear();
        setStatus( StatusProperty::Error, "URDF", "URDF failed XML parse" );
        return;
    }


    if( !descr.initXml( doc.RootElement() ))
    {
        clear();
        setStatus( StatusProperty::Error, "URDF", "URDF failed Model parse" );
        return;
    }

    //setModelPrefix("simulation", descr);

    setStatus( StatusProperty::Ok, "URDF", "URDF parsed OK" );
    robot_->load( descr );
    robot_->update( TFLinkUpdaterCustom( context_->getFrameManager(),
                                         boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                         tf_prefix_property_->getStdString() ));
}

void RobotDisplayCustom::onEnable()
{
    load();
    updateColor();
    updateAlpha();
    robot_->setVisible( true );
}

void RobotDisplayCustom::onDisable()
{
    robot_->setVisible( false );
    clear();
}

void RobotDisplayCustom::update( float wall_dt, float ros_dt )
{
    //updateColor();
    updateAlpha();

    time_since_last_transform_ += wall_dt;
    float rate = update_rate_property_->getFloat();
    bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

    if( has_new_transforms_ || update )
    {
        robot_->update( TFLinkUpdaterCustom( context_->getFrameManager(),
                                             boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                             tf_prefix_property_->getStdString() ));
        context_->queueRender();

        has_new_transforms_ = false;
        time_since_last_transform_ = 0.0f;
    }
}

void RobotDisplayCustom::fixedFrameChanged()
{
    has_new_transforms_ = true;
}

void RobotDisplayCustom::clear()
{
    robot_->clear();
    clearStatuses();
    robot_description_.clear();
}

void RobotDisplayCustom::reset()
{
    Display::reset();
    has_new_transforms_ = true;
}

void RobotDisplayCustom::setModelPrefix(std::string prefix, urdf::ModelInterface &descr)
{
    //std::cout << "1" << std::endl;
    // first deal with links
    std::map<std::string, boost::shared_ptr<urdf::Link> > links = descr.links_;

    std::map<std::string, boost::shared_ptr<urdf::Link> > new_links;

    // modify key values - I need to add a constant value to each key
    for (std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator mi=links.begin(); mi != links.end(); ++mi)
    {
        std::string new_name = prefix+std::string("/")+mi->first;
        new_links[new_name] = mi->second;
        new_links[new_name]->name = new_name;

        for (int i = 0; i < new_links[new_name]->child_joints.size(); i++)
        {
            std::string new_joint_name = prefix+std::string("/")+new_links[new_name]->child_joints[i]->name;
            new_links[new_name]->child_joints[i]->name = new_joint_name;
            new_links[new_name]->child_joints[i]->parent_link_name = new_name;
        }
        //std::cout << "2" << std::endl;
        if(new_links[new_name] != NULL && new_links[new_name]->parent_joint != NULL)
        {
            std::string new_joint_name = prefix+std::string("/")+new_links[new_name]->parent_joint->name;
            new_links[new_name]->parent_joint->name = new_joint_name;
            new_links[new_name]->parent_joint->child_link_name = new_name;
        }
    }

    descr.links_ = new_links;
    //std::cout << "5" << std::endl;

    // then with joints
    std::map<std::string, boost::shared_ptr<urdf::Joint> > joints = descr.joints_;

    std::map<std::string, boost::shared_ptr<urdf::Joint> > new_joints;

    // modify key values - I need to add a constant value to each key
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator mi=joints.begin(); mi != joints.end(); ++mi)
    {
        std::string new_name = prefix+std::string("/")+mi->first;
        new_joints[new_name] = mi->second;
        new_joints[new_name]->name = new_name;
    }

    descr.joints_ = new_joints;
}

void RobotDisplayCustom::processLinkColorChange(const vigir_ocs_msgs::OCSLinkColor::ConstPtr& color)
{
    std::string link_name = color->link;
    //std::cout << "initial link name: " << link_name << std::endl;
    if(tf_prefix_property_->getStdString().compare("") != 0)
    {
        std::size_t found = link_name.find(tf_prefix_property_->getStdString()); // look for tf_prefix
        if(found != std::string::npos && found < 2)
        {
            //std::cout << "initial link name: " << link_name << std::endl;
            link_name = link_name.substr(found+tf_prefix_property_->getStdString().size(),link_name.size()); // remove prefix from name
            //std::cout << "no tf link name: " << link_name << std::endl;
            boost::erase_all(link_name, "/");
            //std::cout << "final link name: " << link_name << std::endl;
            robot_->setLinkColor(link_name,QColor(color->r,color->g,color->b));
        }
    }
    else
    {
        robot_->setLinkColor(link_name,QColor(color->r,color->g,color->b));
    }
}

std::string RobotDisplayCustom::getChildLinkName(std::string joint)
{
    return descr.getJoint(joint)->child_link_name;
}

} // namespace rviz

