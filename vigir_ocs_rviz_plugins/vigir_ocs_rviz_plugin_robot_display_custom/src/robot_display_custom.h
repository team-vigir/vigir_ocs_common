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

#ifndef RVIZ_ROBOT_MODEL_DISPLAY_CUSTOM_H
#define RVIZ_ROBOT_MODEL_DISPLAY_CUSTOM_H

#include "rviz/display.h"
#include "rviz/robot/link_updater.h"

#include <OGRE/OgreVector3.h>

#include <vigir_ocs_msgs/OCSLinkColor.h>

#include <map>

#include <urdf/model.h>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz
{
class Axes;
}

namespace urdf
{
class ModelInterface;
}

namespace rviz
{

class FloatProperty;
class Property;
class RobotCustom;
class StringProperty;
class ColorProperty;

/**
 * \class RobotDisplayCustom
 * \brief Uses a robot xml description to display the pieces of a robot at the transforms broadcast by rosTF
 */
class RobotDisplayCustom: public Display
{
    Q_OBJECT
public:
    RobotDisplayCustom();
    virtual ~RobotDisplayCustom();
    RobotCustom* getRobotCustom();
    // Overrides from Display
    virtual void onInitialize();
    virtual void update( float wall_dt, float ros_dt );
    virtual void fixedFrameChanged();
    virtual void reset();

    void clear();

    void setModelPrefix(std::string prefix, urdf::ModelInterface &descr);

    void processLinkColorChange(const vigir_ocs_msgs::OCSLinkColor::ConstPtr& color);

    std::string getChildLinkName(std::string joint);

private Q_SLOTS:
    void updateVisualVisible();
    void updateCollisionVisible();
    void updateTfPrefix();
    void updateAlpha();
    void updateColor();
    void updateRobotDescription();

protected:
    /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
    virtual void load();

    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    RobotCustom* robot_;                 ///< Handles actually drawing the robot

    bool has_new_transforms_;      ///< Callback sets this to tell our update function it needs to update the transforms

    float time_since_last_transform_;

    std::string robot_description_;

    ColorProperty* color_property_;
    Property* visual_enabled_property_;
    Property* collision_enabled_property_;
    FloatProperty* update_rate_property_;
    StringProperty* robot_description_property_;
    FloatProperty* alpha_property_;
    StringProperty* tf_prefix_property_;

    ros::NodeHandle nh_;
    ros::Subscriber link_color_sub_;

    urdf::Model descr;
};

} // namespace rviz

#endif

