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

#ifndef RVIZ_ROBOT_MODEL_DISPLAY_H
#define RVIZ_ROBOT_MODEL_DISPLAY_H

#include "rviz/display.h"

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>

#include <map>
#include <vector>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz
{
class Axes;
}

namespace rviz
{

class FloatProperty;
class Property;
class Robot;
class StringProperty;
class VisualizationManager;

/**
 * \class TemplateDisplayCustom
 * \brief Uses a robot xml description to display the pieces of a robot at the transforms broadcast by rosTF
 */
class TemplateDisplayCustom: public Display
{
    Q_OBJECT
public:
    TemplateDisplayCustom();
    virtual ~TemplateDisplayCustom();

    // Overrides from Display
    virtual void onInitialize();
    virtual void update( float wall_dt, float ros_dt );
    virtual void fixedFrameChanged();
    virtual void reset();

    void clear();

    void processPoseChange(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& pose);
    void processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg);
    void publishTemplateUpdate(const unsigned char& id, const geometry_msgs::PoseStamped& pose);
    void processTemplateRemove(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg);

    void setVisualizationManager(rviz::VisualizationManager* manager) { vis_manager_ = manager; };

    void onMarkerFeedback( const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

public Q_SLOTS:
    void enableTemplateMarker( int i, bool enable );
    void enableTemplateMarkers( bool );

private Q_SLOTS:
    void updateVisualVisible();
    void updateCollisionVisible();
    void updateTfPrefix();
    void updateAlpha();
    void updateRobotDescription();

protected:
    /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
    virtual void load();

    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    bool has_new_transforms_;      ///< Callback sets this to tell our update function it needs to update the transforms

    float time_since_last_transform_;

    std::string robot_description_;

    Property* visual_enabled_property_;
    Property* collision_enabled_property_;
    FloatProperty* update_rate_property_;
    StringProperty* robot_description_property_;
    FloatProperty* alpha_property_;
    StringProperty* tf_prefix_property_;

private:
    void addTemplate(int index, std::string path, Ogre::Vector3 pos, Ogre::Quaternion quat);
    void addTemplateMarker(std::string label, unsigned char id, Ogre::Vector3 pos);

    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> template_pose_sub_list_;
    ros::Subscriber template_list_sub_;
    ros::Subscriber template_remove_sub_;
    ros::Publisher template_update_pub_;
    std::vector<ros::Publisher> template_pose_pub_list_;
    ros::Publisher interactive_marker_add_pub_;
    ros::Publisher interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;
    ros::Publisher interactive_marker_remove_pub_;

    std::vector<unsigned char> template_id_list_;
    std::vector<std::string> template_list_;
    std::vector<Ogre::SceneNode*> template_node_list_;

    static std::vector<InteractiveMarkerServerCustom*> template_marker_list_;
    std::vector<rviz::Display*> display_template_marker_list_;
    //InteractiveMarkerServerCustom *template_marker_;

    rviz::VisualizationManager* vis_manager_;

};

} // namespace rviz

#endif


