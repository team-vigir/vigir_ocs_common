/*
 * TemplateDisplayCustom class implementation.
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

#include <urdf/model.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/visualization_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"

#include "template_display_custom.h"

#include <ros/package.h>

namespace rviz
{

// initialize static class member
std::vector<InteractiveMarkerServerCustom*> TemplateDisplayCustom::template_marker_list_;

void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                TemplateDisplayCustom* display )
{
    display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

TemplateDisplayCustom::TemplateDisplayCustom()
    : Display()
    , has_new_transforms_( false )
    , time_since_last_transform_( 0.0f )
    //, lNode_(NULL)
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
}

TemplateDisplayCustom::~TemplateDisplayCustom()
{
    //delete robot_;
}

void TemplateDisplayCustom::onInitialize()
{
    //robot_ = new Robot( scene_node_, context_, "Robot: " + getName().toStdString(), this );
    updateVisualVisible();
    updateCollisionVisible();
    updateAlpha();
}

void TemplateDisplayCustom::updateAlpha()
{
    //robot_->setAlpha( alpha_property_->getFloat() );
    context_->queueRender();
}

void TemplateDisplayCustom::updateRobotDescription()
{
    if( isEnabled() )
    {
        load();
        context_->queueRender();
    }
}

void TemplateDisplayCustom::updateVisualVisible()
{
    //robot_->setVisualVisible( visual_enabled_property_->getValue().toBool() );
    context_->queueRender();
}

void TemplateDisplayCustom::updateCollisionVisible()
{
    //robot_->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
    context_->queueRender();
}

void TemplateDisplayCustom::updateTfPrefix()
{
    clearStatuses();
    context_->queueRender();
}

void TemplateDisplayCustom::load()
{

    // I add an ambient color. The ambient color is managed in the scenemanager.
    // If you want to learn more about ambient/specular/diffuse color, check the 'basic material tutorial'
    // from this serie.
    Ogre::ColourValue ambientColour(0.2f, 0.2f, 0.2f, 1.0f);
    this->scene_manager_->setAmbientLight(ambientColour);

    // create a new resource group for all our templates
    Ogre::String nameOfResourceGroup = "templates";
    Ogre::ResourceGroupManager& resourceManager = Ogre::ResourceGroupManager::getSingleton();
    if(!resourceManager.resourceGroupExists(nameOfResourceGroup))
    {
        resourceManager.createResourceGroup(nameOfResourceGroup);

        // Use "templates" package.  @TODO make this a parameter
        std::string template_path = ros::package::getPath("templates")+"/";//vigir_grasp_control") + "/../templates/";
        ROS_INFO("  Reading templates from <%s>", template_path.c_str());
        //template_dir_path_ = QString(template_path.c_str());
        Ogre::String templatePath = template_path;


        //Ogre::String templatePath = "/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/";
        resourceManager.addResourceLocation(templatePath, "FileSystem", nameOfResourceGroup, false);

        // parse scripts
        resourceManager.initialiseResourceGroup(nameOfResourceGroup);
    }
    // loads files into our resource manager
    resourceManager.loadResourceGroup(nameOfResourceGroup);

    // Now the loaded Mesh is available from its ResourceGroup,
    // as well as from the Ogre::MeshManager. A shared pointer to
    // it can be accessed by : Ogre::MeshManager::getSingleton().getByName(name_of_the_mesh);

    /*// create entity for mesh and attach it to the scene node
    Ogre::String lNameOfTheMesh = "vehicle/steering_wheel_mm.mesh";
    Ogre::Entity* lEntity = this->scene_manager_->createEntity(lNameOfTheMesh);
    lNode_ = this->scene_node_->createChildSceneNode();
    lNode_->attachObject(lEntity);
    // change position and scale (from mm to m)
    lNode_->setPosition(1.0f, 0.0f, 0.0f);
    //lNode_->scale(0.001f,0.001f,0.001f);
    // The loaded mesh will be white. This is normal.*/

    // subscribe to the topic to load all templates
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>( "/template/list", 5, &TemplateDisplayCustom::processTemplateList, this );

    // subscribe to the topic to load all templates
    template_remove_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 5, &TemplateDisplayCustom::processTemplateRemove, this );

    // and advertise the template update to update the manipulator
    template_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "/template/update", 1, false );

    // advertise/subscribe to the interactive marker topics
    interactive_marker_add_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 1, true );
    interactive_marker_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 1, false );
    interactive_marker_feedback_sub_ = nh_.subscribe<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/feedback", 5, &TemplateDisplayCustom::onMarkerFeedback, this );;
    interactive_marker_remove_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 1, false );

}

void TemplateDisplayCustom::onEnable()
{
    load();
    for(int i = 0; i < template_node_list_.size(); i++)
    {
        template_node_list_[i]->setVisible(true);
    }
}

void TemplateDisplayCustom::onDisable()
{
    for(int i = 0; i < template_node_list_.size(); i++)
    {
        template_node_list_[i]->setVisible(false);
    }

    clear();
}

void TemplateDisplayCustom::update( float wall_dt, float ros_dt )
{
    time_since_last_transform_ += wall_dt;
    float rate = update_rate_property_->getFloat();
    bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

    //std::cout << "update" << std::endl;

    context_->queueRender();
}

void TemplateDisplayCustom::fixedFrameChanged()
{
    has_new_transforms_ = true;
}

void TemplateDisplayCustom::clear()
{
    //robot_->clear();
    clearStatuses();
    robot_description_.clear();
}

void TemplateDisplayCustom::reset()
{
    Display::reset();
    has_new_transforms_ = true;
}

void TemplateDisplayCustom::enableTemplateMarkers( bool enable )
{
    //ROS_ERROR("Disabling the template markers");
    for(int i = 0; i < display_template_marker_list_.size(); i++)
    {
        display_template_marker_list_[i]->setEnabled( enable );
    }
}

void TemplateDisplayCustom::processPoseChange(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& pose)
{
    std::cout << "Processing pose change" << std::endl;
    //    printf(" Template pose change (%f, %f, %f) quat(%f, %f, %f, %f)\n",
    //             pose->pose.position.x,pose->pose.position.y,pose->pose.position.z,
    //             pose->pose.orientation.w,
    //             pose->pose.orientation.x,
    //             pose->pose.orientation.y,
    //             pose->pose.orientation.z );

    /*template_node_list_[id]->setPosition(pose->pose.position.x,pose->pose.position.y,pose->pose.position.z);
    Ogre::Quaternion quat;
    quat.w= pose->pose.orientation.w;
    quat.x= pose->pose.orientation.x;
    quat.y= pose->pose.orientation.y;
    quat.z= pose->pose.orientation.z;

    template_node_list_[id]->setOrientation(quat);*/

    publishTemplateUpdate(pose->template_id,pose->pose);

    context_->queueRender();
}

void TemplateDisplayCustom::addTemplate(int index, std::string path, Ogre::Vector3 pos, Ogre::Quaternion quat)
{
    std::cout << "Adding Ogre object for template" << std::endl;
    static int counter = 0;
    std::ostringstream convert;
    convert << counter++ << "." << index;
    // create entity for mesh and attach it to the scene node
    Ogre::Entity* lEntity = this->scene_manager_->createEntity(std::string("template ")+convert.str(), path);
    Ogre::SceneNode* lNode = this->scene_node_->createChildSceneNode();
    lNode->attachObject(lEntity);
    // change position and scale (from mm to m)
    lNode->setPosition(pos);
    lNode->setOrientation(quat);
    //lNode->scale(0.001f,0.001f,0.001f); - converting templates to use meters in mesh by default
    // The loaded mesh will be white. This is normal.
    template_node_list_.push_back(lNode);
}

void TemplateDisplayCustom::addTemplateMarker(std::string label, unsigned char id, Ogre::Vector3 pos)
{
    std::cout << "Adding template marker " << id << std::endl;
    std::string template_pose_string = std::string("/template_pose_")+boost::to_string((unsigned int)id); // one for each template

    // Add template marker
    rviz::Display* interactive_marker_template = vis_manager_->createDisplay( "rviz/InteractiveMarkers", (std::string("Interactive marker template ")+boost::to_string((unsigned int)id)).c_str(), true );
    interactive_marker_template->subProp( "Update Topic" )->setValue( (template_pose_string+std::string("/pose_marker/update")).c_str() );
    interactive_marker_template->setEnabled( true );
    display_template_marker_list_.push_back(interactive_marker_template);

    // initialize template interactive marker server if it doesn't exist yet
    //    bool exists = false;
    //    for(int i = 0; i < template_marker_list_.size(); i++)
    //    {
    //        if(template_marker_list_[i]->getTopicName().compare(template_pose_string) == 0)
    //        {
    //            exists = true;
    //            break;
    //        }
    //    }

    //    std::cout << "Need to create template marker server? " << !exists << std::endl;

    //    if(!exists)
    //    {
    //        geometry_msgs::Point point;
    //        point.x = pos.x;
    //        point.y = pos.y;
    //        point.z = pos.z;
    //        InteractiveMarkerServerCustom* template_marker_ = new InteractiveMarkerServerCustom(template_pose_string, template_pose_string, fixed_frame_.toUtf8().constData(), 0.2, point);
    //        template_marker_->onFeedback = boost::bind(&TemplateDisplayCustom::onMarkerFeedback, this, _1, _2);
    //        template_pose_pub_list_.push_back(nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>(template_pose_string, 1, false));
    //        template_marker_list_.push_back(template_marker_);
    //    }

    geometry_msgs::Point point;
    point.x = pos.x;
    point.y = pos.y;
    point.z = pos.z;
    flor_ocs_msgs::OCSInteractiveMarkerAdd marker_server_template;
    std::string template_name = label;
    if(template_name.size() > 5 && template_name.substr(template_name.size()-5,5) == ".mesh")
        template_name = template_name.substr(0,template_name.size()-5);
    marker_server_template.name  = std::string("Template ")+boost::to_string((unsigned int)id)+std::string("\n")+template_name;
    marker_server_template.topic = template_pose_string;
    marker_server_template.frame = fixed_frame_.toUtf8().constData();
    marker_server_template.scale = 0.2;
    marker_server_template.point = point;
    interactive_marker_add_pub_.publish(marker_server_template);

    // add the template pose publisher
    template_pose_pub_list_.push_back( nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( template_pose_string, 1, false) );

    // and subscribe to the template marker feedback loop
    ros::Subscriber template_pose_sub = nh_.subscribe<flor_ocs_msgs::OCSTemplateUpdate>( template_pose_string, 5, &TemplateDisplayCustom::processPoseChange, this );
    template_pose_sub_list_.push_back(template_pose_sub);
    std::cout << "subscribed to topic" << std::endl;
}

void TemplateDisplayCustom::onMarkerFeedback(const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr& msg)//std::string topic_name, geometry_msgs::PoseStamped pose)
{
    std::string topic_name = msg->topic;
    for(int i = 0; i < template_pose_pub_list_.size(); i++)
    {
        if(template_pose_pub_list_[i].getTopic() == topic_name)
        {
            flor_ocs_msgs::OCSTemplateUpdate out;
            out.pose = msg->pose;
            out.template_id = atoi(topic_name.erase(0,std::string("/template_pose_").size()).c_str());
            template_pose_pub_list_[i].publish(out);
        }
    }
}

void TemplateDisplayCustom::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
{
    //std::cout << "Processing template list" << std::endl;
    for(int i = 0; i < msg->template_list.size(); i++)
    {
        std::cout << "Template: " << msg->template_list[i] << std::endl;

        geometry_msgs::PoseStamped pose = msg->pose[i];
        transform(fixed_frame_.toUtf8().constData(),pose);

        Ogre::Vector3 pos;
        pos.x = pose.pose.position.x;
        pos.y = pose.pose.position.y;
        pos.z = pose.pose.position.z;
        Ogre::Quaternion quat;
        quat.w= pose.pose.orientation.w;
        quat.x= pose.pose.orientation.x;
        quat.y= pose.pose.orientation.y;
        quat.z= pose.pose.orientation.z;

        // if i is outside boundaries, add to lists
        if(i >= template_list_.size())
        {
            std::string path = msg->template_list[i];
            addTemplate(msg->template_id_list[i],path,pos,quat);
            template_list_.push_back(path);
            template_id_list_.push_back(msg->template_id_list[i]);
            addTemplateMarker(path,msg->template_id_list[i],pos);
        }
        else // just update position
        {
            template_node_list_[i]->setPosition(pos);
            template_node_list_[i]->setOrientation(quat);
            //template_marker_list_[i]->setPose(pose);

            flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
            cmd.topic = template_pose_pub_list_[i].getTopic();
            cmd.pose = pose;
            interactive_marker_update_pub_.publish(cmd);
        }
    }
    //template_list_
    //msg->template_list;
}

void TemplateDisplayCustom::publishTemplateUpdate(const unsigned char& id, const geometry_msgs::PoseStamped& pose)
{
    flor_ocs_msgs::OCSTemplateUpdate cmd;

    cmd.template_id = id;
    cmd.pose = pose;

    transform("/world",cmd.pose);

    // publish complete list of templates and poses
    template_update_pub_.publish( cmd );
}

void TemplateDisplayCustom::processTemplateRemove(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg)
{
    std::cout << "Processing template remove" << std::endl;
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        // remove from ogre
        this->scene_manager_->destroyEntity((Ogre::Entity*)template_node_list_[index]->getAttachedObject(0));
        //template_node_list_[index]->detachObject(0);
        this->scene_node_->removeChild(template_node_list_[index]);
        template_node_list_.erase(template_node_list_.begin()+index);
        // make sure we also remove the marker server
        //delete template_marker_list_[index];
        //template_marker_list_.erase(template_marker_list_.begin()+index);
        std_msgs::String topic;
        topic.data = template_pose_pub_list_[index].getTopic();
        interactive_marker_remove_pub_.publish(topic);
        // and the marker display
        //vis_manager_->removeDisplay(display_template_marker_list_[index]);
        display_template_marker_list_.erase(display_template_marker_list_.begin()+index);
        // and the publisher
        template_pose_pub_list_.erase(template_pose_pub_list_.begin()+index);
        // and finally remove the subscriber
        template_pose_sub_list_.erase(template_pose_sub_list_.begin()+index);
        // remove template id and template pointer
        template_id_list_.erase(template_id_list_.begin()+index);
        template_list_.erase(template_list_.begin()+index);
    }
}

void TemplateDisplayCustom::transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose)
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
