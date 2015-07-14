/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
#include "waypoint_nodelet.h"

namespace ocs_waypoint
{
void WaypointNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh_out(nh, "waypoint");

    // create publishers for visualization
    waypoint_list_pub_          = nh_out.advertise<nav_msgs::Path>( "list", 1, false );
    waypoint_achieved_list_pub_ = nh_out.advertise<nav_msgs::Path>( "achieved_list", 1, false );
    navigation_list_pub_        = nh_out.advertise<nav_msgs::Path>( "navigation_list", 1, false );
    // subscribe to be able to manipulate waypoints lists
    waypoint_add_sub_       = nh_out.subscribe( "add", 1, &WaypointNodelet::addWaypointCb, this );
    waypoint_remove_sub_    = nh_out.subscribe( "remove", 1, &WaypointNodelet::removeWaypointCb, this );
    waypoint_update_sub_    = nh_out.subscribe( "update", 1, &WaypointNodelet::updateWaypointCb, this );
    waypoint_achieved_sub_  = nh_out.subscribe( "achieved", 1, &WaypointNodelet::waypointAchievedCb, this );
    confirm_navigation_sub_ = nh_out.subscribe( "confirm", 1, &WaypointNodelet::confirmNavigationCb, this );
}

void WaypointNodelet::addWaypointCb(const vigir_ocs_msgs::OCSWaypointAdd::ConstPtr msg)
{
    std::cout << "Adding waypoint to list" << std::endl;
    waypoint_list_.poses.push_back(msg->pose);
    this->publishWaypointList();
}

void WaypointNodelet::removeWaypointCb(const vigir_ocs_msgs::OCSWaypointRemove::ConstPtr msg)
{
    std::cout << "Removing waypoint from list" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < waypoint_list_.poses.size())
        waypoint_list_.poses.erase(waypoint_list_.poses.begin()+msg->waypoint_id);
    this->publishWaypointList();
}

void WaypointNodelet::updateWaypointCb(const vigir_ocs_msgs::OCSWaypointUpdate::ConstPtr msg)
{
    std::cout << "Updating waypoint" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < waypoint_list_.poses.size())
        waypoint_list_.poses[msg->waypoint_id] = msg->pose;
    this->publishWaypointList();
}

void WaypointNodelet::waypointAchievedCb(const vigir_ocs_msgs::OCSWaypointRemove::ConstPtr msg)
{
    std::cout << "Waypoint achieved from list" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < waypoint_list_.poses.size())
    {
        waypoint_achieved_list_.poses.insert(waypoint_achieved_list_.poses.begin(),waypoint_list_.poses[msg->waypoint_id]);
        waypoint_list_.poses.erase(waypoint_list_.poses.begin()+msg->waypoint_id);
    }
    this->publishWaypointList();
    this->publishWaypointAchievedList();
}

void WaypointNodelet::confirmNavigationCb(const vigir_ocs_msgs::OCSWaypointUpdate::ConstPtr msg)
{
    waypoint_list_.header.frame_id = "/world";
    waypoint_list_.header.stamp = ros::Time::now();
    // publish complete list of templates and poses
    navigation_list_pub_.publish( waypoint_list_ );
}

void WaypointNodelet::publishWaypointList()
{
    waypoint_list_.header.frame_id = "/world";
    waypoint_list_.header.stamp = ros::Time::now();
    // publish complete list of templates and poses
    waypoint_list_pub_.publish( waypoint_list_ );
}

void WaypointNodelet::publishWaypointAchievedList()
{
    waypoint_achieved_list_.header.frame_id = "/world";
    waypoint_achieved_list_.header.stamp = ros::Time::now();
    // publish complete list of templates and poses
    waypoint_achieved_list_pub_.publish( waypoint_achieved_list_ );
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_waypoint_nodelet, WaypointNodelet, ocs_waypoint::WaypointNodelet, nodelet::Nodelet);
