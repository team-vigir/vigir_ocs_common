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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <vigir_ocs_msgs/OCSWaypointAdd.h>
#include <vigir_ocs_msgs/OCSWaypointRemove.h>
#include <vigir_ocs_msgs/OCSWaypointUpdate.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace ocs_waypoint
{
    class WaypointNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addWaypointCb(const vigir_ocs_msgs::OCSWaypointAdd::ConstPtr msg);
        void removeWaypointCb(const vigir_ocs_msgs::OCSWaypointRemove::ConstPtr msg);
        void updateWaypointCb(const vigir_ocs_msgs::OCSWaypointUpdate::ConstPtr msg);
        void waypointAchievedCb(const vigir_ocs_msgs::OCSWaypointRemove::ConstPtr msg);
        void confirmNavigationCb(const vigir_ocs_msgs::OCSWaypointUpdate::ConstPtr msg);
        void publishWaypointList();
        void publishWaypointAchievedList();

      protected:
        ros::Subscriber waypoint_update_sub_;
        ros::Subscriber waypoint_add_sub_;
        ros::Subscriber waypoint_remove_sub_;
        ros::Subscriber waypoint_achieved_sub_;
        ros::Subscriber confirm_navigation_sub_;
        ros::Publisher waypoint_list_pub_;
        ros::Publisher waypoint_achieved_list_pub_;
        ros::Publisher navigation_list_pub_;

        ros::Timer image_publish_timer_;

      private:
        nav_msgs::Path waypoint_list_;
        nav_msgs::Path waypoint_achieved_list_;
    };
}
