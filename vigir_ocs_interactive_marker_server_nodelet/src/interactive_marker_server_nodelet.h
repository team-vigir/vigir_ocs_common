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
@TODO_ADD_AUTHOR_INFO
#include <ros/ros.h>
#include <ros/message_event.h>
//#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>
#include <boost/asio/ip/host_name.hpp>

#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>

#include <vigir_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <vigir_ocs_msgs/OCSControlMode.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>
#include <vigir_ocs_msgs/OCSSelectedObjectUpdate.h>
//#include <vigir_ocs_msgs/OCSMarkerVisibility.h>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/PoseStamped.h>

namespace ocs_interactive_marker_server
{

    class InteractiveMarkerServerNodelet //: public nodelet::Nodelet
    {
      public:
        //virtual void onInit();
        InteractiveMarkerServerNodelet();

        void addInteractiveMarker( const vigir_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr msg );
        void removeInteractiveMarker( const std_msgs::String::ConstPtr msg );
        void updatePose(const vigir_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr msg );
        void onMarkerFeedback(unsigned char event_type, std::string topic_name, geometry_msgs::PoseStamped pose, std::string client_id );
        void setMode(const vigir_ocs_msgs::OCSControlMode::ConstPtr msg);
        void processObjectSelection(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr obj);
        //void processMarkerVisibility(const vigir_ocs_msgs::OCSMarkerVisibility::ConstPtr msg);

      private:
        ros::NodeHandle nh;

        void publishSelectedObject();

        ros::Publisher interactive_marker_server_feedback_pub_;
        ros::Subscriber interactive_marker_server_add_sub_;
        ros::Subscriber interactive_marker_server_remove_sub_;
        ros::Subscriber interactive_marker_server_update_sub_;
        ros::Subscriber interactive_marker_server_mode_sub_;
        //ros::Subscriber interactive_marker_server_visibility_sub_;
        ros::Subscriber select_object_sub_;
        ros::Publisher selected_object_update_pub_;

        std::map<std::string,boost::shared_ptr<InteractiveMarkerServerCustom> > marker_map_;
        std::map<std::string,geometry_msgs::PoseStamped> pose_map_;
        std::map<std::string,std::string> host_selected_object_topic_map_;

//        boost::recursive_mutex interactive_marker_server_change_mutex_;
//        boost::recursive_mutex interactive_marker_server_publisher_mutex_;

//        boost::posix_time::ptime marker_feedback_timer_;
    };
}
