#include <ros/ros.h>
#include <ros/message_event.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>

#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>

#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>

namespace ocs_interactive_marker_server
{
    class InteractiveMarkerServerNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addInteractiveMarker( const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr& msg );
        void removeInteractiveMarker( const std_msgs::String::ConstPtr& msg );
        void updatePose( const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr& msg );
        void onMarkerFeedback( std::string topic_name, geometry_msgs::PoseStamped pose );

      private:
        ros::NodeHandle nh_;

        ros::Subscriber interactive_marker_server_add_sub_;
        ros::Subscriber interactive_marker_server_remove_sub_;
        ros::Subscriber interactive_marker_server_update_sub_;
        ros::Publisher interactive_marker_server_feedback_pub_;

        std::map<std::string,InteractiveMarkerServerCustom*> marker_map_;

        boost::mutex interactive_marker_server_change_mutex_;
    };
}
