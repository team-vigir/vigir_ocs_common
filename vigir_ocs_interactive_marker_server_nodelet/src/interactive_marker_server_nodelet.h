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

#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <flor_ocs_msgs/OCSControlMode.h>
#include <flor_ocs_msgs/OCSObjectSelection.h>
#include <flor_ocs_msgs/OCSSelectedObjectUpdate.h>
#include <flor_ocs_msgs/OCSMarkerVisibility.h>

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

        void addInteractiveMarker( const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr msg );
        void removeInteractiveMarker( const std_msgs::String::ConstPtr msg );
        void updatePose(const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr msg );
        void onMarkerFeedback(unsigned char event_type, std::string topic_name, geometry_msgs::PoseStamped pose, std::string client_id );
        void setMode(const flor_ocs_msgs::OCSControlMode::ConstPtr msg);
        void processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr obj);
        //void processMarkerVisibility(const flor_ocs_msgs::OCSMarkerVisibility::ConstPtr msg);

      private:
        ros::NodeHandle nh;

        void publishSelectedObject();

        ros::Publisher interactive_marker_server_feedback_pub_;
        ros::Subscriber interactive_marker_server_add_sub_;
        ros::Subscriber interactive_marker_server_remove_sub_;
        ros::Subscriber interactive_marker_server_update_sub_;
        ros::Subscriber interactive_marker_server_mode_sub_;
        ros::Subscriber interactive_marker_server_visibility_sub_;
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
