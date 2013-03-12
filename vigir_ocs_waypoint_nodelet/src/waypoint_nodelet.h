#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <flor_ocs_msgs/OCSWaypointAdd.h>
#include <flor_ocs_msgs/OCSWaypointRemove.h>
#include <flor_ocs_msgs/OCSWaypointUpdate.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace ocs_waypoint
{
    class WaypointNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addWaypointCb(const flor_ocs_msgs::OCSWaypointAdd::ConstPtr& msg);
        void removeWaypointCb(const flor_ocs_msgs::OCSWaypointRemove::ConstPtr& msg);
        void updateWaypointCb(const flor_ocs_msgs::OCSWaypointUpdate::ConstPtr& msg);
        void publishWaypointList();

      protected:
        ros::Subscriber waypoint_update_sub_;
        ros::Subscriber waypoint_add_sub_;
        ros::Subscriber waypoint_remove_sub_;
        ros::Publisher waypoint_list_pub_;

        ros::Timer image_publish_timer_;

      private:
        nav_msgs::Path pose_list_;
    };
}
