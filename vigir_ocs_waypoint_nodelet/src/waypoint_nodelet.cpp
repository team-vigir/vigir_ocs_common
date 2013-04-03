#include "waypoint_nodelet.h"

namespace ocs_waypoint
{
void WaypointNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh_out(nh, "waypoint");

    // also create a publisher to set parameters of cropped image
    waypoint_list_pub_   = nh_out.advertise<nav_msgs::Path>( "list", 1, false );
    // then, subscribe to the resulting cropped image
    waypoint_add_sub_    = nh_out.subscribe<flor_ocs_msgs::OCSWaypointAdd>( "add", 1, &WaypointNodelet::addWaypointCb, this );
    waypoint_remove_sub_ = nh_out.subscribe<flor_ocs_msgs::OCSWaypointRemove>( "remove", 1, &WaypointNodelet::removeWaypointCb, this );
    waypoint_update_sub_ = nh_out.subscribe<flor_ocs_msgs::OCSWaypointUpdate>( "update", 1, &WaypointNodelet::updateWaypointCb, this );
}

void WaypointNodelet::addWaypointCb(const flor_ocs_msgs::OCSWaypointAdd::ConstPtr& msg)
{
    std::cout << "Adding waypoint to list" << std::endl;
    pose_list_.poses.push_back(msg->pose);
    this->publishWaypointList();
}

void WaypointNodelet::removeWaypointCb(const flor_ocs_msgs::OCSWaypointRemove::ConstPtr& msg)
{
    std::cout << "Removing waypoint from list" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < pose_list_.poses.size())
        pose_list_.poses.erase(pose_list_.poses.begin()+msg->waypoint_id);
    this->publishWaypointList();
}

void WaypointNodelet::updateWaypointCb(const flor_ocs_msgs::OCSWaypointUpdate::ConstPtr& msg)
{
    std::cout << "Updating waypoint" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < pose_list_.poses.size())
        pose_list_.poses[msg->waypoint_id] = msg->pose;
    this->publishWaypointList();
}

void WaypointNodelet::publishWaypointList()
{
    pose_list_.header.frame_id = "/world";
    pose_list_.header.stamp = ros::Time::now();
    // publish complete list of templates and poses
    waypoint_list_pub_.publish( pose_list_ );
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_waypoint_nodelet, WaypointNodelet, ocs_waypoint::WaypointNodelet, nodelet::Nodelet);
