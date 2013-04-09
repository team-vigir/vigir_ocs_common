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
    waypoint_add_sub_       = nh_out.subscribe<flor_ocs_msgs::OCSWaypointAdd>( "add", 1, &WaypointNodelet::addWaypointCb, this );
    waypoint_remove_sub_    = nh_out.subscribe<flor_ocs_msgs::OCSWaypointRemove>( "remove", 1, &WaypointNodelet::removeWaypointCb, this );
    waypoint_update_sub_    = nh_out.subscribe<flor_ocs_msgs::OCSWaypointUpdate>( "update", 1, &WaypointNodelet::updateWaypointCb, this );
    waypoint_achieved_sub_  = nh_out.subscribe<flor_ocs_msgs::OCSWaypointRemove>( "achieved", 1, &WaypointNodelet::waypointAchievedCb, this );
    confirm_navigation_sub_ = nh_out.subscribe<flor_ocs_msgs::OCSWaypointUpdate>( "confirm", 1, &WaypointNodelet::confirmNavigationCb, this );
}

void WaypointNodelet::addWaypointCb(const flor_ocs_msgs::OCSWaypointAdd::ConstPtr& msg)
{
    std::cout << "Adding waypoint to list" << std::endl;
    waypoint_list_.poses.push_back(msg->pose);
    this->publishWaypointList();
}

void WaypointNodelet::removeWaypointCb(const flor_ocs_msgs::OCSWaypointRemove::ConstPtr& msg)
{
    std::cout << "Removing waypoint from list" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < waypoint_list_.poses.size())
        waypoint_list_.poses.erase(waypoint_list_.poses.begin()+msg->waypoint_id);
    this->publishWaypointList();
}

void WaypointNodelet::updateWaypointCb(const flor_ocs_msgs::OCSWaypointUpdate::ConstPtr& msg)
{
    std::cout << "Updating waypoint" << std::endl;
    if(msg->waypoint_id >= 0 && msg->waypoint_id < waypoint_list_.poses.size())
        waypoint_list_.poses[msg->waypoint_id] = msg->pose;
    this->publishWaypointList();
}

void WaypointNodelet::waypointAchievedCb(const flor_ocs_msgs::OCSWaypointRemove::ConstPtr& msg)
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

void WaypointNodelet::confirmNavigationCb(const flor_ocs_msgs::OCSWaypointUpdate::ConstPtr& msg)
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
