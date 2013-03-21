#include "waypoint_following.h"
#include <math.h>
#include <flor_ocs_msgs/OCSWaypointRemove.h>
#include <geometry_msgs/Point.h>

namespace ocs_waypoint
{
void WaypointFollowingBasic::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh_out(nh, "waypoint");
    maxSpeed = 4;
    destWaypoint = -1;
    timer = nh.createTimer(ros::Duration(0.025),&WaypointFollowingBasic::moveRobot,this,false,false);

    // also create a publisher to set parameters of cropped image
      //waypoint_list_pub_   = nh_out.advertise<nav_msgs::Path>( "list", 1, false );
    // then, subscribe to the resulting cropped image
    waypoint_update    = nh_out.subscribe<nav_msgs::Path>( "list", 1, &WaypointFollowingBasic::recievedUpdateWaypointMessage, this );
    robot_loc = nh.subscribe<sensor_msgs::Imu>( "/atlas/imu", 1, &WaypointFollowingBasic::recievedRobotLocUpdate, this );
    ros::Publisher get_list = nh_out.advertise<flor_ocs_msgs::OCSWaypointRemove>("remove", 1,this);
    flor_ocs_msgs::OCSWaypointRemove emptyMsg;
    emptyMsg.waypoint_id = -1;
    get_list.publish(emptyMsg);
}


void WaypointFollowingBasic::recievedUpdateWaypointMessage( const nav_msgs::Path::ConstPtr& msg)
{
    if(destWaypoint > -1)
    {
        waypoints = *msg;
        if(waypoints.poses.size() > 0)
        {
            destWaypoint = findClosestWaypoint(50);
        }
    }
    else
    {
        timer.stop();
    }


}
int WaypointFollowingBasic::findClosestWaypoint(int maxDist)
{
    double minDist=-1;
    int closest= 0;
    for(int i=0;i<waypoints.poses.size();i++)
    {
        //float dist = sqrt(pow((robotX-(geometry_msgs::Point)(waypoints.poses[i]).x),2)+pow((robotY-(geometry_msgs::Point)(waypoints.poses[i]).y),2));
        //if (minDist == -1 || dist < minDist)
        //{
        //   minDist = dist;
        //    closest = i;
        //}
    }
    if(minDist <= maxDist)
        return closest;
    return 0;
}

void WaypointFollowingBasic::recievedRobotLocUpdate( const sensor_msgs::Imu::ConstPtr& msg)
{
    if(atWaypoint())
        destWaypoint++;
    robotX = msg->orientation.x;
    robotY = msg->orientation.y;
    robotW = msg->orientation.w;
    //if(destWaypoint > waypoints.poses.size())
        //waitFor
}
bool WaypointFollowingBasic::atWaypoint()
{
    //float dist = sqrt(pow((robotX-(geometry_msgs::Point)(waypoints.poses[destWaypoint]).x),2)+pow((robotY-(geometry_msgs::Point)(waypoints.poses[destWaypoint]).y),2));
    //if(dist <= 2.5)
    //    return true;
    return false;
}

void WaypointFollowingBasic::moveRobot(const ros::TimerEvent& event)
{
    std::cout << "Moving Robot..." << std::endl;


}

//void WaypointFollowingBasic::addWaypointCb(const flor_ocs_msgs::OCSWaypointAdd::ConstPtr& msg)
//{
//    std::cout << "Adding waypoint to list" << std::endl;
//    pose_list_.poses.push_back(msg->pose);
//    this->publishWaypointList();
//}

//void WaypointFollowingBasic::removeWaypointCb(const flor_ocs_msgs::OCSWaypointRemove::ConstPtr& msg)
//{
//    std::cout << "Removing waypoint from list" << std::endl;
//    if(msg->waypoint_id >= 0 && msg->waypoint_id < pose_list_.poses.size())
//        pose_list_.poses.erase(pose_list_.poses.begin()+msg->waypoint_id);
//    this->publishWaypointList();
//}

//void WaypointFollowingBasic::updateWaypointCb(const flor_ocs_msgs::OCSWaypointUpdate::ConstPtr& msg)
//{
//    std::cout << "Updating waypoint" << std::endl;
//    if(msg->waypoint_id >= 0 && msg->waypoint_id < pose_list_.poses.size())
//        pose_list_.poses[msg->waypoint_id] = msg->pose;
//    this->publishWaypointList();
//}

//void WaypointFollowingBasic::publishWaypointList()
//{
//	pose_list_.header.frame_id = "/pelvis";
//    // publish complete list of templates and poses
//    waypoint_list_pub_.publish( pose_list_ );
//}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_waypoint_following_basic, WaypointFollowing, ocs_waypoint::WaypointFollowingBasic, nodelet::Nodelet);
