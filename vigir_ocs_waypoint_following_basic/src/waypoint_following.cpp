#include "waypoint_following.h"
#include <math.h>
#include <flor_ocs_msgs/OCSWaypointRemove.h>
#include <eigen3/Eigen/Geometry>
#include <flor_ocs_msgs/OCSWaypointAdd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

namespace ocs_waypoint
{
void WaypointFollowingBasic::onInit()
{
    std::cout << "Setting up waypoint follower...." << std::endl;
    ros::NodeHandle& nh = getNodeHandle();
    //ros::NodeHandle nh_out(nh, "waypoint");
    maxSpeed = 1.0;
    maxTurn =  1.5;
    oldTurn=0;
    destWaypoint = -1;
    timer = nh.createTimer(ros::Duration(0.025),&WaypointFollowingBasic::moveRobot,this,false,false);

    std::cout << "Timer created..." << std::endl;
    // also create a publisher to set parameters of cropped image
    //waypoint_list_pub_   = nh_out.advertise<nav_msgs::Path>( "list", 1, false );
    drive_pub_ = nh.advertise<geometry_msgs::Twist>("/atlas/cmd_vel",1,false);
    remove_pub_ = nh.advertise<flor_ocs_msgs::OCSWaypointRemove>("/waypoint/achieved", 1, false);
    waypoint_update = nh.subscribe<nav_msgs::Path>( "/waypoint/navigation_list", 1, &WaypointFollowingBasic::recievedUpdateWaypointMessage, this );
    robot_loc = nh.subscribe<nav_msgs::Odometry>( "ground_truth_odom", 1, &WaypointFollowingBasic::recievedRobotLocUpdate, this );
    std::cout << "subscribers created now sending empty message to get initial waypoint list." << std::endl;
    //add.publish(addMsg);
    //emptyMsg.waypoint_id = -1;
    //get_list.publish(emptyMsg);
    std::cout << "Waypoint list requested. Set up complete." << std::endl;
}


void WaypointFollowingBasic::recievedUpdateWaypointMessage( const nav_msgs::Path::ConstPtr& msg)
{
    std::cout << "Recieved waypoint list update message...." << std::endl;
    if(destWaypoint == -1)
    {
        timer.start();
    }
    waypoints = *msg;
    std::cout << "New updated list has " << waypoints.poses.size() << " points" << std::endl;
    for(int i=0;i<waypoints.poses.size(); i++)
        std::cout << "Waypoint " << i << " at (" << waypoints.poses[i].pose.position.x << ", " << waypoints.poses[i].pose.position.y << ")" <<std::endl;
    if(waypoints.poses.size() > 0)
    {
        //destWaypoint = findClosestWaypoint(20);
        destWaypoint = 0;
    }

}
int WaypointFollowingBasic::findClosestWaypoint(int maxDist)
{
    double minDist=-1;
    int closest= 0;
    for(int i=0;i<waypoints.poses.size();i++)
    {
        geometry_msgs::PoseStamped  pnt = waypoints.poses[i];
        float dist = sqrt(pow((robotX-pnt.pose.position.x),2)+pow((robotY-pnt.pose.position.y),2));
        if (minDist == -1 || dist < minDist)
        {
           minDist = dist;
           closest = i;
        }
    }
    std::cout << "closest waypoint found to be " << minDist << " away and waypoint # " << closest << std::endl;
    if(minDist <= maxDist)
        return closest;
    return 0;
}

void WaypointFollowingBasic::recievedRobotLocUpdate( const nav_msgs::Odometry::ConstPtr& msg)
{
    //std::cout << "Recieved robot location message. X = " << msg->pose.pose.position.x<< std::endl;
    if(destWaypoint >= waypoints.poses.size())
        return;
    if(destWaypoint != -1 && atWaypoint())
    {
        destWaypoint++;
        if(destWaypoint < waypoints.poses.size())
            std::cout << "Next waypoint at (" << waypoints.poses[destWaypoint].pose.position.x << "," << waypoints.poses[destWaypoint].pose.position.y << ")" << std::endl;
    }
    robotX = msg->pose.pose.position.x;
    robotY = msg->pose.pose.position.y;
    Eigen::Quaternionf quat((msg->pose.pose.orientation.w)*180/M_PI,(msg->pose.pose.orientation.x)*180/M_PI,(msg->pose.pose.orientation.y)*180/M_PI,(msg->pose.pose.orientation.z)*180/M_PI);
    quat.normalize();
    robotHeading = atan2(2*(quat.w() * quat.z()+quat.y() * quat.x()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
    if(pointsClose(abs(robotHeading),M_PI))
        robotHeading = 0;
    //convert quaternion orientation to roll pitch yaw. Want yaw. turn should be gain*(yaw*normalized heading)
    //std::cout << "Finished processing robot location message..." << std::endl;
}
bool WaypointFollowingBasic::atWaypoint()
{

    geometry_msgs::PoseStamped pnt = waypoints.poses[destWaypoint];
    float dist = sqrt(pow((robotX-pnt.pose.position.x),2)+pow((robotY-pnt.pose.position.y),2));
    if(dist <= 0.25)
    {
        std::cout << "Arived at waypoint # " << destWaypoint << std::endl;
        geometry_msgs::Twist stopMsg;
        flor_ocs_msgs::OCSWaypointRemove achievedMsg;
        achievedMsg.waypoint_id = 0;//destWaypoint; // always refers to the next waypoint in the updated list, which is always 0
        achievedMsg.stamp = ros::Time::now();
        stopMsg.linear.x=0;
        drive_pub_.publish(stopMsg);
        remove_pub_.publish(achievedMsg);
        return true;
    }
    return false;
}

void WaypointFollowingBasic::moveRobot(const ros::TimerEvent& event)
{
    //std::cout << "Moving Robot..." << std::endl;
    if(destWaypoint >= waypoints.poses.size())
    {
        std::cout << "Reached the end of the Waypoint List" << std::endl;
        endOfList();
        return;
    }
    geometry_msgs::Twist driveMsg;
    geometry_msgs::PoseStamped pnt = waypoints.poses[destWaypoint];
    float bearing=0;
    if(pnt.pose.position.x != robotX)
        bearing = atan2(((float)(pnt.pose.position.y - robotY)),((float)(pnt.pose.position.x - robotX)));
    if(pointsClose(abs(bearing),M_PI))
        bearing = M_PI;
    driveMsg.linear.x = maxSpeed;
    float turn = maxTurn*(bearing - robotHeading);  
    if (abs(turn) >= M_PI*0.9*maxTurn && oldTurn != 0 )
        turn = oldTurn;
    else
        oldTurn = turn;
    driveMsg.angular.z = turn;
    //std::cout << "Bearing to next point  = " << bearing <<  " Current bearing = " <<  robotHeading  <<"Current Pose = (" << robotX << "," << robotY <<  ") dest point=(" << pnt.pose.position.x <<"," << pnt.pose.position.y << ") turn= " << turn << std::endl;
    drive_pub_.publish(driveMsg);
}
bool WaypointFollowingBasic::pointsClose(float x1, float x2)
{
    float diff = x1-x2;
    if(diff < 0.025*x1 && diff > -0.025*x1)
        return true;
    return false;
}

void WaypointFollowingBasic::endOfList()
{
    std::cout << "Reached the end of the Waypoint List. Waiting for more points...." << std::endl;
    timer.stop();
    destWaypoint = -1;
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_waypoint_following_basic, WaypointFollowingBasic, ocs_waypoint::WaypointFollowingBasic, nodelet::Nodelet);
