#include "robot_simulation_nodelet.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace ocs_robot_simulation
{
void RobotSimulationNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();

    // also create a publisher to set parameters of cropped image
    robot_simulation_tf_pub_  = nh.advertise<tf::tfMessage>( "tf", 1, false );

    // then, subscribe to the resulting cropped image
    robot_tf_sub_   = nh.subscribe<tf::tfMessage>( "tf", 1, &RobotSimulationNodelet::updateTFCb, this );
}

void RobotSimulationNodelet::updateTFCb(const tf::tfMessage::ConstPtr& msg)
{
    std::cout << "Received new tf (n_tfs: " << msg->transforms.size() << ")" << std::endl;
    this->publishRobotSimulationTF(msg);
}

void RobotSimulationNodelet::publishRobotSimulationTF(const tf::tfMessage::ConstPtr& msg)
{
    tf::TransformBroadcaster br;
    for(int i = 0; i < msg->transforms.size(); i++)
    {
        std::cout << "  frame_id: " << msg->transforms[i].header.frame_id << " -> " << msg->transforms[i].child_frame_id << ")" << std::endl;
        if(msg->transforms[i].header.frame_id.find("/simulation") != 0)
        {
            tf::Vector3 pos(msg->transforms[i].transform.translation.x,msg->transforms[i].transform.translation.y,msg->transforms[i].transform.translation.z);
            tf::Quaternion quat(msg->transforms[i].transform.rotation.x,msg->transforms[i].transform.rotation.y,msg->transforms[i].transform.rotation.z,msg->transforms[i].transform.rotation.w);
            tf::Transform transform(quat,pos);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), std::string("/simulation") + msg->transforms[i].header.frame_id, std::string("/simulation") + msg->transforms[i].child_frame_id));
            std::cout << "     new frame_id: " << msg->transforms[i].header.frame_id << " -> " << msg->transforms[i].child_frame_id << ")" << std::endl;
        }
    }
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_robot_simulation_nodelet, RobotSimulationNodelet, ocs_robot_simulation::RobotSimulationNodelet, nodelet::Nodelet);
