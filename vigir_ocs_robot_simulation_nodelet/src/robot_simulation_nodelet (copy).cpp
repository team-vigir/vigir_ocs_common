#include "robot_simulation_nodelet.h"

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
    std::cout << "Received new tf (frame id: " << msg->child_frame_id << ")" << std::endl;
    this->publishRobotSimulationList(msg);
}

void RobotSimulationNodelet::publishRobotSimulationTF(const tf::tfMessage::ConstPtr& msg)
{
    tf::tfMessage cmd;

    cmd.header.frame_id = std::string("/simulation") + msg->header.frame_id;
    cmd.child_frame_id = std::string("/simulation") + msg->header.child_frame_id;
    cmd.transform = msg->transform;

    // publish complete list of robot_simulations and poses
    robot_simulation_tf_pub_.publish( cmd );
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_robot_simulation_nodelet, RobotSimulationNodelet, ocs_robot_simulation::RobotSimulationNodelet, nodelet::Nodelet);
