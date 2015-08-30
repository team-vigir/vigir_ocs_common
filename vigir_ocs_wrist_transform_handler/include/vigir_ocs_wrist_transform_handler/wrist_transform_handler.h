#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <vigir_ocs_msgs/OCSRequestWristTransform.h>

namespace vigir_ocs_wrist_transform_handler {

class WristTransformHandler {
public:
    WristTransformHandler();
    ~WristTransformHandler();

private:
    bool wristTransformServerCallback(vigir_ocs_msgs::OCSRequestWristTransform::Request &req, vigir_ocs_msgs::OCSRequestWristTransform::Response &res);

    ros::ServiceServer wrist_transform_server_;

    tf::Transform r_hand_T_palm_;
    tf::Transform l_hand_T_palm_;

    tf::Transform r_hand_T_marker_;
    tf::Transform l_hand_T_marker_;
};

}
