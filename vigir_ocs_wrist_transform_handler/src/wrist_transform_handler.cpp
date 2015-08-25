#include <vigir_ocs_wrist_transform_handler/wrist_transform_handler.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <tf_conversions/tf_eigen.h>

#include <string>

namespace vigir_ocs_wrist_transform_handler {

WristTransformHandler::WristTransformHandler() {
    ros::NodeHandle nh;

    //Get hand parameters from server
    std::string left_palm_link = "left_palm";
    std::string left_wrist_link = "left_hand";
    std::string right_palm_link = "right_palm";
    std::string right_wrist_link = "right_hand";

    if(!nh.getParam("/left_wrist_link", left_wrist_link))
        ROS_WARN("No left wrist link defined, using l_hand as default");
    if(!nh.getParam("/left_palm_link",  left_palm_link))
        ROS_WARN("No left palm link defined, using l_palm as default");

    if(!nh.getParam("/right_wrist_link", right_wrist_link))
        ROS_WARN("No right wrist link defined, using r_hand as default");
    if(!nh.getParam("/right_palm_link",  right_palm_link))
        ROS_WARN("No right palm link defined, using r_palm as default");

    //Getting hand_T_palm transforms from URDF
    robot_model_loader::RobotModelLoader robot_urdf_model_loader("robot_description");
    robot_model::RobotModelPtr robot_urdf_model = robot_urdf_model_loader.getModel();

    r_hand_T_palm_.setIdentity();
    l_hand_T_palm_.setIdentity();

    r_hand_T_marker_.setIdentity();
    l_hand_T_marker_.setIdentity();

    //Getting left side
    if(!robot_urdf_model->hasLinkModel(left_palm_link)){
        ROS_WARN("Hand model does not contain %s, not geting transform",left_palm_link.c_str());
    }else{
        robot_model::LinkTransformMap hand_palm_tf_map = robot_urdf_model->getLinkModel(left_palm_link)->getAssociatedFixedTransforms();
        ROS_INFO("Requested linktransform for %s",left_palm_link.c_str());

        Eigen::Affine3d hand_palm_aff;
        bool found = false;

        for(robot_model::LinkTransformMap::iterator it = hand_palm_tf_map.begin(); it != hand_palm_tf_map.end(); ++it){
            if(it->first->getName() == left_wrist_link){
                ROS_INFO("Wrist %s found!!!",left_wrist_link.c_str());
                hand_palm_aff = it->second;
                found = true;
                break;
            }
        }
        if(found){
            tf::transformEigenToTF( hand_palm_aff,l_hand_T_palm_);
            l_hand_T_palm_   = l_hand_T_palm_.inverse();
            l_hand_T_marker_ = l_hand_T_palm_;
        }
    }

    //Getting right side
    if(!robot_urdf_model->hasLinkModel(right_palm_link)){
        ROS_WARN("Hand model does not contain %s, not geting transform",right_palm_link.c_str());
    }else{

        Eigen::Affine3d hand_palm_aff;
        bool found = false;

        robot_model::LinkTransformMap hand_palm_tf_map = robot_urdf_model->getLinkModel(right_palm_link)->getAssociatedFixedTransforms();
        ROS_INFO("Requested linktransform for %s",right_palm_link.c_str());

        found = false;

        for(robot_model::LinkTransformMap::iterator it = hand_palm_tf_map.begin(); it != hand_palm_tf_map.end(); ++it){
            if(it->first->getName() == right_wrist_link){
                ROS_INFO("Wrist %s found!!!",right_wrist_link.c_str());
                hand_palm_aff = it->second;
                found = true;
                break;
            }
        }
        if(found){
            tf::transformEigenToTF( hand_palm_aff,r_hand_T_palm_);
            r_hand_T_palm_   = r_hand_T_palm_.inverse();
            r_hand_T_marker_ = r_hand_T_palm_;
        }
    }

    //Finished getting hand transform

    // init server
    wrist_transform_server_ = nh.advertiseService("wrist_transform", &WristTransformHandler::wristTransformServerCallback, this);

}

WristTransformHandler::~WristTransformHandler() {

}

bool WristTransformHandler::wristTransformServerCallback(vigir_ocs_msgs::OCSRequestWristTransform::Request &req, vigir_ocs_msgs::OCSRequestWristTransform::Response &res) {
    tf::Transform selected_transform;

    switch( req.transform_type ) {
    case vigir_ocs_msgs::OCSRequestWristTransformRequest::L_HAND_T_PALM:
        selected_transform = l_hand_T_palm_;
        break;

    case vigir_ocs_msgs::OCSRequestWristTransformRequest::R_HAND_T_PALM:
        selected_transform = r_hand_T_palm_;
        break;

    case vigir_ocs_msgs::OCSRequestWristTransformRequest::L_HAND_T_MARKER:
        selected_transform = l_hand_T_palm_;
        break;

    case vigir_ocs_msgs::OCSRequestWristTransformRequest::R_HAND_T_MARKER:
        selected_transform = r_hand_T_marker_;
        break;

    default:
        ROS_WARN("[WristTransformHandler] Requested invalid transform type: %d", req.transform_type);
        return false;
    }

    res.transform.position.x = selected_transform.getOrigin().getX();
    res.transform.position.y = selected_transform.getOrigin().getY();
    res.transform.position.z = selected_transform.getOrigin().getZ();
    res.transform.orientation.w = selected_transform.getRotation().getW();
    res.transform.orientation.x = selected_transform.getRotation().getX();
    res.transform.orientation.y = selected_transform.getRotation().getY();
    res.transform.orientation.z = selected_transform.getRotation().getZ();

    return true;
}

}
