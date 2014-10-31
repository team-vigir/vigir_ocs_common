#include "interactive_marker_server_nodelet.h"

namespace ocs_interactive_marker_server
{
void InteractiveMarkerServerNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();

    interactive_marker_server_feedback_pub_ = nh.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/feedback",100, false );
    interactive_marker_server_add_sub_ = nh.subscribe<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 100, &InteractiveMarkerServerNodelet::addInteractiveMarker, this );
    interactive_marker_server_remove_sub_ = nh.subscribe<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 100, &InteractiveMarkerServerNodelet::removeInteractiveMarker, this );
    interactive_marker_server_update_sub_ = nh.subscribe<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 100, &InteractiveMarkerServerNodelet::updatePose, this );
    interactive_marker_server_mode_sub_ = nh.subscribe<flor_ocs_msgs::OCSControlMode>( "/flor/ocs/control_modes", 100, &InteractiveMarkerServerNodelet::setMode, this );

    // related to object selection
    select_object_sub_ = nh.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &InteractiveMarkerServerNodelet::processObjectSelection, this );
    selected_object_update_pub_ = nh.advertise<flor_ocs_msgs::OCSSelectedObjectUpdate>( "/flor/ocs/interactive_marker_server/selected_object_update", 100, false);

    selected_object_topic_ = "";

    timer_ = nh.createTimer(ros::Duration(0.033), &InteractiveMarkerServerNodelet::timerCallback, this);
}

void InteractiveMarkerServerNodelet::timerCallback(const ros::TimerEvent& event)
{
    //publishSelectedObject();
}

void InteractiveMarkerServerNodelet::publishSelectedObject()
{
    if (pose_map_.find(selected_object_topic_) != pose_map_.end())
    {
        flor_ocs_msgs::OCSSelectedObjectUpdate cmd;
        cmd.topic = selected_object_topic_;
        cmd.pose = pose_map_[selected_object_topic_];
        selected_object_update_pub_.publish(cmd);
    }
}

void InteractiveMarkerServerNodelet::addInteractiveMarker(const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr &msg)
{
    // name, topic, frame, scale, pointOCSInteractiveMarkerAdd
    if (marker_map_.find(msg->topic) == marker_map_.end())
    {
        //ROS_INFO("Adding marker %s", msg->topic.c_str());
        marker_map_[msg->topic] = new InteractiveMarkerServerCustom(msg->name, msg->topic, msg->mode, msg->frame, msg->scale, msg->point);
        marker_map_[msg->topic]->onFeedback = boost::bind(&InteractiveMarkerServerNodelet::onMarkerFeedback, this, _1, _2);
    }
}

void InteractiveMarkerServerNodelet::removeInteractiveMarker( const std_msgs::String::ConstPtr& msg )
{
    //ROS_ERROR("%s marker exists?", msg->data.c_str());
    if(marker_map_.find(msg->data) != marker_map_.end())
    {
        //marker_map_[msg->data];
        //ROS_INFO("Removing marker %s", msg->data.c_str());
        delete marker_map_[msg->data];
        marker_map_.erase(marker_map_.find(msg->data));
    }
}

geometry_msgs::Quaternion quatMult(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
    geometry_msgs::Quaternion q;
    q.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    q.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    return q;
}

void InteractiveMarkerServerNodelet::updatePose(const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr &msg)
{
    if(marker_map_.find(msg->topic) != marker_map_.end())
    {
        // return if message is older than latest change
//        if(msg->pose.header.stamp < pose_map_[msg->topic].header.stamp)
//            return;

        //ROS_INFO("Updating marker %s", msg->topic.c_str());
        if(msg->pose_mode == flor_ocs_msgs::OCSInteractiveMarkerUpdate::ABSOLUTE)
        {
            // simply sets the pose of the marker
            marker_map_[msg->topic]->setPose(msg->pose);

            // close the loop by sending feedback IF needed
            if(msg->update_mode == flor_ocs_msgs::OCSInteractiveMarkerUpdate::SET_POSE)
            {
                marker_map_[msg->topic]->onFeedback(msg->topic,msg->pose);
                pose_map_[msg->topic] = msg->pose;
                publishSelectedObject();
            }
        }
//        else
//        {
//            // need to add/multiply
//            geometry_msgs::PoseStamped old_pose = marker_map_[msg->topic]->getPose();
//            //ROS_INFO("%.4f %.4f %.4f",old_pose.pose.position.x,old_pose.pose.position.y,old_pose.pose.position.z);
//            geometry_msgs::PoseStamped new_pose;
//            new_pose.pose.position.x = old_pose.pose.position.x + msg->pose.pose.position.x;
//            new_pose.pose.position.y = old_pose.pose.position.y + msg->pose.pose.position.y;
//            new_pose.pose.position.z = old_pose.pose.position.z + msg->pose.pose.position.z;
//            new_pose.pose.orientation = quatMult(old_pose.pose.orientation, msg->pose.pose.orientation);
//            new_pose.header = old_pose.header;

//            marker_map_[msg->topic]->setPose(new_pose);
//            pose_map_[msg->topic] = new_pose;

//            // close the loop by sending feedback IF needed
//            if(msg->update_mode == flor_ocs_msgs::OCSInteractiveMarkerUpdate::SET_POSE)
//                marker_map_[msg->topic]->onFeedback(msg->topic,new_pose);
//        }
    }
}

void InteractiveMarkerServerNodelet::onMarkerFeedback(std::string topic_name, geometry_msgs::PoseStamped pose)
{
    flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
    cmd.topic = topic_name;
    cmd.pose = pose;
    interactive_marker_server_feedback_pub_.publish(cmd);
}

void InteractiveMarkerServerNodelet::setMode(const flor_ocs_msgs::OCSControlMode::ConstPtr& msg)
{
    //ROS_ERROR("CHANGING MODE");
    std::map<std::string,InteractiveMarkerServerCustom*>::iterator iter;
    for (iter = marker_map_.begin(); iter != marker_map_.end(); ++iter)
        if(iter->second->getMode() != flor_ocs_msgs::OCSInteractiveMarkerAdd::WAYPOINT_3DOF)
            iter->second->setMode(msg->manipulationMode);
}

void InteractiveMarkerServerNodelet::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &msg)
{
    // save the interactive marker topic
    selected_object_topic_ = "";

    //Get id of object that is selected
    switch(msg->type)
    {
        case flor_ocs_msgs::OCSObjectSelection::TEMPLATE:
            selected_object_topic_ = "/template_"+boost::lexical_cast<std::string>(msg->id)+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP:
            selected_object_topic_ = "/footstep_"+boost::lexical_cast<std::string>(msg->id/2)+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP_GOAL:
            selected_object_topic_ = "/footstep_goal_"+boost::lexical_cast<std::string>(msg->id/2)+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR:
            selected_object_topic_ = (msg->id == flor_ocs_msgs::OCSObjectSelection::LEFT_ARM ? "/l_arm_pose_marker" : "/r_arm_pose_marker");
            break;
        default:
            break;
    }

    ROS_ERROR("SELECTED OBJECT: %s", selected_object_topic_.c_str());

    pose_map_[selected_object_topic_] = marker_map_[selected_object_topic_]->getPose();
    publishSelectedObject();
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_interactive_marker_server_nodelet, InteractiveMarkerServerNodelet, ocs_interactive_marker_server::InteractiveMarkerServerNodelet, nodelet::Nodelet);
