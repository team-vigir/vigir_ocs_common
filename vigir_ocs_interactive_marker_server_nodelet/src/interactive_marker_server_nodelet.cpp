#include "interactive_marker_server_nodelet.h"

namespace ocs_interactive_marker_server
{
void InteractiveMarkerServerNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();

    interactive_marker_server_feedback_pub_ = nh.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/feedback",100, false );
    interactive_marker_server_add_sub_ = nh.subscribe<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 100, &InteractiveMarkerServerNodelet::addInteractiveMarker, this );
    interactive_marker_server_remove_sub_ = nh.subscribe<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 100, &InteractiveMarkerServerNodelet::removeInteractiveMarker, this );
    interactive_marker_server_update_sub_ = nh.subscribe( "/flor/ocs/interactive_marker_server/update", 100, &InteractiveMarkerServerNodelet::updatePose, this);
    interactive_marker_server_mode_sub_ = nh.subscribe<flor_ocs_msgs::OCSControlMode>( "/flor/ocs/control_modes", 100, &InteractiveMarkerServerNodelet::setMode, this );
    interactive_marker_server_visibility_sub_ = nh.subscribe<flor_ocs_msgs::OCSMarkerVisibility>("/flor/ocs/interactive_marker_server/visibility",5, &InteractiveMarkerServerNodelet::processMarkerVisibility,this);

    // related to object selection
    select_object_sub_ = nh.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &InteractiveMarkerServerNodelet::processObjectSelection, this );
    selected_object_update_pub_ = nh.advertise<flor_ocs_msgs::OCSSelectedObjectUpdate>( "/flor/ocs/interactive_marker_server/selected_object_update", 100, false);

    marker_feedback_timer_ = boost::posix_time::second_clock::local_time();
}

void InteractiveMarkerServerNodelet::publishSelectedObject()
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    for (std::map<std::string,std::string>::iterator it = host_selected_object_topic_map_.begin(); it != host_selected_object_topic_map_.end(); ++it)
    {
        std::string host = it->first;
        std::string selected_object_topic = it->second;

        flor_ocs_msgs::OCSSelectedObjectUpdate cmd;
        cmd.topic = selected_object_topic;
        cmd.pose = pose_map_[selected_object_topic];
        cmd.host = host;
        selected_object_update_pub_.publish(cmd);
    }
}

void InteractiveMarkerServerNodelet::addInteractiveMarker(const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr &msg)
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    // name, topic, frame, scale, point
    if (marker_map_.find(msg->topic) == marker_map_.end())
    {
        //ROS_INFO("Adding marker %s", msg->topic.c_str());
        marker_map_[msg->topic] = new InteractiveMarkerServerCustom(msg->name, msg->topic, msg->mode, msg->frame, msg->scale, msg->point);
        marker_map_[msg->topic]->onFeedback = boost::bind(&InteractiveMarkerServerNodelet::onMarkerFeedback, this, _1, _2, _3, _4);
    }

}

void InteractiveMarkerServerNodelet::removeInteractiveMarker( const std_msgs::String::ConstPtr& msg )
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    //ROS_ERROR("%s marker exists?", msg->data.c_str());
    if(marker_map_.find(msg->data) != marker_map_.end())
    {
        //marker_map_[msg->data];
        //ROS_INFO("Removing marker %s", msg->data.c_str());
        delete marker_map_[msg->data];
        marker_map_.erase(marker_map_.find(msg->data));
    }
}

void InteractiveMarkerServerNodelet::updatePose( const flor_ocs_msgs::OCSInteractiveMarkerUpdate::ConstPtr msg )
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    if(marker_map_.find(msg->topic) != marker_map_.end())
    {
        // return if message is older than latest change
        //if(msg->pose.header.stamp < pose_map_[msg->topic].header.stamp)
        //    return;

        // simply sets the pose of the marker
        marker_map_[msg->topic]->setPose(msg->pose);

        // close the loop by sending feedback IF needed
        if(msg->update_mode == flor_ocs_msgs::OCSInteractiveMarkerUpdate::SET_POSE)
        {
            marker_map_[msg->topic]->onFeedback(msg->event_type,msg->topic,msg->pose,msg->client_id);
            pose_map_[msg->topic] = msg->pose;
            publishSelectedObject();
        }
    }
}

void InteractiveMarkerServerNodelet::onMarkerFeedback(unsigned char event_type, std::string topic_name, geometry_msgs::PoseStamped pose, std::string client_id)
{
    // throttle marker feedback to ~30hz
    //if(event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE && (boost::posix_time::second_clock::local_time()-marker_feedback_timer_).total_milliseconds() < 33)
    //    return;

    marker_feedback_timer_ = boost::posix_time::second_clock::local_time();

    //ROS_INFO("update_pose: %s -> %s",client_id.c_str(),topic_name.c_str());
    flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
    cmd.client_id = client_id;
    cmd.topic = topic_name;
    cmd.pose = pose;
    cmd.event_type = event_type;
    {
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_publisher_mutex_ );
    interactive_marker_server_feedback_pub_.publish(cmd);
    }

    if(event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        publishSelectedObject();
}

void InteractiveMarkerServerNodelet::setMode(const flor_ocs_msgs::OCSControlMode::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    //ROS_ERROR("CHANGING MODE");
    std::map<std::string,InteractiveMarkerServerCustom*>::iterator iter;
    for (iter = marker_map_.begin(); iter != marker_map_.end(); ++iter)
        if(iter->second->getMode() != flor_ocs_msgs::OCSInteractiveMarkerAdd::WAYPOINT_3DOF)
            iter->second->setMode(msg->manipulationMode);
}

void InteractiveMarkerServerNodelet::processMarkerVisibility(const flor_ocs_msgs::OCSMarkerVisibility::ConstPtr &msg)
{
    //ROS_ERROR("marker visibiilty");
    //set visibility of different interactive markers
    if(msg->all_markers)
    {
        boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

        std::map<std::string,InteractiveMarkerServerCustom*>::iterator iter;
        for (iter = marker_map_.begin(); iter != marker_map_.end(); ++iter)
        {
            iter->second->setVisible(msg->all_markers_visibility);
        }
    }
    //TODO:: change markers to have visibility set via changing markers, not enable/disable
    //not worrying about selective visibility of markers for now
//    else // just disable by type
//    {
//        std::map<std::string,InteractiveMarkerServerCustom*>::iterator iter;
//        for (iter = marker_map_.begin(); iter != marker_map_.end(); ++iter)
//        {
//            //string comparison on the marker's topic to determine type
//            std::string topic_name = iter->first;
//            ROS_ERROR("marker topic %s",topic_name.c_str());
//            //templates
//            if(topic_name.find("/template"))
//            {
//                iter->second->setVisible(msg->template_visibility);
//            }
//            else if(topic_name.find("/footstep_"))
//            {
//                iter->second->setVisible(msg->template_visibility);
//            }
//            else if(topic_name.find("/footstep_goal_"))
//            {
//                iter->second->setVisible(msg->template_visibility);
//            }
//        }
//    }
}

void InteractiveMarkerServerNodelet::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &msg)
{
    boost::recursive_mutex::scoped_lock lock( interactive_marker_server_change_mutex_ );

    // save the interactive marker topic
    std::string selected_object_topic = "";

    //Get id of object that is selected
    switch(msg->type)
    {
        case flor_ocs_msgs::OCSObjectSelection::TEMPLATE:
            selected_object_topic = "/template_"+boost::lexical_cast<std::string>(msg->id)+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP:
            selected_object_topic = "/footstep_"+boost::lexical_cast<std::string>(msg->id/2)+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP_GOAL:
            selected_object_topic = "/footstep_goal_"+boost::lexical_cast<std::string>(msg->id/2 ? "right" : "left")+"_marker";
            break;
        case flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR:
            selected_object_topic = (msg->id == flor_ocs_msgs::OCSObjectSelection::LEFT_ARM ? "/l_arm_pose_marker" : "/r_arm_pose_marker");
            break;
        default:
            break;
    }

    ROS_INFO("SELECTED OBJECT: %s", selected_object_topic.c_str());

    host_selected_object_topic_map_[msg->host] = selected_object_topic;
    pose_map_[host_selected_object_topic_map_[msg->host]] = marker_map_[selected_object_topic]->getPose();

    publishSelectedObject();

}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_interactive_marker_server_nodelet, InteractiveMarkerServerNodelet, ocs_interactive_marker_server::InteractiveMarkerServerNodelet, nodelet::Nodelet);
