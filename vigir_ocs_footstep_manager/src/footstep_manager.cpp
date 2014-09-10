#include "footstep_manager.h"

#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/Foot.h>

#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>

namespace ocs_footstep
{
void FootstepManager::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();

    nh.param("foot/size/x", foot_size.x, 0.26);
    nh.param("foot/size/y", foot_size.y, 0.13);
    nh.param("foot/size/z", foot_size.z, 0.015);
    nh.param("foot/origin_shift/x", foot_origin_shift.x, 0.0);
    nh.param("foot/origin_shift/y", foot_origin_shift.y, 0.0);
    nh.param("foot/origin_shift/z", foot_origin_shift.z, 0.0);
    nh.param("foot/separation", foot_separation, 0.23);

    nh.param("upper_body/size/x", upper_body_size.x, 0.7);
    nh.param("upper_body/size/y", upper_body_size.y, 1.1);
    nh.param("upper_body/size/z", upper_body_size.z, 0.0);
    nh.param("upper_body/origin_shift/x", upper_body_origin_shift.x, 0.0);
    nh.param("upper_body/origin_shift/y", upper_body_origin_shift.y, 0.0);
    nh.param("upper_body/origin_shift/z", upper_body_origin_shift.z, 0.0);

    // creates publishers and subscribers for the interaction loop
    footstep_list_pub_          = nh.advertise<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, false );
    footstep_update_sub_        = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/update", 1, &FootstepManager::processFootstepPoseUpdate, this );

    // creates publishers for visualization messages
    footstep_array_pub_         = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_array", 1, false );
    footstep_body_bb_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_path_body_array", 1, false );
    footstep_path_pub_          = nh.advertise<nav_msgs::Path>( "/flor/ocs/footstep/path", 1, false );

    //////// placeholders, waiting for alex to provide real message
    // for now, creates subscribers for the messages coming from the footstep planner
    // this will become a single action for the footstep result coming from the planner
    footstep_array_sub_         = nh.subscribe<visualization_msgs::MarkerArray>( "/flor/walk_monitor/footsteps_array", 1, &FootstepManager::processFootstepArray, this );
    footstep_body_bb_array_sub_ = nh.subscribe<visualization_msgs::MarkerArray>( "/flor/walk_monitor/footsteps_path_body_array", 1, &FootstepManager::processFootstepBodyBBArray, this );
    footstep_path_sub_          = nh.subscribe<nav_msgs::Path>( "/flor/walk_monitor/path", 1, &FootstepManager::processFootstepPathArray, this );

    timer = nh.createTimer(ros::Duration(0.066), &FootstepManager::timerCallback, this);
}

void FootstepManager::timerCallback(const ros::TimerEvent& event)
{
    //this->publishFootstepList();
}

void FootstepManager::processFootstepArray(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    // for now, simply forward footstep array message since it has correct orientation and so on
    footstep_array_ = *msg;
    // and use namespace to identify foot markers
    for(int i = 0; i < footstep_array_.markers.size(); i++)
    {
        //ROS_ERROR("process %d",i);
        if(i % 2 == 0)
            footstep_array_.markers[i].ns = std::string("footstep");
    }

    // I'm going to receive a StepPlan message
    ///// placeholder that creates fake stepplan message based on marker array
    vigir_footstep_planning_msgs::StepPlan input;
    for(int i = 0; i < footstep_array_.markers.size(); i++)
    {
        if(i % 2 == 0)
        {
            vigir_footstep_planning_msgs::Foot foot;
            foot.foot = ((footstep_array_.markers[i].color.r == 1.0) ? vigir_footstep_planning_msgs::Foot::LEFT : vigir_footstep_planning_msgs::Foot::RIGHT);
            foot.pose = footstep_array_.markers[i].pose;
            vigir_footstep_planning_msgs::Step step;
            step.foot = foot;
            step.step_index = (int)i/2;
            input.steps.push_back(step);
        }
    }

    // which has a list of steps
    // each step has step index, foot, and bool locked
    // for each step, will need to create a set of two footstep markers
    // a TEXT_VIEW_FACING and CUBE
    //visualization_msgs::MarkerArray foot_array_msg;
    //stepPlanToFootMarkerArray(input, foot_array_msg);
    //footstep_array_ = foot_array_msg;

    // and body marker array
    //visualization_msgs::MarkerArray body_array_msg;
    //stepPlanToBodyMarkerArray(input, body_array_msg);
    //footstep_body_array_ = foot_array_msg;

    // also need to create path
    //nav_msgs::Path path_msg;
    //stepPlanToFootPath(input, path_msg);
    //footstep_path_ = path_msg;

    // save last plan
    footstep_plan_ = input;

    publishFootstepVis();
    publishFootstepList();
}

void FootstepManager::processFootstepBodyBBArray(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    footstep_body_array_ = *msg;
    publishFootstepVis();
}

void FootstepManager::processFootstepPathArray(const nav_msgs::Path::ConstPtr& msg)
{
    footstep_path_ = *msg;
    publishFootstepVis();
}

void FootstepManager::publishFootstepVis()
{
    footstep_array_pub_.publish(footstep_array_);
    footstep_body_bb_array_pub_.publish(footstep_body_array_);
    footstep_path_pub_.publish(footstep_path_);
}

void FootstepManager::processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg)
{
    if(msg->footstep_id >= footstep_plan_.steps.size())
        return;

    for(int i = 0; i < footstep_plan_.steps.size(); i++)
    {
        if(msg->footstep_id == footstep_plan_.steps[i].step_index)
        {
            footstep_array_.markers[i*2].pose = msg->pose.pose;   // step marker
            footstep_array_.markers[i*2+1].pose = msg->pose.pose; // text id
            footstep_path_.poses[i] = msg->pose;                  // path
            break;
        }
    }

    // need to send update request to footstep planner?
    footstep_array_pub_.publish(footstep_array_);
    footstep_path_pub_.publish(footstep_path_);
}

void FootstepManager::publishFootstepList()
{
    flor_ocs_msgs::OCSFootstepList list;
    // if using step plan
    //for(int i = 0; i < footstep_plan_.steps.size(); i++)
    // else
    for(int i = 0; i < footstep_array_.markers.size(); i++)
    {
        //ROS_ERROR("publish %d",i);
        // if using step plan
        //list.footstep_id_list.push_back(footstep_plan_.steps[i].step_index);
        // else
        if(i % 2 == 0)
        {
            list.footstep_id_list.push_back(i/2);
            // use the already calculated pose for the marker array
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = footstep_array_.markers[i].pose.position.x;
            pose.pose.position.y = footstep_array_.markers[i].pose.position.y;
            pose.pose.position.z = footstep_array_.markers[i].pose.position.z;
            pose.pose.orientation.x = footstep_array_.markers[i].pose.orientation.x;
            pose.pose.orientation.y = footstep_array_.markers[i].pose.orientation.y;
            pose.pose.orientation.z = footstep_array_.markers[i].pose.orientation.z;
            pose.pose.orientation.w = footstep_array_.markers[i].pose.orientation.w;
            list.pose.push_back(pose);
        }
    }
    footstep_list_pub_.publish(list);
}

void FootstepManager::stepPlanToFootMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    if(!input.steps.size())
        return;

    foot_array_msg.markers.clear();
    for(int i = 0; input.steps.size(); i++)
    {
        visualization_msgs::Marker marker;
        stepToMarker(input.steps[i], marker);

        marker.id = foot_array_msg.markers.size();
        marker.color.r = input.steps[i].foot.foot == vigir_footstep_planning_msgs::Foot::LEFT ? 0.6 : 0.0;
        marker.color.g = input.steps[i].foot.foot == vigir_footstep_planning_msgs::Foot::LEFT ? 0.0 : 0.6;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.ns = std::string("footstep");
        foot_array_msg.markers.push_back(marker);

        // add text
        marker.id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = boost::lexical_cast<std::string>(input.steps[i].step_index);
        marker.scale.z *= 3;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        foot_array_msg.markers.push_back(marker);
    }
}

void FootstepManager::stepPlanToBodyMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& body_array_msg)
{
    if(!input.steps.size())
        return;

    visualization_msgs::Marker marker;
    marker.header = input.steps[0].foot.header;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    body_array_msg.markers.clear();
    for(int i = 1; i < input.steps.size(); i++)
    {
        // approximate upper body dimensions
        float x = input.steps[i].foot.pose.position.x + 0.5 * (input.steps[i-1].foot.pose.position.x - input.steps[i].foot.pose.position.x);
        float y = input.steps[i].foot.pose.position.y + 0.5 * (input.steps[i-1].foot.pose.position.y - input.steps[i].foot.pose.position.y);
        float z = input.steps[i].foot.pose.position.z + 0.5 * (input.steps[i-1].foot.pose.position.z - input.steps[i].foot.pose.position.z);
        float yaw1 = tf::getYaw(input.steps[i-1].foot.pose.orientation);
        float yaw2 = tf::getYaw(input.steps[i].foot.pose.orientation);
        float theta = yaw2 + 0.5 * (yaw1 - yaw2);

        // compute center position of body
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        // determine shift of polygon based on orientation
        tf::Pose transform;
        tf::poseMsgToTF(marker.pose, transform);
        tf::Vector3 shift_world;
        tf::vector3MsgToTF(upper_body_origin_shift, shift_world);
        shift_world = transform.getBasis() * shift_world;

        marker.pose.position.x += shift_world.getX();
        marker.pose.position.y += shift_world.getY();
        marker.pose.position.z += shift_world.getZ();
        // end shift

        // rescale marker based on body size
        marker.scale.x = upper_body_size.x;
        marker.scale.y = upper_body_size.y;
        marker.scale.z = 0.05;

        marker.id = body_array_msg.markers.size();
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.5;
        marker.color.a = 0.2;
        body_array_msg.markers.push_back(marker);
    }
}

void FootstepManager::stepPlanToFootPath(vigir_footstep_planning_msgs::StepPlan& input, nav_msgs::Path& foot_path_msg)
{
    if(!input.steps.size())
        return;

    foot_path_msg.header = input.header;
    foot_path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < input.steps.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = input.steps[i].foot.header;
        pose.header.stamp = ros::Time::now();
        pose.pose.position = input.steps[i].foot.pose.position;

        foot_path_msg.poses.push_back(pose);
    }
}

void FootstepManager::stepToMarker(const vigir_footstep_planning_msgs::Step &step, visualization_msgs::Marker &marker)
{
    vigir_footstep_planning_msgs::Step step_transformed = step;
    step_transformed.foot.pose.position.z += foot_size.z/2; // marker should touch ground

    // shift to foot center (remove shift to foot frame)
    geometry_msgs::Vector3 shift_foot = foot_origin_shift;
    if (step.foot.foot == vigir_footstep_planning_msgs::Foot::LEFT)
      shift_foot.y = -shift_foot.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(step.foot.pose.orientation, q);
    tf::Matrix3x3 rot(q);

    tf::Vector3 shift_world;
    tf::vector3MsgToTF(shift_foot, shift_world);
    shift_world = rot * shift_world;

    step_transformed.foot.pose.position.x -= shift_world.getX();
    step_transformed.foot.pose.position.y -= shift_world.getY();
    step_transformed.foot.pose.position.z -= shift_world.getZ();
    // end shift

    marker.header = step_transformed.foot.header;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // compute absolut position of foot
    marker.pose = step_transformed.foot.pose;

    // rescale marker based on foot size
    marker.scale = foot_size;
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_footstep_manager, FootstepManager, ocs_footstep::FootstepManager, nodelet::Nodelet);
