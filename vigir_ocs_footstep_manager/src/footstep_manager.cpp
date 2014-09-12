#include "footstep_manager.h"

#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
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
    nh.param("foot/size/z", foot_size.z, 0.05);
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
    footstep_undo_req_sub_      = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/undo", 1, &FootstepManager::processUndoRequest, this );
    footstep_redo_req_sub_      = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/redo", 1, &FootstepManager::processRedoRequest, this );

    // creates publishers for visualization messages
    footstep_array_pub_         = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_array", 1, false );
    footstep_body_bb_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_path_body_array", 1, false );
    footstep_path_pub_          = nh.advertise<nav_msgs::Path>( "/flor/ocs/footstep/path", 1, false );

    // use this to track current feet pose
    lower_body_state_sub_       = nh.subscribe("/flor/state/lower_body_world", 1, &FootstepManager::processLowerBodyState, this);

    // footstep request coming from the OCS
    footstep_plan_request_sub_  = nh.subscribe<geometry_msgs::PoseStamped>( "/flor/ocs/footstep/goal_pose", 1, &FootstepManager::processFootstepGoalPose, this );

    //////// placeholders, waiting for alex to provide real message
    // for now, creates subscribers for the messages coming from the footstep planner
    // this will become a single action for the footstep result coming from the planner
    footstep_array_sub_         = nh.subscribe<visualization_msgs::MarkerArray>( "/flor/walk_monitor/footsteps_array", 1, &FootstepManager::processFootstepArray, this );
    footstep_body_bb_array_sub_ = nh.subscribe<visualization_msgs::MarkerArray>( "/flor/walk_monitor/footsteps_path_body_array", 1, &FootstepManager::processFootstepBodyBBArray, this );
    footstep_path_sub_          = nh.subscribe<nav_msgs::Path>( "/flor/walk_monitor/path", 1, &FootstepManager::processFootstepPathArray, this );

    // initialize all ros action clients
    //plan request
    step_plan_request_client_ = new StepPlanRequestClient("/vigir/global_footstep_planner/step_plan_request", true);
    step_plan_request_client_->waitForServer();
    //edit step
    edit_step_client_ = new EditStepClient("/vigir/global_footstep_planner/edit_step", true);
    edit_step_client_->waitForServer();

    timer = nh.createTimer(ros::Duration(0.066), &FootstepManager::timerCallback, this);
}

void FootstepManager::timerCallback(const ros::TimerEvent& event)
{
    this->publishFootstepList();
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
    //vigir_footstep_planning_msgs::StepPlan input;
    //for(int i = 0; i < footstep_array_.markers.size(); i++)
    //{
    //    if(i % 2 == 0)
    //    {
    //        vigir_footstep_planning_msgs::Foot foot;
    //        foot.foot_index = ((footstep_array_.markers[i].color.r == 1.0) ? vigir_footstep_planning_msgs::Foot::LEFT : vigir_footstep_planning_msgs::Foot::RIGHT);
    //        foot.pose = footstep_array_.markers[i].pose;
    //        vigir_footstep_planning_msgs::Step step;
    //        step.foot = foot;
    //        step.step_index = (int)i/2;
    //        input.steps.push_back(step);
    //    }
    //}

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
    //footstep_plans_stack_.top().back() = input;

    //publishFootstepVis();
    //publishFootstepList();
}

void FootstepManager::processFootstepBodyBBArray(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //footstep_body_array_ = *msg;
    //publishFootstepVis();
}

void FootstepManager::processFootstepPathArray(const nav_msgs::Path::ConstPtr& msg)
{
    //footstep_path_ = *msg;
    //publishFootstepVis();
}

void FootstepManager::processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg)
{
    if(msg->footstep_id >= getStepPlan().steps.size())
        return;

    vigir_footstep_planning_msgs::EditStepGoal action_goal;
    action_goal.step_plan = getStepPlan();
    // find step in the plan
    int step_index;
    for(int i = 0; i < getStepPlan().steps.size(); i++)
    {
        if(msg->footstep_id == getStepPlan().steps[i].step_index)
        {
            step_index = i;
            break;
        }
    }
    action_goal.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_2D;
    action_goal.edit_step.step = getStepPlan().steps[step_index];
    action_goal.edit_step.step.foot.pose = msg->pose.pose;

    // Fill in goal here
    edit_step_client_->sendGoalAndWait(action_goal, ros::Duration(60.0));
    if(edit_step_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        vigir_footstep_planning_msgs::EditStepResultConstPtr result = edit_step_client_->getResult();
        ROS_ERROR("Got action response: [%s]", result->status.error_msg.c_str());

        if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
        {
            // add resulting plan to the top of the stack of plans
            getStepPlanList().insert(getStepPlanList().end(), result->step_plans.begin(), result->step_plans.end());

            publishFootstepList();
        }
    }
}

void FootstepManager::processUndoRequest(const std_msgs::Bool::ConstPtr& msg)
{
    undo();
}

void FootstepManager::processRedoRequest(const std_msgs::Bool::ConstPtr& msg)
{
    redo();
}

void FootstepManager::stepToMarker(const vigir_footstep_planning_msgs::Step &step, visualization_msgs::Marker &marker)
{
    vigir_footstep_planning_msgs::Step step_transformed = step;
    step_transformed.foot.pose.position.z += foot_size.z/2; // marker should touch ground

    // shift to foot center (remove shift to foot frame)
    geometry_msgs::Vector3 shift_foot = foot_origin_shift;
    if (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT)
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

void FootstepManager::stepPlanToFootMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    if(!input.steps.size())
        return;

    foot_array_msg.markers.clear();
    for(int i = 0; i < input.steps.size(); i++)
    {
        visualization_msgs::Marker marker;
        stepToMarker(input.steps[i], marker);

        marker.id = foot_array_msg.markers.size();
        marker.color.r = 0.0;//input.steps[i].foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? 0.6 : 0.0;
        marker.color.g = 0.6;//input.steps[i].foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? 0.0 : 0.6;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.ns = std::string("footstep");
        foot_array_msg.markers.push_back(marker);

        // add text
        marker.id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = boost::lexical_cast<std::string>(input.steps[i].step_index);
        marker.scale.x *= 2.0;
        marker.scale.y *= 2.0;
        marker.scale.z *= 2.0;
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
        marker.scale.z = 0.02;

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

void FootstepManager::processLowerBodyState(const flor_state_msgs::LowerBodyStateConstPtr &lower_body_state)
{
    lower_body_state_ = *lower_body_state;
}

void FootstepManager::processFootstepGoalPose(const geometry_msgs::PoseStampedConstPtr &goal_pose)
{
    goal_pose_ = *goal_pose;

    requestStepPlan();
}

void FootstepManager::requestStepPlan()
{
    // This function will create a completely new plan, so we need to add a new empty list of plans to the stack
    addNewPlanList();

    // first we calculate start and end feet poses
    vigir_footstep_planning_msgs::Feet start;
    vigir_footstep_planning_msgs::Feet goal;

    //start left
    start.left.foot_index = vigir_footstep_planning_msgs::Foot::LEFT;
    start.left.pose = lower_body_state_.left_foot_pose;
    //start right
    start.right.foot_index = vigir_footstep_planning_msgs::Foot::RIGHT;
    start.right.pose = lower_body_state_.right_foot_pose;

    //end estimates for foot distance
    double end_yaw = tf::getYaw(goal_pose_.pose.orientation);
    double shift_x = -sin(end_yaw) * (0.5 * foot_separation);
    double shift_y =  cos(end_yaw) * (0.5 * foot_separation);

    goal.left.pose.position.x = goal_pose_.pose.position.x + shift_x;
    goal.left.pose.position.y = goal_pose_.pose.position.y + shift_y;
    goal.left.pose.position.z = goal_pose_.pose.position.z;
    goal.left.pose.orientation = goal_pose_.pose.orientation;

    goal.right.pose.position.x = goal_pose_.pose.position.x - shift_x;
    goal.right.pose.position.y = goal_pose_.pose.position.y - shift_y;
    goal.right.pose.position.z = goal_pose_.pose.position.z;
    goal.right.pose.orientation = goal_pose_.pose.orientation;

    requestStepPlan(start, goal);
}

void FootstepManager::requestStepPlanFromStep(vigir_footstep_planning_msgs::Step& step)
{
}

void FootstepManager::requestStepPlan(vigir_footstep_planning_msgs::Feet& start, vigir_footstep_planning_msgs::Feet& goal)
{
    vigir_footstep_planning_msgs::StepPlanRequest request;

    request.start = start;
    request.goal = goal;

    // default planning mode is 2D, but will get that from the OCS
    request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

    // need to get the following from the OCS as well
    //float32 max_planning_time         # maximum planning time given in second
    //float32 max_number_steps          # maximum number of steps, set 0 for unlimited
    //float32 max_path_length_ratio     # maximum path length ratio computed as (current path length)/(beeline start<->goal), must be larger 1 otherwise it will be ignored

    vigir_footstep_planning_msgs::StepPlanRequestGoal action_goal;
    action_goal.plan_request = request;
    // Fill in goal here
    step_plan_request_client_->sendGoalAndWait(action_goal, ros::Duration(60.0));
    if(step_plan_request_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr result = step_plan_request_client_->getResult();
        ROS_ERROR("Got action response: [%s]", result->status.error_msg.c_str());

        if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
        {
            // add resulting plan to the top of the stack of plans
            getStepPlanList().push_back(result->step_plan);

            publishFootsteps();

        }
    }
    ROS_ERROR("Current State: %s\n", step_plan_request_client_->getState().toString().c_str());
}

void FootstepManager::addNewPlanList()
{
    // create new plan list and push it to the top of the stack
    std::vector<vigir_footstep_planning_msgs::StepPlan> plan_list;
    footstep_plans_undo_stack_.push(plan_list);
    // clear redo stack
    footstep_plans_redo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();
}

void FootstepManager::addCopyPlanList()
{
    // if there is a previous list, save it
    std::vector<vigir_footstep_planning_msgs::StepPlan> previous;
    if(footstep_plans_undo_stack_.size() > 0)
        previous = footstep_plans_undo_stack_.top();
    // create new plan list
    addNewPlanList();
    // copy plan list from previous
    if(footstep_plans_undo_stack_.size() > 1)
        getStepPlanList().insert(getStepPlanList().end(), previous.begin(), previous.end());

}

void FootstepManager::undo()
{
    if(footstep_plans_undo_stack_.size() > 0)
    {
        // add top to the redo stack
        footstep_plans_redo_stack_.push(getStepPlanList());
        // remove from undo stack
        footstep_plans_undo_stack_.pop();
    }
}

void FootstepManager::redo()
{
    if(footstep_plans_redo_stack_.size() > 0)
    {
        // add top to the undo stack
        footstep_plans_undo_stack_.push(footstep_plans_redo_stack_.top());
        // remove from redo stack
        footstep_plans_redo_stack_.pop();
    }
}

void FootstepManager::updateVisualizationMsgs()
{
    // for each step, will need to create a set of two footstep markers
    // a TEXT_VIEW_FACING and CUBE
    visualization_msgs::MarkerArray foot_array_msg;
    stepPlanToFootMarkerArray(getStepPlan(), foot_array_msg);
    footstep_array_ = foot_array_msg;

    // and body marker array
    visualization_msgs::MarkerArray body_array_msg;
    stepPlanToBodyMarkerArray(getStepPlan(), body_array_msg);
    footstep_body_array_ = body_array_msg;

    // also need to create path
    nav_msgs::Path path_msg;
    stepPlanToFootPath(getStepPlan(), path_msg);
    footstep_path_ = path_msg;
}

void FootstepManager::publishFootsteps()
{
    // update visualization msgs so we can publish them
    updateVisualizationMsgs();

    footstep_array_pub_.publish(footstep_array_);
    footstep_body_bb_array_pub_.publish(footstep_body_array_);
    footstep_path_pub_.publish(footstep_path_);

    // and also publish the footstep list
    publishFootstepList();
}

void FootstepManager::publishFootstepList()
{
    flor_ocs_msgs::OCSFootstepList list;
    for(int i = 0; i < getStepPlan().steps.size(); i++)
    {
        list.footstep_id_list.push_back(getStepPlan().steps[i].step_index);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/world";
        pose.header.stamp = ros::Time::now();
        pose.pose = getStepPlan().steps[i].foot.pose;
        list.pose.push_back(pose);
    }
    footstep_list_pub_.publish(list);
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_footstep_manager, FootstepManager, ocs_footstep::FootstepManager, nodelet::Nodelet);
