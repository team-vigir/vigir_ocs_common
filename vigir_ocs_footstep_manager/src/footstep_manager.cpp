#include "footstep_manager.h"

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/Foot.h>
#include <vigir_footstep_planning_msgs/EditStepService.h>

#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>

#include <sensor_msgs/PointCloud2.h>

namespace ocs_footstep
{
void FootstepManager::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    
    foot_pose_transformer_.reset(new vigir_footstep_planning::FootPoseTransformer(nh));

    // TODO: Get them from footstep planner
    nh.param("foot/size/x", foot_size.x, 0.26);
    nh.param("foot/size/y", foot_size.y, 0.13);
    nh.param("foot/size/z", foot_size.z, 0.05);

    nh.param("upper_body/size/x", upper_body_size.x, 0.7);
    nh.param("upper_body/size/y", upper_body_size.y, 1.1);
    nh.param("upper_body/size/z", upper_body_size.z, 0.0);
    nh.param("upper_body/origin_shift/x", upper_body_origin_shift.x, 0.0);
    nh.param("upper_body/origin_shift/y", upper_body_origin_shift.y, 0.0);
    nh.param("upper_body/origin_shift/z", upper_body_origin_shift.z, 0.0);

    // planner will always create new plans if this is unchanged
    start_step_index_ = -1;

    // initialize step plan list
    addNewPlanList();

    // creates publishers and subscribers for the interaction loop
    footstep_list_pub_                       = nh.advertise<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, false );
    footstep_update_sub_                     = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/step_update", 1, &FootstepManager::processFootstepPoseUpdate, this );
    footstep_undo_req_sub_                   = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/undo", 1, &FootstepManager::processUndoRequest, this );
    footstep_has_undo_pub_                   = nh.advertise<std_msgs::UInt8>( "/flor/ocs/footstep/undos_available", 1, false );
    footstep_redo_req_sub_                   = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/redo", 1, &FootstepManager::processRedoRequest, this );
    footstep_has_redo_pub_                   = nh.advertise<std_msgs::UInt8>( "/flor/ocs/footstep/redos_available", 1, false );
    footstep_start_index_pub_                = nh.subscribe<std_msgs::Int32>( "/flor/ocs/footstep/set_start_index", 1, &FootstepManager::processSetStartIndex, this );
    footstep_execute_req_sub_                = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/execute", 1, &FootstepManager::processExecuteFootstepRequest, this );
    footstep_send_ocs_plan_req_sub_          = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/send_ocs_plan", 1, &FootstepManager::processSendOCSStepPlanRequest, this );
    footstep_stitch_req_sub_                 = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/stitch", 1, &FootstepManager::processStitchPlansRequest, this );
    footstep_plan_parameters_pub_            = nh.advertise<flor_ocs_msgs::OCSFootstepPlanParameters>( "/flor/ocs/footstep/plan_parameters_feedback", 1, false );
    footstep_plan_parameters_sub_            = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanParameters>( "/flor/ocs/footstep/plan_parameters", 5, &FootstepManager::processFootstepPlanParameters, this );
    footstep_param_set_list_pub_             = nh.advertise<flor_ocs_msgs::OCSFootstepParamSetList>( "/flor/ocs/footstep/parameter_set_list", 1, false );
    footstep_param_set_selected_sub_         = nh.subscribe<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected", 5, &FootstepManager::processFootstepParamSetSelected, this );
    footstep_param_set_selected_pub_         = nh.advertise<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected_feedback", 1, false );
    footstep_param_set_selected_ocs_pub_     = nh.advertise<std_msgs::String>( "set_active_parameter_set_ocs", 1, false );

    // footstep request coming from the OCS
    footstep_goal_pose_sub_       = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_update", 1, &FootstepManager::processFootstepPlanGoalFeedback, this );
    footstep_goal_pose_pub_       = nh.advertise<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_update_feedback", 1, false );
    footstep_plan_goal_sub_       = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoal>( "/flor/ocs/footstep/plan_goal", 1, &FootstepManager::processFootstepPlanGoal, this );
    footstep_plan_request_sub_    = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/plan_request", 1, &FootstepManager::processFootstepPlanRequest, this );
    footstep_plan_update_sub_     = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanUpdate>( "/flor/ocs/footstep/plan_update", 1, &FootstepManager::processFootstepPlanUpdate, this );

    // creates publishers for visualization messages - latched because we want to be able to see planned footsteps in new stations or if something goes wrong
    footstep_array_pub_         = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_array", 1, true );
    footstep_body_bb_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_path_body_array", 1, true );
    footstep_path_pub_          = nh.advertise<nav_msgs::Path>( "/flor/ocs/footstep/path", 1, true );
    plan_goal_array_pub_        = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/plan_goal_array", 1, true );

    start_step_index_feet_pub_ = nh.advertise<vigir_footstep_planning_msgs::Feet>( "start_index_feet", 1, true );

    // ONBOARD FOOTSTEP MANAGER INTERACTION
    validate_step_plan_sub_ = nh.subscribe<std_msgs::Int8>( "/flor/ocs/footstep/validate", 1, &FootstepManager::processValidatePlanRequest, this );
    // publishers: ocs -> obfsm
    // Single step update of existing plan
    obfsm_step_update_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepUpdate>( "/vigir/footstep_manager/step_update", 1, false );
    // 3DOF pose of goal and 2 6D feet (flag whether to recalculate the 6D pose feet based on terrain) this is if operator edits feet poses
    obfsm_update_step_plan_goal_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/vigir/footstep_manager/update_step_plan_goal", 1, false );
    // 3dof (x,y,yaw) is only important part across comms, remainder will be determined by terrain on update
    obfsm_plan_goal_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepPlanGoal>( "/vigir/footstep_manager/plan_goal", 1, false );
    // Ability to change planner parameters (rarely used)
    obfsm_plan_parameters_goal_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepPlanParameters>( "/vigir/footstep_manager/plan_parameters", 1, false );
    // Selected parameter set for onboard planning (sent as index into list of names) Not through Bridge state
    obfsm_set_active_parameter_set_pub_ = nh.advertise<std_msgs::String>( "/vigir/footstep_manager/set_active_parameter_set", 1, false );
    // Step plan updated by OCS - only send 6D poses for feet and name of parameter set. See note 3
    obfsm_updated_step_plan_pub_ = nh.advertise<vigir_footstep_planning_msgs::StepPlan>( "/vigir/footstep_manager/updated_step_plan", 1, false );
    // Command to execute the plan (Use the header time stamp only with empty step plan)
    obfsm_execute_step_plan_pub_ = nh.advertise<vigir_footstep_planning_msgs::ExecuteStepPlanActionGoal>( "/vigir/footstep_manager/execute_step_plan/goal", 1, false );
    // Request to replan to existing goal with specified parameters
    obfsm_replan_request_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepPlanRequest>( "/vigir/footstep_manager/replan_request", 1, false );

    // subscribers: obfsm -> ocs
    // Action result status message (int32) from execute action (OCS or Behaviors) Not through Bridge state
    obfsm_execute_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::ExecuteStepPlanActionResult>( "/vigir/footstep_manager/execute_step_plan/result", 1, &FootstepManager::processExecuteStepPlanResult, this );
    // Latest footstep plan ready to execute or edit (only pose and risk) See notes 1,2,4
    obfsm_current_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlan>( "/vigir/footstep_manager/current_step_plan", 1, &FootstepManager::processOnboardStepPlan, this );
    // Goal pose with updated feet poses (sent in response to any action request by onboard planner whether triggered by OCS or behaviors
    obfsm_generate_feet_pose_sub_ = nh.subscribe<vigir_footstep_planning_msgs::GenerateFeetPoseActionResult>( "/vigir/footstep_manager/generate_feet_pose/result", 1, &FootstepManager::processGenerateFeetPoseResult, this );
    // Currently active parameter set for onboard planning in case behaviors changes it (sent as index into list of names) Not through Bridge state
    obfsm_active_parameter_set_sub_ = nh.subscribe<std_msgs::String>( "/vigir/footstep_manager/active_parameter_set", 1, &FootstepManager::processFootstepParamSetSelected, this );

    // subscribe to the onboard planner in case something generates behaviors on the onboard side
//    onboard_step_plan_request_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlanRequest>( "onboard_step_plan_request", 1, &FootstepManager::processOnboardStepPlanRequest, this );
//    onboard_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlan>( "onboard_step_plan", 1, &FootstepManager::processOnboardStepPlan, this );

    // also subscribe to the ocs planner
    ocs_step_plan_request_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlanRequest>( "ocs_step_plan_request", 1, &FootstepManager::processOCSStepPlanRequest, this );
    ocs_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlan>( "ocs_step_plan", 1, &FootstepManager::processOCSStepPlan, this );

    // point cloud visualization for plan request feedback
    planner_plan_request_feedback_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>( "/flor/ocs/footstep/plan_request_feedback", 1, true );

    // initialize all ros action clients
    //update feet considering intial goal
    update_feet_client_ = new UpdateFeetClient("update_feet", true);
    //plan request
    step_plan_request_client_ = new StepPlanRequestClient("step_plan_request", true);
    //edit step
    edit_step_client_ = new EditStepClient("edit_step", true);
    //stitch step plan
    stitch_step_plan_client_ = new StitchStepPlanClient("stitch_step_plan", true);
    //update step plan
    update_step_plan_client_ = new UpdateStepPlanClient("update_step_plan", true);
    //get all parameter sets
    get_all_parameter_sets_client_ = new GetAllParameterSetsClient("params/get_all_parameter_sets", true);
    //execute step plan
    execute_step_plan_client_ = new ExecuteStepPlanClient("execute_step_plan", true);
    //wait for servers to come online
    while(!update_feet_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the update_feet server to come up");
    }
    while(!step_plan_request_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the step_plan_request server to come up");
    }
    while(!edit_step_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the edit_step server to come up");
    }
    while(!stitch_step_plan_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the stitch_step_plan server to come up");
    }
    while(!update_step_plan_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the update_step_plan server to come up");
    }
    while(!get_all_parameter_sets_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the get_all_parameter_sets server to come up");
    }
    //while(!execute_step_plan_client_->waitForServer(ros::Duration(5.0)))
    //{
    //    ROS_INFO("Waiting for the execute_step_plan server to come up");
    //}

    // client for feet pose generator
    generate_feet_pose_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("generate_feet_pose");

    ROS_INFO("FootstepManager initialized!");

    // request parameter sets
    sendGetAllParameterSetsGoal();
    selected_footstep_parameter_set_ = "";

    // topic to update ocs feedback
    planner_status_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepStatus>( "/flor/ocs/footstep/status", 1, true );

    // sync status between ocs manager and onboard manager
    sync_status_pub_ = nh.advertise<flor_ocs_msgs::OCSFootstepSyncStatus>( "/flor/ocs/footstep/sync_status", 1, true );

    timer = nh.createTimer(ros::Duration(0.33), &FootstepManager::timerCallback, this);
}

void FootstepManager::timerCallback(const ros::TimerEvent& event)
{
    // this is just in case some view crashes or a new one opens somewhere.
    this->publishFootstepList();
    this->publishFootstepParameterSetList();
    this->publishSyncStatus();
}

void FootstepManager::processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg)
{
    // find step in the plan
    vigir_footstep_planning_msgs::Step step;

    // if it didn't find a step with the requested step_index, return
    unsigned int step_plan_index;
    if(!findStep(msg->footstep_id, step, step_plan_index))
        return;

    // update pose
    step.foot.pose = msg->pose.pose;

    // now we know we have edited steps for this plan
    if(updated_steps_.find(getStepPlan().header.stamp) == updated_steps_.end())
        updated_steps_[getStepPlan().header.stamp] = std::set<unsigned char>();
    updated_steps_[getStepPlan().header.stamp].insert(msg->footstep_id);

    // send goal
    sendEditStepGoal(getStepPlanList()[step_plan_index], step);
}

void FootstepManager::processUndoRequest(const std_msgs::Int8::ConstPtr& msg)
{
    if(footstep_plans_undo_stack_.size() > 0)
    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // add top to the redo stack
        footstep_plans_redo_stack_.push(getStepPlanList());
        // remove from undo stack
        footstep_plans_undo_stack_.pop();

        // publish everything again
        publishFootsteps();
    }
}

void FootstepManager::processRedoRequest(const std_msgs::Int8::ConstPtr& msg)
{
    if(footstep_plans_redo_stack_.size() > 0)
    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // add top to the undo stack
        footstep_plans_undo_stack_.push(footstep_plans_redo_stack_.top());
        // remove from redo stack
        footstep_plans_redo_stack_.pop();

        // publish everything again
        publishFootsteps();
    }
}

// maybe create an action for this so that I can forward feedback from controller
void FootstepManager::processSetStartIndex(const std_msgs::Int32::ConstPtr &msg)
{
    start_step_index_ = msg->data;

    publishFootsteps();

    vigir_footstep_planning_msgs::Feet start_feet;
    // if it's not starting from 0 or clearing step plan starting foot
    if(start_step_index_ > 0)
    {
        bool found_steps = false;

        vigir_footstep_planning_msgs::Step step_1;
        // if it doesn't find this find a step with the requested step_index, return
        unsigned int step_plan_index_1;
        found_steps = findStep(start_step_index_-1, step_1, step_plan_index_1);

        vigir_footstep_planning_msgs::Step step_2;
        // if it doesn't find this find a step with the requested step_index, return
        unsigned int step_plan_index_2;
        found_steps = findStep(start_step_index_, step_2, step_plan_index_2);

        if(found_steps)
        {
            // populate start_feet
            start_feet.left = step_1.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? step_1.foot : step_2.foot;
            start_feet.right = step_1.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? step_2.foot : step_1.foot;
            start_feet.header = start_feet.left.header;
        }
    } //else it just sends uninitialized start position to reset
    start_step_index_feet_pub_.publish(start_feet);
}

void FootstepManager::sendEditedSteps()
{
    last_validated_step_plan_stamp_ = getStepPlan().header.stamp;

    // need to send all updated steps
    for(std::set<unsigned char>::iterator it = updated_steps_[last_onboard_step_plan_stamp_].begin(); it != updated_steps_[last_onboard_step_plan_stamp_].end(); ++it)
    {
        // only send the message if we can find the plan/step containing the edited step (should never fail if it gets to this point)
        vigir_footstep_planning_msgs::Step step;
        unsigned int step_plan_index;
        if(findStep(*it, step, step_plan_index))
        {
            vigir_footstep_planning_msgs::Step step_copy = step;
            foot_pose_transformer_->transformToRobotFrame(step_copy);

            flor_ocs_msgs::OCSFootstepUpdate cmd;
            cmd.footstep_id = *it;
            cmd.pose.pose = step_copy.foot.pose;
            cmd.pose.header.stamp = last_validated_step_plan_stamp_;
            cmd.pose.header.frame_id = "/world";
            obfsm_step_update_pub_.publish(cmd);
        }
    }

    // only need to send this once
    updated_steps_.clear();
}

void FootstepManager::sendCurrentStepPlan()
{
    getStepPlan().header.stamp = ros::Time::now(); // since this is a force-overwrite of the onboard step plan, create new timestamp

    vigir_footstep_planning_msgs::StepPlan step_plan_copy = getStepPlan();
    foot_pose_transformer_->transformToRobotFrame(step_plan_copy);
    step_plan_copy.header.frame_id = "/world";

    obfsm_updated_step_plan_pub_.publish(step_plan_copy);

    // overwriting everything, set local timestamps
    last_validated_step_plan_stamp_ = step_plan_copy.header.stamp;
    goal_.header.stamp = step_plan_copy.header.stamp;
    goal_pose_.header.stamp = step_plan_copy.header.stamp;

    // only need to send this once
    updated_steps_.clear();
}

void FootstepManager::sendStepPlanGoal()
{
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    last_validated_step_plan_stamp_ = ros::Time::now();

    flor_ocs_msgs::OCSFootstepPlanGoal cmd;
    cmd.goal_pose = goal_pose_;
    cmd.goal_pose.header.frame_id = "/world";
    cmd.goal_pose.header.stamp = last_validated_step_plan_stamp_;
    obfsm_plan_goal_pub_.publish(cmd);
}

void FootstepManager::sendStepPlanGoalFeet()
{
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    last_validated_step_plan_stamp_ = ros::Time::now();

    vigir_footstep_planning_msgs::Feet goal_copy = goal_;
    foot_pose_transformer_->transformToRobotFrame(goal_copy);

    flor_ocs_msgs::OCSFootstepPlanGoalUpdate cmd;
    cmd.goal_pose = goal_pose_;
    cmd.goal_pose.header.frame_id = "/world";
    cmd.goal_pose.header.stamp = last_validated_step_plan_stamp_;
    cmd.left_foot.pose = goal_copy.left.pose;
    cmd.left_foot.header.frame_id = "/world";
    cmd.left_foot.header.stamp = last_validated_step_plan_stamp_;
    cmd.right_foot.pose = goal_copy.right.pose;
    cmd.right_foot.header.frame_id = "/world";
    cmd.right_foot.header.stamp = last_validated_step_plan_stamp_;
    obfsm_update_step_plan_goal_pub_.publish(cmd);
}

void FootstepManager::processValidatePlanRequest(const std_msgs::Int8::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // ONBOARDFOOTSTEPMANAGER EXPECTS EVERYTHING TO BE IN ROBOT FRAME

    // if the plan we're editing is an onboard plan and we have edited steps for that plan, send the step
    if(updated_steps_.find(getStepPlan().header.stamp) != updated_steps_.end())
    {
        if(getStepPlan().header.stamp.nsec == last_onboard_step_plan_stamp_.nsec && getStepPlan().header.stamp.sec == last_onboard_step_plan_stamp_.sec)
        {
            sendEditedSteps();
        }
        // if not and we have edited steps, we have to send the entire plan
        else
        {
            sendCurrentStepPlan();
        }
    }
    // if the goal has been updated
    else if(updated_goal_)
    {
        sendStepPlanGoalFeet();
    }
    // only send 3dof goal if it hasn't been modified
    else
    {
        sendStepPlanGoal();
    }

}

void FootstepManager::processExecuteFootstepRequest(const std_msgs::Int8::ConstPtr& msg)
{
    sendExecuteStepPlanGoal();

    // need to move the following to sendExecuteStepPlanGoal
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // need to make sure we only have one step plan, and that plan has steps
    if(getStepPlanList().size() != 1 || !getStepPlan().steps.size())
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute empty plan or multiple plans (stitch first).";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // also need to make sure this plan hasn't been sent before
    if(last_executed_step_plan_stamp_ == getStepPlan().header.stamp)
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute same plan twice.";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // save the last plan executed timestamp
    last_executed_step_plan_stamp_ = getStepPlan().header.stamp;

    // Fill in goal here
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal;
    action_goal.step_plan = getStepPlan();

    //convert transform to ankle for planner, might be redundant here?
    foot_pose_transformer_->transformToRobotFrame(action_goal.step_plan);
}

void FootstepManager::processSendOCSStepPlanRequest(const std_msgs::Int8::ConstPtr& msg)
{
    sendCurrentStepPlan();
}

void FootstepManager::processStitchPlansRequest(const std_msgs::Int8::ConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    sendStitchStepPlanGoal(getStepPlanList());
}

void FootstepManager::processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg)
{
    if(selected_footstep_parameter_set_ != msg->data || selected_footstep_parameter_set_ == "")
    {
        selected_footstep_parameter_set_ = msg->data;
        footstep_param_set_selected_pub_.publish(*msg);
        footstep_param_set_selected_ocs_pub_.publish(*msg);
        obfsm_set_active_parameter_set_pub_.publish(*msg);
    }
}

void FootstepManager::processFootstepPlanParameters(const flor_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg)
{
    flor_ocs_msgs::OCSFootstepPlanParameters new_planner_config = *msg;
    if(planner_config_.edit_mode != new_planner_config.edit_mode ||
       planner_config_.max_steps != new_planner_config.max_steps ||
       planner_config_.max_time != new_planner_config.max_time ||
       planner_config_.path_length_ratio != new_planner_config.path_length_ratio ||
       planner_config_.use_3d_planning != new_planner_config.use_3d_planning)
    {
        planner_config_ = new_planner_config;
        footstep_plan_parameters_pub_.publish(planner_config_);
        obfsm_plan_parameters_goal_pub_.publish(planner_config_);
    }
}

void FootstepManager::processExecuteStepPlanResult(vigir_footstep_planning_msgs::ExecuteStepPlanResult result)
{
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = result.status.status;
    planner_status.status_msg = "Footstep execution status: "+boost::lexical_cast<std::string>(result.status.status);
    planner_status_pub_.publish(planner_status);
//  cleanStacks();
//  publishFootsteps();
//  publishFootstepList();
}

void FootstepManager::processExecuteStepPlanResult(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResult::ConstPtr& msg)
{
    processExecuteStepPlanResult(msg->result);
}

void FootstepManager::processUpdateFeetResult(vigir_footstep_planning_msgs::UpdateFeetResult result)
{
    // update the goal feet
    vigir_footstep_planning_msgs::Feet goal = result.feet;

    //convert to sole for visualization
    foot_pose_transformer_->transformToPlannerFrame(goal);

    processNewStepPlanGoal(goal);
}

void FootstepManager::processUpdateFeetResult(const vigir_footstep_planning_msgs::UpdateFeetActionResult::ConstPtr& msg)
{
    processUpdateFeetResult(msg->result);
}

void FootstepManager::processGenerateFeetPoseResult(const vigir_footstep_planning_msgs::GenerateFeetPoseActionResult::ConstPtr& msg)
{
    // check result
    if (vigir_footstep_planning::hasError(msg->result.status))
    {
      ROS_ERROR("Error occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(msg->result.status).c_str());
      return;
    }
    else if (vigir_footstep_planning::hasWarning(msg->result.status))
    {
      ROS_ERROR("Warning occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(msg->result.status).c_str());
      return;
    }

    goal_ = msg->result.feet;

    // since feet poses are reported in robot feet frame (ankle), transform from ankle to sole
    foot_pose_transformer_->transformToPlannerFrame(goal_);

    // need to send visualization message
    updateGoalVisMsgs();

    plan_goal_array_pub_.publish(footstep_goal_array_);

    // update goal pose - should I be doing this?
    goal_pose_.pose.position.x = (goal_.left.pose.position.x+goal_.right.pose.position.x)/2.0;
    goal_pose_.pose.position.y = (goal_.left.pose.position.y+goal_.right.pose.position.y)/2.0;
    goal_pose_.pose.position.z = (goal_.left.pose.position.z+goal_.right.pose.position.z)/2.0;
    Ogre::Quaternion q1(goal_.left.pose.orientation.w,goal_.left.pose.orientation.x,goal_.left.pose.orientation.y,goal_.left.pose.orientation.z);
    Ogre::Quaternion q2(goal_.right.pose.orientation.w,goal_.right.pose.orientation.x,goal_.right.pose.orientation.y,goal_.right.pose.orientation.z);
    Ogre::Quaternion qr = Ogre::Quaternion::Slerp(0.5,q1,q2);
    goal_pose_.pose.orientation.w = qr.w;
    goal_pose_.pose.orientation.x = qr.x;
    goal_pose_.pose.orientation.y = qr.y;
    goal_pose_.pose.orientation.z = qr.z;

    // and update the interactive markers
    publishGoalMarkerFeedback();
}

void FootstepManager::feetToFootMarkerArray(vigir_footstep_planning_msgs::Feet& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    foot_array_msg.markers.clear();

    for(int i = 0; i < 2; i++)
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.6;
        color.a = 0.5;

        visualization_msgs::Marker marker;
        vigir_footstep_planning::msgs::footToFootMarker(i ? input.right : input.left, foot_size, color, marker);
        marker.id = foot_array_msg.markers.size();
        marker.ns = std::string("footstep goal");

        foot_array_msg.markers.push_back(marker);

        // add text
        marker.id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = i ? "R" : "L";
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

void FootstepManager::stepPlanToFootMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    foot_array_msg.markers.clear();

    for(int i = 0; i < input.size(); i++)
    {
        // get the parameter set used for this step plan
        for(int j = 0; j < input[i].steps.size(); j++)
        {
            float risk = input[i].steps[j].valid ? input[i].steps[j].risk/0.5 : 1.0;
            risk = risk < 0.0 ? 0.0 : (risk > 1.0 ? 1.0 : risk); //clamp

            std_msgs::ColorRGBA color;
            color.r = risk * 0.6;
            color.g = (1.0 - risk) * 0.6;
            color.b = 0.0;
            color.a = 0.5;

            visualization_msgs::Marker marker;
            vigir_footstep_planning::msgs::stepToFootMarker(input[i].steps[j], foot_size, color, marker);
            marker.id = foot_array_msg.markers.size();
            marker.ns = std::string("footstep");

            foot_array_msg.markers.push_back(marker);

            // add text
            marker.id++;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.text = (start_step_index_ == input[i].steps[j].step_index ? "*" : "") + boost::lexical_cast<std::string>(input[i].steps[j].step_index) + (start_step_index_ == input[i].steps[j].step_index ? "*" : "");
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
}

void FootstepManager::stepPlanToBodyMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& body_array_msg)
{
    vigir_footstep_planning_msgs::Feet feet;
    feet.header = input[0].steps[0].foot.header;

    visualization_msgs::Marker marker;
    body_array_msg.markers.clear();
    for(int i = 0; i < input.size(); i++)
    {
        for(int j = 0; j < input[i].steps.size(); j++)
        {
            vigir_footstep_planning_msgs::Step& step = input[i].steps[j];
            if (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT)
              feet.left = step.foot;
            else
              feet.right = step.foot;

            if (i == 0 && j == 0)
              continue;

            // need something better than this to compute color, but for now we only have two known modes (green, yellow)
            std_msgs::ColorRGBA color;
            color.r = input[i].mode == 4 ? 0.4 : 0.0; // TO DO: READ PARAMETER MAP FOR MODE -> COLOR
            color.g = input[i].mode == 4 ? 0.4 : 0.0;
            color.b = input[i].mode == 4 ? 0.0 : 0.4;
            color.a = 0.2;

            vigir_footstep_planning::msgs::feetToUpperBodyMarker(feet, upper_body_size, upper_body_origin_shift, color, marker, true);
            marker.id = body_array_msg.markers.size();

            body_array_msg.markers.push_back(marker);
        }
    }
}

void FootstepManager::stepPlanToFootPath(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, nav_msgs::Path& foot_path_msg)
{
    foot_path_msg.header = input[0].header;
    foot_path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < input.size(); i++)
    {
        nav_msgs::Path path;
        vigir_footstep_planning::msgs::stepPlanToPath(input[i], path);
        foot_path_msg.poses.insert(foot_path_msg.poses.end(), path.poses.begin(), path.poses.end());
    }
}

void FootstepManager::processFootstepPlanGoal(const flor_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal)
{
    // need to reset the updated goal flag since this goal hasn't been changed
    updated_goal_ = false;

    // uses goal pose to request
    goal_pose_ = plan_goal->goal_pose;

    // and then the end feet poses
    calculateGoal();

    // then update feet using the footstep planner
    sendUpdateFeetGoal(goal_); /// TODO: Could be already done in calculateGoal within one single request
}

void FootstepManager::processFootstepPlanGoalFeedback(const flor_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal)
{
    // only one that sends feedback is the manager, so return to avoid infinite loop
    if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::FEEDBACK)
        return;

    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    // update time stamp, as it will require a new validate
    goal_.header.stamp = ros::Time::now();
    goal_pose_.header.stamp = goal_.header.stamp;

    if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::GOAL)
    {
        // need to update feet poses, so we first find the difference between the old and the new pose
        Ogre::Vector3 p_old(goal_pose_.pose.position.x,
                            goal_pose_.pose.position.y,
                            goal_pose_.pose.position.z);
        Ogre::Vector3 p_new(plan_goal->goal_pose.pose.position.x,
                            plan_goal->goal_pose.pose.position.y,
                            plan_goal->goal_pose.pose.position.z);
        Ogre::Quaternion q_old( goal_pose_.pose.orientation.w, // need the conjugate (w,-x,-y,-z) of the old one
                               -goal_pose_.pose.orientation.x,
                               -goal_pose_.pose.orientation.y,
                               -goal_pose_.pose.orientation.z);
        Ogre::Quaternion q_new(plan_goal->goal_pose.pose.orientation.w,
                               plan_goal->goal_pose.pose.orientation.x,
                               plan_goal->goal_pose.pose.orientation.y,
                               plan_goal->goal_pose.pose.orientation.z);
        Ogre::Quaternion d_quat = q_old * q_new;

        // update left foot
        {
        // need to calculate position relative to old goal pose
        Ogre::Vector3 d(goal_.left.pose.position.x-goal_pose_.pose.position.x,
                        goal_.left.pose.position.y-goal_pose_.pose.position.y,
                        goal_.left.pose.position.z-goal_pose_.pose.position.z);
        d = p_new + d_quat * d;

        goal_.left.pose.position.x = d.x;
        goal_.left.pose.position.y = d.y;
        goal_.left.pose.position.z = d.z;

        // and we need to multiply quaternions
        Ogre::Quaternion q(goal_.left.pose.orientation.w,
                           goal_.left.pose.orientation.x,
                           goal_.left.pose.orientation.y,
                           goal_.left.pose.orientation.z);
        q = d_quat * q;

        goal_.left.pose.orientation.w = q.w;
        goal_.left.pose.orientation.x = q.x;
        goal_.left.pose.orientation.y = q.y;
        goal_.left.pose.orientation.z = q.z;
        }
        // update right foot
        {
        // need to calculate position relative to old goal pose
        Ogre::Vector3 d(goal_.right.pose.position.x-goal_pose_.pose.position.x,
                        goal_.right.pose.position.y-goal_pose_.pose.position.y,
                        goal_.right.pose.position.z-goal_pose_.pose.position.z);
        d = p_new + d_quat * d;

        goal_.right.pose.position.x = d.x;
        goal_.right.pose.position.y = d.y;
        goal_.right.pose.position.z = d.z;
        // and we need to multiply quaternions
        Ogre::Quaternion q(goal_.right.pose.orientation.w,
                           goal_.right.pose.orientation.x,
                           goal_.right.pose.orientation.y,
                           goal_.right.pose.orientation.z);
        q = d_quat * q;

        goal_.right.pose.orientation.w = q.w;
        goal_.right.pose.orientation.x = q.x;
        goal_.right.pose.orientation.y = q.y;
        goal_.right.pose.orientation.z = q.z;
        }

        // updates internal goal pose
        goal_pose_ = plan_goal->goal_pose;
    }
    else if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::LEFT)
    {
        goal_.left.pose.position.x = plan_goal->left_foot.pose.position.x;
        goal_.left.pose.position.y = plan_goal->left_foot.pose.position.y;
        goal_.left.pose.position.z = plan_goal->left_foot.pose.position.z;
        goal_.left.pose.orientation.w = plan_goal->left_foot.pose.orientation.w;
        goal_.left.pose.orientation.x = plan_goal->left_foot.pose.orientation.x;
        goal_.left.pose.orientation.y = plan_goal->left_foot.pose.orientation.y;
        goal_.left.pose.orientation.z = plan_goal->left_foot.pose.orientation.z;

        // make sure we know the goal has been updated for the current plan
        updated_goal_ = true;
    }
    else if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::RIGHT)
    {
        goal_.right.pose.position.x = plan_goal->right_foot.pose.position.x;
        goal_.right.pose.position.y = plan_goal->right_foot.pose.position.y;
        goal_.right.pose.position.z = plan_goal->right_foot.pose.position.z;
        goal_.right.pose.orientation.w = plan_goal->right_foot.pose.orientation.w;
        goal_.right.pose.orientation.x = plan_goal->right_foot.pose.orientation.x;
        goal_.right.pose.orientation.y = plan_goal->right_foot.pose.orientation.y;
        goal_.right.pose.orientation.z = plan_goal->right_foot.pose.orientation.z;

        // make sure we know the goal has been updated for the current plan
        updated_goal_ = true;
    }

    // need to update feet poses
    updateGoalVisMsgs();

    // send
    plan_goal_array_pub_.publish(footstep_goal_array_);

    // update the interactive markers
    publishGoalMarkerFeedback();

    // then update feet pose using the footstep planner
    sendUpdateFeetGoal(goal_);
}

void FootstepManager::calculateGoal()
{
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header = goal_pose_.header;
    feet_pose_service.request.request.pose = goal_pose_.pose;
    // this is for 3D interaction
    feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT_Z;
    // this is for 2D interaction
    //feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_MOVE;

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("Can't call 'FeetPoseGenerator'!");
      return;
    }

    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    goal_ = feet_pose_service.response.feet;

    // since feet poses are reported in robot feet frame (ankle), transform from ankle to sole
    foot_pose_transformer_->transformToPlannerFrame(goal_);
}

void FootstepManager::processFootstepPlanUpdate(const flor_ocs_msgs::OCSFootstepPlanUpdate::ConstPtr& msg)
{
    // basic error checking
    if(getStepPlanList().size() == 0 || getStepPlan().steps.size() == 0 || msg->step_plan_id >= getStepPlanList().size())
        return;

    // update the stepplan that ends at the marker the marker
    sendUpdateStepPlanGoal(getStepPlanList()[msg->step_plan_id]);

    // if there is a stepplan starting at the marker, we need to update it as well
    if(msg->step_plan_id+1 < getStepPlanList().size())
    {
        // WHEN I UPDATE STEPPLAN MARKERS IN BETWEEN STEPPLANS, I WILL HAVE TO ADD THE LAST STEP OF PLAN A TO PLAN B
        // this step is deleted when the plan is received because it's already part of the previous step plan
        vigir_footstep_planning_msgs::StepPlan next_plan;
        next_plan = getStepPlanList()[msg->step_plan_id+1];
        next_plan.steps.insert(next_plan.steps.begin(), getStepPlanList()[msg->step_plan_id].steps.back());
        sendUpdateStepPlanGoal(next_plan);
    }
}

void FootstepManager::processFootstepPlanRequest(const std_msgs::Int8::ConstPtr& plan_request)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(getStepPlanList().size() == 0 || getStepPlan().steps.size() == 0 || start_step_index_ > getStepPlan().steps.back().step_index)
        start_step_index_ = -1;

    if(start_step_index_ == -1)//if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN)
    {
        // request a completely new plan starting from the robot position
        requestStepPlanFromRobot();
    }
    else if(start_step_index_ == getStepPlan().steps.back().step_index)//if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::CONTINUE_CURRENT_PLAN)
    {
        // get last step
        vigir_footstep_planning_msgs::Step step = getStepPlan().steps.back();
        // request a footstep plan starting from the last step
        requestStepPlanFromStep(step);
    }
    else //if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::CONTINUE_FROM_STEP)
    {
        vigir_footstep_planning_msgs::Step step;
        // unlikely, but if start index is the very first step
        if(start_step_index_ == 0)
        {
            if(getStepPlan().steps.size() > 1)
                step = getStepPlan().steps[0];
            else
                return;
        }
        else
        {
            // if it didn't find a step with the requested step_index, return
            unsigned int step_plan_index;
            if(!findStep(start_step_index_-1, step, step_plan_index))
                return;
        }

        requestStepPlanFromStep(step);
    }
}

void FootstepManager::requestStepPlanFromRobot()
{
    // get start feet pose
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header.frame_id = "/world";
    feet_pose_service.request.request.header.stamp = ros::Time::now();
    feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT;

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("Can't call 'FeetPoseGenerator'!");
      return;
    }

    // check result
    if (vigir_footstep_planning::hasError(feet_pose_service.response.status))
    {
      ROS_ERROR("Error occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      return;
    }
    else if (vigir_footstep_planning::hasWarning(feet_pose_service.response.status))
    {
      ROS_ERROR("Warning occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      return;
    }

    // since feet is reported in robot frame (ankle), need to transform it to planner frame
    foot_pose_transformer_->transformToPlannerFrame(feet_pose_service.response.feet);

    sendStepPlanRequestGoal(feet_pose_service.response.feet, goal_);
}

void FootstepManager::requestStepPlanFromStep(vigir_footstep_planning_msgs::Step& step)
{
    // then we need to find the next step after the starting one
    vigir_footstep_planning_msgs::Step next_step;
    unsigned int step_plan_index;

    if(!findStep(step.step_index+1, next_step, step_plan_index))
        return;

    // first we get the start feet poses based on the selected step
    vigir_footstep_planning_msgs::Feet start;
    start.header = step.foot.header;
    start.left  = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? step.foot : next_step.foot);
    start.right = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ? step.foot : next_step.foot);

    // calculate
    unsigned char start_foot;
    if(next_step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT)
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::LEFT;
    else
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::RIGHT;

    sendStepPlanRequestGoal(start, goal_, next_step.step_index, start_foot);
}

void FootstepManager::cleanMarkerArray(visualization_msgs::MarkerArray& old_array, visualization_msgs::MarkerArray& new_array)
{
    // update visualization message of the new step plan removing unused markers
    for(int i = 0; i < old_array.markers.size(); i++)
    {
        // add delete action to the marker if it doesn't exist in the new plan, ignore if it was already deleted
        if(old_array.markers[i].action != visualization_msgs::Marker::DELETE && new_array.markers.size() == i)
        {
            visualization_msgs::Marker marker;
            marker = old_array.markers[i];
            marker.action = visualization_msgs::Marker::DELETE;
            new_array.markers.push_back(marker);
        }
    }
}

void FootstepManager::updateStepPlanVisMsgs()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // for each step, will need to create a set of two footstep markers
    // a TEXT_VIEW_FACING and CUBE
    visualization_msgs::MarkerArray foot_array_msg;
    if(getStepPlanList().size() > 0)
        stepPlanToFootMarkerArray(getStepPlanList(), foot_array_msg);
    cleanMarkerArray(footstep_array_,foot_array_msg);
    footstep_array_ = foot_array_msg;

    // and body marker array
    visualization_msgs::MarkerArray body_array_msg;
    if(getStepPlanList().size() > 0)
        stepPlanToBodyMarkerArray(getStepPlanList(), body_array_msg);
    cleanMarkerArray(footstep_body_array_,body_array_msg);
    footstep_body_array_ = body_array_msg;

    // also need to create path
    nav_msgs::Path path_msg;
    if(getStepPlanList().size() > 0)
        stepPlanToFootPath(getStepPlanList(), path_msg);
    else
        path_msg.header.frame_id = "/world"; // needed for the message to be processed
    footstep_path_ = path_msg;
}

void FootstepManager::updateGoalVisMsgs()
{
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    // for each step, will need to create a set of two footstep markers
    // a TEXT_VIEW_FACING and CUBE
    visualization_msgs::MarkerArray foot_array_msg;
    feetToFootMarkerArray(goal_, foot_array_msg);
    footstep_goal_array_ = foot_array_msg;
}

void FootstepManager::publishGoalMarkerClear()
{
    flor_ocs_msgs::OCSFootstepPlanGoalUpdate cmd;
    cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::CLEAR;
    footstep_goal_pose_pub_.publish(cmd);
}

void FootstepManager::publishGoalMarkerFeedback()
{
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    flor_ocs_msgs::OCSFootstepPlanGoalUpdate cmd;
    cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::FEEDBACK;
    // for stepplan goal
    cmd.goal_pose = goal_pose_;
    // and for goal feetposes
    cmd.left_foot.pose = goal_.left.pose;
    cmd.right_foot.pose = goal_.right.pose;
    footstep_goal_pose_pub_.publish(cmd);
}

void FootstepManager::publishSyncStatus()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    flor_ocs_msgs::OCSFootstepSyncStatus synced;

    // sync status
    if(getStepPlanList().size() == 0)
        synced.status = flor_ocs_msgs::OCSFootstepSyncStatus::EMPTY_PLAN_LIST;
    else if(getStepPlan().steps.size() == 0)
        synced.status = flor_ocs_msgs::OCSFootstepSyncStatus::EMPTY_PLAN;
    else
    {
        // there is a step plan, so safe to access it
        if(goal_.header.stamp.toSec() != getStepPlan().header.stamp.toSec()) // goal and step plan headers don't match
            synced.status = flor_ocs_msgs::OCSFootstepSyncStatus::PROCESSING_OCS_PLAN;
        else if(updated_steps_.find(getStepPlan().header.stamp) != updated_steps_.end() ||
                last_validated_step_plan_stamp_.toSec() != getStepPlan().header.stamp.toSec())
            synced.status = flor_ocs_msgs::OCSFootstepSyncStatus::NEED_PLAN_VALIDATION;
        else
            synced.status = flor_ocs_msgs::OCSFootstepSyncStatus::SYNCED;

        // validation option based on status
        if(updated_steps_.find(getStepPlan().header.stamp) != updated_steps_.end())
        {
            if(getStepPlan().header.stamp.nsec == last_onboard_step_plan_stamp_.nsec && getStepPlan().header.stamp.sec == last_onboard_step_plan_stamp_.sec)
            {
                synced.validate_mode = flor_ocs_msgs::OCSFootstepSyncStatus::EDITED_STEPS;
            }
            // if not and we have edited steps, we have to send the entire plan
            else
            {
                synced.validate_mode = flor_ocs_msgs::OCSFootstepSyncStatus::CURRENT_PLAN;
            }
        }
        // if the goal has been updated
        else if(updated_goal_)
        {
            synced.validate_mode = flor_ocs_msgs::OCSFootstepSyncStatus::GOAL_FEET;
        }
        // only send 3dof goal if it hasn't been modified
        else
        {
            synced.validate_mode = flor_ocs_msgs::OCSFootstepSyncStatus::GOAL;
        }
    }

    sync_status_pub_.publish(synced);
}

void FootstepManager::publishFootsteps()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // update visualization msgs so we can publish them
    updateStepPlanVisMsgs();

    footstep_array_pub_.publish(footstep_array_);
    footstep_body_bb_array_pub_.publish(footstep_body_array_);
    footstep_path_pub_.publish(footstep_path_);

    // and also publish the footstep list
    publishFootstepList();
}

void FootstepManager::publishFootstepList()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    flor_ocs_msgs::OCSFootstepList list;
    for(int i = 0; i < getStepPlanList().size(); i++)
    {
        for(int j = 0; j < getStepPlanList()[i].steps.size(); j++)
        {
            list.footstep_id_list.push_back(getStepPlanList()[i].steps[j].step_index);
            list.step_plan_id_list.push_back(i);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/world";
            pose.header.stamp = ros::Time::now();
            pose.pose = getStepPlanList()[i].steps[j].foot.pose;
            list.pose.push_back(pose);
        }
    }
    footstep_list_pub_.publish(list);

    // also publish undo/redo status
    std_msgs::UInt8 cmd;
    cmd.data = footstep_plans_undo_stack_.size()-1; // -1 since we don't want to be able to undo when we only have one plan in the stack (it will make it empty and crash)
    footstep_has_undo_pub_.publish(cmd);
    cmd.data = footstep_plans_redo_stack_.size();
    footstep_has_redo_pub_.publish(cmd);
}

void FootstepManager::publishFootstepParameterSetList()
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);

    flor_ocs_msgs::OCSFootstepParamSetList cmd;
    for(int i = 0; i < footstep_parameter_set_list_.size(); i++)
    {
        cmd.param_set.push_back(footstep_parameter_set_list_[i].name.data);
    }
    footstep_param_set_list_pub_.publish(cmd);
}

// action goal for updatefeet
void FootstepManager::sendUpdateFeetGoal(vigir_footstep_planning_msgs::Feet feet)
{
    // convert to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(feet);

    // Fill in goal here
    vigir_footstep_planning_msgs::UpdateFeetGoal action_goal;
    action_goal.feet = feet;
    action_goal.update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D;

    // and send it to the server
    if(update_feet_client_->isServerConnected())
    {
        update_feet_client_->sendGoal(action_goal,
                                      boost::bind(&FootstepManager::doneUpdateFeet, this, _1, _2),
                                      boost::bind(&FootstepManager::activeUpdateFeet, this),
                                      boost::bind(&FootstepManager::feedbackUpdateFeet, this, _1));
    }
    else
    {
        ROS_INFO("UpdateFeet: Server not connected!");
    }
}

// action callbacks for goal feet pose update request
void FootstepManager::activeUpdateFeet()
{
    ROS_INFO("UpdateFeet: Status changed to active.");
}

void FootstepManager::feedbackUpdateFeet(const vigir_footstep_planning_msgs::UpdateFeetFeedbackConstPtr& feedback)
{
    ROS_INFO("UpdateFeet: Feedback received.");
}

void FootstepManager::doneUpdateFeet(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateFeetResultConstPtr& result)
{
    ROS_INFO("UpdateFeet: Got action response.\n");

    if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("UpdateFeet: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        processUpdateFeetResult(*result);
    }
}

// action goal for StepPlanRequest
void FootstepManager::sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index, const unsigned char start_foot)
{
    vigir_footstep_planning_msgs::StepPlanRequest request;

    //convert transform to ankle for planner, but only transform start pose if start is 0
    foot_pose_transformer_->transformToRobotFrame(start);
    foot_pose_transformer_->transformToRobotFrame(goal);

    request.header.frame_id = "/world";
    request.header.stamp = goal.header.stamp;

    request.start = start;
    request.goal = goal;
    request.start_step_index = start_step_index;

    request.start_foot_selection = start_foot;

    // default planning mode is 2D, but will get that from the OCS
    if(planner_config_.use_3d_planning)
        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
    else
        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

    // need to get the following from the OCS as well
    request.max_planning_time = planner_config_.max_time;
    request.max_number_steps = planner_config_.max_steps;
    request.max_path_length_ratio = planner_config_.path_length_ratio;

    // and then use the selected parameter set
    request.parameter_set_name.data = selected_footstep_parameter_set_;

    // Fill in goal here
    vigir_footstep_planning_msgs::StepPlanRequestGoal action_goal;
    action_goal.plan_request = request;

    // and send it to the server
    if(step_plan_request_client_->isServerConnected())
    {
        ROS_INFO("StepPlanRequest: Sending action goal (%f)...",action_goal.plan_request.header.stamp.toSec());
        step_plan_request_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneStepPlanRequest, this, _1, _2),
                                            boost::bind(&FootstepManager::activeStepPlanRequest, this),
                                            boost::bind(&FootstepManager::feedbackStepPlanRequest, this, _1));
    }
    else
    {
        ROS_INFO("StepPlanRequest: Server not connected!");
    }

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Sending StepPlanRequest action goal...";
    planner_status_pub_.publish(planner_status);
}

// action callbacks for step plan request
void FootstepManager::activeStepPlanRequest()
{
    ROS_INFO("StepPlanRequest: Status changed to active.");
}

void FootstepManager::feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
    ROS_INFO("StepPlanRequest: Feedback received.");

    vigir_footstep_planning::vis::publishRecentlyVistedSteps(planner_plan_request_feedback_cloud_pub_, feedback->feedback.visited_steps, feedback->feedback.header);

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Footstep planner working (percent complete).";
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
    ROS_INFO("StepPlanRequest: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("StepPlanRequest: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        // send update to glance_hub "done"
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        if(result->step_plan.header.stamp.toSec() != last_ocs_step_plan_stamp_.toSec())
        {
            vigir_footstep_planning_msgs::StepPlan plan = result->step_plan;
            processNewStepPlan(plan);

            last_ocs_step_plan_stamp_ = result->step_plan.header.stamp;
        }
        else
        {
            ROS_INFO("StepPlanRequest: Ignoring repeated plan (%d, %d).", result->step_plan.header.stamp.sec, result->step_plan.header.stamp.nsec);
        }

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_SUCCESS;
        planner_status.status_msg = "Footstep planner done.";
        planner_status_pub_.publish(planner_status);
    }
}

// action goal for EditStep
void FootstepManager::sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step)
{
    //convert to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(step_plan);
    foot_pose_transformer_->transformToRobotFrame(step.foot);

    // Fill in goal here
    vigir_footstep_planning_msgs::EditStepGoal action_goal;
    action_goal.step_plan = step_plan;
    if(planner_config_.edit_mode)
        action_goal.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;
    else
        action_goal.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_3D;
    action_goal.edit_step.step = step;

    // and send it to the server
    if(edit_step_client_->isServerConnected())
    {
        edit_step_client_->sendGoal(action_goal,
                                    boost::bind(&FootstepManager::doneEditStep, this, _1, _2),
                                    boost::bind(&FootstepManager::activeEditStep, this),
                                    boost::bind(&FootstepManager::feedbackEditStep, this, _1));
    }
    else
    {
        ROS_INFO("EditStep: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeEditStep()
{
    ROS_INFO("EditStep: Status changed to active.");
}

void FootstepManager::feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback)
{
    ROS_INFO("EditStep: Feedback received.");
}

void FootstepManager::doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
    ROS_INFO("EditStep: Got action response.");

    if (vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("EditStep: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        if(result->step_plans.size() == 0)
        {
            ROS_ERROR("EditStep: Received no step plan.");
            return;
        }

        if(result->step_plans[0].steps.size() == 0)
        {
            ROS_ERROR("EditStep: Received empty step plan.");
            return;
        }

        // we really don't want to update the timestamp, so we save the old one
        ros::Time plan_stamp = getStepPlan().header.stamp;

        vigir_footstep_planning_msgs::EditStepResult result_copy = *result;
        //convert all step plans transforms to be relative to sole frame for visualization
        //Brian::can optimize to just transform one plan?
        for(int i=0;i<result_copy.step_plans.size();i++)
        {
            result_copy.step_plans[i].header.stamp = plan_stamp;
            foot_pose_transformer_->transformToPlannerFrame(result_copy.step_plans[i]);
        }

        // first need to figure out which step plan contains the step index used in the result
        unsigned int step_plan_index;
        findStepPlan(result_copy.step_plans[0].steps[0].step_index, step_plan_index);

        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // save the index of the step plan
        std::vector<vigir_footstep_planning_msgs::StepPlan>::iterator step_plan_it = getStepPlanList().begin()+step_plan_index;
        // remove the plan
        getStepPlanList().erase(step_plan_it);

        // and add resulting plan(s) to the list again using the previous index
        getStepPlanList().insert(getStepPlanList().begin()+step_plan_index, result_copy.step_plans.begin(), result_copy.step_plans.end());

        publishFootsteps();
    }
}

// action goal for stitchstepplan
void FootstepManager::sendStitchStepPlanGoal(std::vector<vigir_footstep_planning_msgs::StepPlan>& step_plan_list)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan_list.size() < 2)
        return;

    //convert all step plans transforms to be relative to ankle frame for planner
    for(int i=0;i<step_plan_list.size();i++)
    {
        foot_pose_transformer_->transformToRobotFrame(step_plan_list[i]);
    }

    // Fill in goal here
    vigir_footstep_planning_msgs::StitchStepPlanGoal action_goal;
    action_goal.step_plans = step_plan_list;

    // and send it to the server
    if(stitch_step_plan_client_->isServerConnected())
    {
        stitch_step_plan_client_->sendGoal(action_goal,
                                           boost::bind(&FootstepManager::doneStitchStepPlan, this, _1, _2),
                                           boost::bind(&FootstepManager::activeStitchStepPlan, this),
                                           boost::bind(&FootstepManager::feedbackStitchStepPlan, this, _1));
    }
    else
    {
        ROS_INFO("StitchStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeStitchStepPlan()
{
    ROS_INFO("StitchStepPlan: Status changed to active.");
}

void FootstepManager::feedbackStitchStepPlan(const vigir_footstep_planning_msgs::StitchStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("StitchStepPlan: Feedback received.");
}

void FootstepManager::doneStitchStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StitchStepPlanResultConstPtr& result)
{
    ROS_INFO("StitchStepPlan: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("EditStep: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // create a new plan list for our stitched step plan
        addNewPlanList();

        vigir_footstep_planning_msgs::StitchStepPlanResult result_copy = *result;
        //convert step plan transform to be relative to sole frame for visualization
        foot_pose_transformer_->transformToPlannerFrame(result_copy.step_plan);

        // add new step plan to the list
        getStepPlanList().push_back(result_copy.step_plan);

        // clear start step index
        start_step_index_ = -1;

        publishFootsteps();
    }
}

// action goal for pplan
void FootstepManager::sendUpdateStepPlanGoal(vigir_footstep_planning_msgs::StepPlan step_plan)
{
    //convert transform to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(step_plan);

    // Fill in goal here
    vigir_footstep_planning_msgs::UpdateStepPlanGoal action_goal;
    action_goal.step_plan = step_plan;
    action_goal.parameter_set_name.data = selected_footstep_parameter_set_;
    action_goal.update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_REPLAN;

    // and send it to the server
    if(update_step_plan_client_->isServerConnected())
    {
        update_step_plan_client_->sendGoal(action_goal,
                                           boost::bind(&FootstepManager::doneUpdateStepPlan, this, _1, _2),
                                           boost::bind(&FootstepManager::activeUpdateStepPlan, this),
                                           boost::bind(&FootstepManager::feedbackUpdateStepPlan, this, _1));
    }
    else
    {
        ROS_INFO("UpdateStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeUpdateStepPlan()
{
    ROS_INFO("UpdateStepPlan: Status changed to active.");
}

void FootstepManager::feedbackUpdateStepPlan(const vigir_footstep_planning_msgs::UpdateStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("UpdateStepPlan: Feedback received.");
}

void FootstepManager::doneUpdateStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result)
{
    ROS_INFO("UpdateStepPlan: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("UpdateStepPlan: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // this is reserved for validation
    }
}

// action goal for executestep
void FootstepManager::sendExecuteStepPlanGoal()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // need to make sure we only have one step plan, and that plan has steps
    if(getStepPlanList().size() != 1 || !getStepPlan().steps.size())
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute empty plan or multiple plans (stitch first).";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // also need to make sure this plan hasn't been sent before
    if(last_executed_step_plan_stamp_ == getStepPlan().header.stamp)
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute same plan twice.";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // save the last plan executed timestamp
    last_executed_step_plan_stamp_ = getStepPlan().header.stamp;

    // Fill in goal here
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal;
    action_goal.step_plan = getStepPlan();

    //convert transform to ankle for planner, might be redundant here?
    foot_pose_transformer_->transformToRobotFrame(action_goal.step_plan);

    // send it to the obfsm
    vigir_footstep_planning_msgs::ExecuteStepPlanActionGoal cmd;
    cmd.goal = action_goal;
    cmd.header.stamp = getStepPlan().header.stamp;
    obfsm_execute_step_plan_pub_.publish(cmd);

    // and send it to the server
//    if(execute_step_plan_client_->isServerConnected())
//    {
//        execute_step_plan_client_->sendGoal(action_goal,
//                                            boost::bind(&FootstepManager::doneExecuteStepPlan, this, _1, _2),
//                                            boost::bind(&FootstepManager::activeExecuteStepPlan, this),
//                                            boost::bind(&FootstepManager::feedbackExecuteStepPlan, this, _1));
//    }
//    else
//    {
//        ROS_INFO("ExecuteStepPlan: Server not connected!");
//    }
}

// action callbacks
void FootstepManager::activeExecuteStepPlan()
{
    ROS_INFO("ExecuteStepPlan: Status changed to active.");

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Execution of footstep plan active";
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("ExecuteStepPlan: Feedback received.");

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Execution feedback received: Step "+boost::lexical_cast<std::string>(feedback->last_performed_step_index)+" -> Step "+boost::lexical_cast<std::string>(feedback->currently_executing_step_index);
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
    ROS_INFO("ExecuteStepPlan: Got action response.");

    if(!(result->status.status & vigir_footstep_planning_msgs::FootstepExecutionStatus::REACHED_GOAL))
    {
        ROS_ERROR("ExecuteStepPlan: Error occured!\nExecution error code: %d", result->status.status);

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Execution error code: "+boost::lexical_cast<std::string>(result->status.status);
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        processExecuteStepPlanResult(*result);
    }
}

void FootstepManager::sendGetAllParameterSetsGoal()
{
    // Fill in goal here
    vigir_footstep_planning_msgs::GetAllParameterSetsGoal action_goal;

    // and send it to the server
    if(get_all_parameter_sets_client_->isServerConnected())
    {
        get_all_parameter_sets_client_->sendGoal(action_goal,
                                                 boost::bind(&FootstepManager::doneGetAllParameterSets, this, _1, _2),
                                                 boost::bind(&FootstepManager::activeGetAllParameterSets, this),
                                                 boost::bind(&FootstepManager::feedbackGetAllParameterSets, this, _1));
    }
    else
    {
        ROS_INFO("GetAllParameterSets: Server not connected!");
    }
}

void FootstepManager::activeGetAllParameterSets()
{
    ROS_INFO("GetAllParameterSets: Status changed to active.");
}

void FootstepManager::feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback)
{
    ROS_INFO("GetAllParameterSets: Feedback received.");
}

void FootstepManager::doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result)
{
    ROS_INFO("GetAllParameterSets: Got action response. %s", result->status.error_msg.c_str());

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("GetAllParameterSets: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(param_mutex_);

        footstep_parameter_set_list_.clear();
        for(int i = 0; i < result->param_sets.size(); i++)
            footstep_parameter_set_list_.push_back(result->param_sets[i]);

        this->publishFootstepParameterSetList();
    }
}

void FootstepManager::processNewStepPlanGoal(vigir_footstep_planning_msgs::Feet& goal)
{
    // update the goal feet
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

    goal_ = goal;

    // need to send visualization message
    updateGoalVisMsgs();

    plan_goal_array_pub_.publish(footstep_goal_array_);

    // update goal pose - should I be doing this?
    goal_pose_.pose.position.x = (goal_.left.pose.position.x+goal_.right.pose.position.x)/2.0;
    goal_pose_.pose.position.y = (goal_.left.pose.position.y+goal_.right.pose.position.y)/2.0;
    goal_pose_.pose.position.z = (goal_.left.pose.position.z+goal_.right.pose.position.z)/2.0;
    Ogre::Quaternion q1(goal_.left.pose.orientation.w,goal_.left.pose.orientation.x,goal_.left.pose.orientation.y,goal_.left.pose.orientation.z);
    Ogre::Quaternion q2(goal_.right.pose.orientation.w,goal_.right.pose.orientation.x,goal_.right.pose.orientation.y,goal_.right.pose.orientation.z);
    Ogre::Quaternion qr = Ogre::Quaternion::Slerp(0.5,q1,q2);
    goal_pose_.pose.orientation.w = qr.w;
    goal_pose_.pose.orientation.x = qr.x;
    goal_pose_.pose.orientation.y = qr.y;
    goal_pose_.pose.orientation.z = qr.z;

    // and update the interactive markers
    publishGoalMarkerFeedback();
}

void FootstepManager::processNewStepPlan(vigir_footstep_planning_msgs::StepPlan& step_plan)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    ROS_INFO("processNewStepPlan: Processing new step plan (%d, %d).", step_plan.header.stamp.sec, step_plan.header.stamp.nsec);

    if(step_plan.steps.size() == 0)
    {
        ROS_ERROR("processNewStepPlan: Received empty step plan.");
        return;
    }

    //convert to sole for visualization
    foot_pose_transformer_->transformToPlannerFrame(step_plan);

    // we only change the current step lists if we receive a response
    if(step_plan.steps[0].step_index == 0)
        // This function will create a completely new plan, so we need to add a new empty list of plans to the stack
        addNewPlanList();
    else
        // This function will add a copy of the current step plan list to the stack, so we can change it
        addCopyPlanList();

    // add resulting plan to the top of the stack of plans, removing any extra steps
    extendPlanList(step_plan);

    publishFootsteps();

    //publishGoalMarkerClear();

    // update goal pose
    vigir_footstep_planning_msgs::Feet goal;
    goal.header = getStepPlan().steps[getStepPlan().steps.size()-1].header;
    goal.left = getStepPlan().steps[getStepPlan().steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ?
                getStepPlan().steps[getStepPlan().steps.size()-1].foot :
                getStepPlan().steps[getStepPlan().steps.size()-2].foot;
    goal.right = getStepPlan().steps[getStepPlan().steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ?
                 getStepPlan().steps[getStepPlan().steps.size()-1].foot :
                 getStepPlan().steps[getStepPlan().steps.size()-2].foot;
    processNewStepPlanGoal(goal);
}

// onboard action callbacks
void FootstepManager::processOnboardStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);

    // update parameter set
    selected_footstep_parameter_set_ = step_plan_request->parameter_set_name.data;
    std_msgs::String cmd;
    cmd.data = selected_footstep_parameter_set_;
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepManager::processOnboardStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    ROS_INFO("processOnboardStepPlan: Received new plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // can't ignore plans from onboard based on timestamp, since that's how we tell if it's the same plan
    //if(step_plan->header.stamp.toSec() != last_onboard_step_plan_stamp_.toSec())
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        // clear updated goals and steps maps only if the new plan is different
        if(step_plan->header.stamp.toSec() != last_onboard_step_plan_stamp_.toSec())
            updated_steps_.clear();

        // activate sending complete pose if plan was not requested by us
        if(step_plan->header.stamp.toSec() != last_validated_step_plan_stamp_.toSec())
            updated_goal_ = true;

        last_onboard_step_plan_stamp_ = step_plan->header.stamp;
    }
//    else
//    {
//        ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
//    }
}

// onboard action callbacks
void FootstepManager::processOCSStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);

    // update parameter set
    selected_footstep_parameter_set_ = step_plan_request->parameter_set_name.data;

    std_msgs::String cmd;
    cmd.data = selected_footstep_parameter_set_;
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepManager::processOCSStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    ROS_INFO("processOCSStepPlan: Received new plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan->header.stamp.toSec() != last_ocs_step_plan_stamp_.toSec())
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        last_ocs_step_plan_stamp_ = step_plan->header.stamp;
    }
    else
    {
        ROS_INFO("processOCSStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    }
}

// utilities
void FootstepManager::addNewPlanList()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // create new plan list and push it to the top of the stack
    std::vector<vigir_footstep_planning_msgs::StepPlan> plan_list;
    footstep_plans_undo_stack_.push(plan_list);
    // clear redo stack
    footstep_plans_redo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();
}

void FootstepManager::addCopyPlanList()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

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

void FootstepManager::cleanStacks()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    footstep_plans_undo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();
    footstep_plans_redo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();

    addNewPlanList();
}

// returns step and the step plan it's contained in based on a step_index
bool FootstepManager::findStep(const unsigned int& step_index, vigir_footstep_planning_msgs::Step &step, unsigned int& step_plan_index)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // look for step with step index
    for(int i = 0; i < getStepPlanList().size(); i++)
    {
        for(int j = 0; j < getStepPlanList()[i].steps.size(); j++)
        {
            if(step_index == getStepPlanList()[i].steps[j].step_index)
            {
                step = getStepPlanList()[i].steps[j];
                step_plan_index = i;
                return true;
            }
        }
    }
    return false;
}

// returns step plan containing step with step_index
bool FootstepManager::findStepPlan(const unsigned int& step_index, unsigned int& step_plan_index)
{
    // just create a step so we can call findstep to retrieve step
    vigir_footstep_planning_msgs::Step step;
    return findStep(step_index, step, step_plan_index);
}

void FootstepManager::extendPlanList(const vigir_footstep_planning_msgs::StepPlan& new_step_plan)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    vigir_footstep_planning_msgs::StepPlan new_step_plan_copy = new_step_plan;
    // first we need to remove any extra steps in the existing step list based on the new step plan step index
    for(int i = 0; i < getStepPlanList().size(); i++)
    {
        for(int j = 0; j < getStepPlanList()[i].steps.size(); j++)
        {
            if(getStepPlanList()[i].steps[j].step_index > new_step_plan_copy.steps[0].step_index)
            {
                // delete [j+1,end] since we already have these in the new plan
                getStepPlanList()[i].steps.erase(getStepPlanList()[i].steps.begin()+j, getStepPlanList()[i].steps.end());
                //getStepPlanList()[i].cost.erase(getStepPlanList()[i].cost.begin()+j, getStepPlanList()[i].cost.end());
                new_step_plan_copy.steps.erase(new_step_plan_copy.steps.begin(), new_step_plan_copy.steps.begin()+1);
                //new_step_plan_copy.cost.erase(new_step_plan_copy.cost.begin(), new_step_plan_copy.cost.begin()+1);
                // WHEN I UPDATE STEPPLAN MARKERS IN BETWEEN STEPPLANS, I WILL HAVE TO ADD THE LAST STEP OF PLAN A TO PLAN B
                break;
            }
        }
        // if there are no steps for this plan [i], remove it and everything that comes after it since it means we have step_index in new plan
        if(!getStepPlanList()[i].steps.size())
        {
            getStepPlanList().erase(getStepPlanList().begin()+i, getStepPlanList().end());
            break;
        }
    }

    // finally, add resulting plan to the top of the stack of plans
    getStepPlanList().push_back(new_step_plan_copy);
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_footstep_manager, FootstepManager, ocs_footstep::FootstepManager, nodelet::Nodelet)
