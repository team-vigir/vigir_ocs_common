#include "footstep_manager.h"

#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/Foot.h>
#include <vigir_footstep_planning_msgs/EditStepService.h>

#include <flor_footstep_planner_msgs/flor_footstep_planner_msgs.h>

namespace ocs_footstep
{
void FootstepManager::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();

    // TODO: Get them from footstep planner
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
    footstep_list_pub_               = nh.advertise<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, false );
    footstep_update_sub_             = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/update", 1, &FootstepManager::processFootstepPoseUpdate, this );
    footstep_undo_req_sub_           = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/undo", 1, &FootstepManager::processUndoRequest, this );
    footstep_redo_req_sub_           = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/redo", 1, &FootstepManager::processRedoRequest, this );
    footstep_exec_req_sub_           = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/execute", 1, &FootstepManager::processExecuteFootstepRequest, this );
    footstep_param_set_list_pub_     = nh.advertise<flor_ocs_msgs::OCSFootstepParamSetList>( "/flor/ocs/footstep/parameter_set_list", 1, false );
    footstep_param_set_selected_sub_ = nh.subscribe<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected", 5, &FootstepManager::processFootstepParamSetSelected, this );

    // footstep request coming from the OCS
    footstep_plan_request_sub_  = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanRequest>( "/flor/ocs/footstep/plan_request", 1, &FootstepManager::processFootstepPlanRequest, this );

    // creates publishers for visualization messages
    footstep_array_pub_         = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_array", 1, false );
    footstep_body_bb_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_path_body_array", 1, false );
    footstep_path_pub_          = nh.advertise<nav_msgs::Path>( "/flor/ocs/footstep/path", 1, false );

    // use this to track current feet pose
    lower_body_state_sub_       = nh.subscribe("/flor/state/lower_body_world", 1, &FootstepManager::processLowerBodyState, this);

    // initialize all ros action clients
    //plan request
    step_plan_request_client_ = new StepPlanRequestClient("step_plan_request", true);
    //edit step
    edit_step_client_ = new EditStepClient("edit_step", true);
    //stitch step plan
    stitch_step_plan_client_ = new StitchStepPlanClient("stitch_step_plan", true);
    //update step plan
    update_step_plan_client_ = new UpdateStepPlanClient("update_step_plan", true);
    //get all parameter sets
    get_all_parameter_sets_client_ = new GetAllParameterSetsClient("get_all_parameter_sets", true);
    //execute step plan
    execute_step_plan_client_ = new ExecuteStepPlanClient("execute_step_plan", true);

    //wait for servers to come online
    step_plan_request_client_->waitForServer();
    edit_step_client_->waitForServer();
    stitch_step_plan_client_->waitForServer();
    update_step_plan_client_->waitForServer();
    get_all_parameter_sets_client_->waitForServer();
    //execute_step_plan_client_->waitForServer();

    // initialize service for deleting footsteps
    edit_step_service_client_ = nh.serviceClient<vigir_footstep_planning_msgs::EditStepService>("/vigir/global_footstep_planner/edit_step");

    // initialize step plan list
    addNewPlanList();

    // request parameter sets
    sendGetAllParameterSetsGoal();
    selected_footstep_parameter_set_ = "";

    timer = nh.createTimer(ros::Duration(0.066), &FootstepManager::timerCallback, this);
}

void FootstepManager::timerCallback(const ros::TimerEvent& event)
{
    this->publishFootstepList();
    // REMOVE:create it's own timer with 1s sleep
    this->publishFootstepParameterSetList();
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

    // send goal
    sendEditStepGoal(getStepPlanList()[step_plan_index], step);
}

void FootstepManager::processUndoRequest(const std_msgs::Bool::ConstPtr& msg)
{

    if(footstep_plans_undo_stack_.size() > 0)
    {
        // add top to the redo stack
        footstep_plans_redo_stack_.push(getStepPlanList());
        // remove from undo stack
        footstep_plans_undo_stack_.pop();

        // publish everything again
        publishFootsteps();
    }
}

void FootstepManager::processRedoRequest(const std_msgs::Bool::ConstPtr& msg)
{
    if(footstep_plans_redo_stack_.size() > 0)
    {
        // add top to the undo stack
        footstep_plans_undo_stack_.push(footstep_plans_redo_stack_.top());
        // remove from redo stack
        footstep_plans_redo_stack_.pop();

        // publish everything again
        publishFootsteps();
    }
}

// maybe create an action for this so that I can forward feedback from controller
void FootstepManager::processExecuteFootstepRequest(const std_msgs::Bool::ConstPtr& msg)
{
    sendExecuteStepPlanGoal();
}

void FootstepManager::processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg)
{
    selected_footstep_parameter_set_ = msg->data;
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

void FootstepManager::stepPlanToFootMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    foot_array_msg.markers.clear();

    for(int i = 0; i < input.size(); i++)
    {
        for(int j = 0; j < input[i].steps.size(); j++)
        {
            visualization_msgs::Marker marker;
            stepToMarker(input[i].steps[j], marker);

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
            marker.text = boost::lexical_cast<std::string>(input[i].steps[j].step_index);
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
    visualization_msgs::Marker marker;
    marker.header = input[0].steps[0].foot.header;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    body_array_msg.markers.clear();
    for(int i = 0; i < input.size(); i++)
    {
        for(int j = 1; j < input[i].steps.size(); j++)
        {
            // approximate upper body dimensions
            float x = input[i].steps[j].foot.pose.position.x + 0.5 * (input[i].steps[j-1].foot.pose.position.x - input[i].steps[j].foot.pose.position.x);
            float y = input[i].steps[j].foot.pose.position.y + 0.5 * (input[i].steps[j-1].foot.pose.position.y - input[i].steps[j].foot.pose.position.y);
            float z = input[i].steps[j].foot.pose.position.z + 0.5 * (input[i].steps[j-1].foot.pose.position.z - input[i].steps[j].foot.pose.position.z);
            float yaw1 = tf::getYaw(input[i].steps[j-1].foot.pose.orientation);
            float yaw2 = tf::getYaw(input[i].steps[j].foot.pose.orientation);
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
}

void FootstepManager::stepPlanToFootPath(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, nav_msgs::Path& foot_path_msg)
{
    foot_path_msg.header = input[0].header;
    foot_path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < input.size(); i++)
    {
        for (size_t j = 0; j < input[i].steps.size(); j++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = input[i].steps[j].foot.header;
            pose.header.stamp = ros::Time::now();
            pose.pose.position = input[i].steps[j].foot.pose.position;

            foot_path_msg.poses.push_back(pose);
        }
    }
}

void FootstepManager::processLowerBodyState(const flor_state_msgs::LowerBodyState::ConstPtr& lower_body_state)
{
    lower_body_state_ = *lower_body_state;
}

void FootstepManager::processFootstepPlanRequest(const flor_ocs_msgs::OCSFootstepPlanRequest::ConstPtr& plan_request)
{
    goal_pose_ = plan_request->goal_pose;

    if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN)
    {
        // request a completely new plan starting from the robot position
        requestStepPlanFromRobot();
    }
    else if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::CONTINUE_CURRENT_PLAN)
    {
        // get last step
        vigir_footstep_planning_msgs::Step step = getStepPlan().steps.back();
        // request a footstep plan starting from the last step
        requestStepPlanFromStep(step);
    }
    else if(plan_request->mode == flor_ocs_msgs::OCSFootstepPlanRequest::CONTINUE_FROM_STEP)
    {
        vigir_footstep_planning_msgs::StepPlan step_plan;
        vigir_footstep_planning_msgs::Step step;
        // unlikely, but if start index is the very first step
        if(plan_request->start_index == 0)
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
            if(!findStep(plan_request->start_index-1, step, step_plan_index))
                return;
        }

        requestStepPlanFromStep(step);
    }
}

void FootstepManager::requestStepPlanFromRobot()
{
    // first we calculate start feet poses
    vigir_footstep_planning_msgs::Feet start;

    //start left
    start.left.foot_index = vigir_footstep_planning_msgs::Foot::LEFT;
    start.left.pose = lower_body_state_.left_foot_pose;
    //start right
    start.right.foot_index = vigir_footstep_planning_msgs::Foot::RIGHT;
    start.right.pose = lower_body_state_.right_foot_pose;

    // and then the end feet poses
    vigir_footstep_planning_msgs::Feet goal;

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

    sendStepPlanRequestGoal(start, goal);
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
    start.left  = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? step.foot : next_step.foot);
    start.right = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ? step.foot : next_step.foot);

    // and then the end feet poses
    vigir_footstep_planning_msgs::Feet goal;

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

    unsigned char start_foot;
    if(next_step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT)
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::LEFT;
    else
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::RIGHT;

    sendStepPlanRequestGoal(start, goal, next_step.step_index, start_foot);
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

void FootstepManager::updateVisualizationMsgs()
{
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
    for(int i = 0; i < getStepPlanList().size(); i++)
    {
        for(int j = 0; j < getStepPlanList()[i].steps.size(); j++)
        {
            list.footstep_id_list.push_back(getStepPlanList()[i].steps[j].step_index);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/world";
            pose.header.stamp = ros::Time::now();
            pose.pose = getStepPlanList()[i].steps[j].foot.pose;
            list.pose.push_back(pose);
        }
    }
    footstep_list_pub_.publish(list);
}

void FootstepManager::publishFootstepParameterSetList()
{
    flor_ocs_msgs::OCSFootstepParamSetList cmd;
    cmd.param_set = footstep_parameter_set_list_;
    footstep_param_set_list_pub_.publish(cmd);
}

// action goal for StepPlanRequest
void FootstepManager::sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet& start, vigir_footstep_planning_msgs::Feet& goal, const unsigned int start_index, const unsigned char start_foot)
{
    vigir_footstep_planning_msgs::StepPlanRequest request;

    request.start = start;
    request.goal = goal;
    request.start_index = start_index;

    request.start_foot_selection = start_foot;

    // default planning mode is 2D, but will get that from the OCS
    request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

    // need to get the following from the OCS as well
    //float32 max_planning_time         # maximum planning time given in second
    //float32 max_number_steps          # maximum number of steps, set 0 for unlimited
    //float32 max_path_length_ratio     # maximum path length ratio computed as (current path length)/(beeline start<->goal), must be larger 1 otherwise it will be ignored

    request.parameter_set_name.data = selected_footstep_parameter_set_;

    // Fill in goal here
    vigir_footstep_planning_msgs::StepPlanRequestGoal action_goal;
    action_goal.plan_request = request;

    // and send it to the server
    if(step_plan_request_client_->isServerConnected())
    {
        step_plan_request_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneStepPlanRequest, this, _1, _2),
                                            boost::bind(&FootstepManager::activeStepPlanRequest, this),
                                            boost::bind(&FootstepManager::feedbackStepPlanRequest, this, _1));
    }
    else
    {
        ROS_ERROR("StepPlanRequest: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeStepPlanRequest()
{
    ROS_ERROR("StepPlanRequest: Status changed to active.");
}

void FootstepManager::feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
    ROS_ERROR("StepPlanRequest: Feedback received.");
}

void FootstepManager::doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
    ROS_ERROR("StepPlanRequest: Got action response. %s", result->status.error_msg.c_str());

    if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
    {
        if(result->step_plan.steps.size() == 0)
        {
            ROS_ERROR("StepPlanRequest: Received empty step plan.");
            return;
        }

        // we only change the current step lists if we receive a response
        if(result->step_plan.steps[0].step_index == 0)
            // This function will create a completely new plan, so we need to add a new empty list of plans to the stack
            addNewPlanList();
        else
            // This function will add a copy of the current step plan list to the stack, so we can change it
            addCopyPlanList();

        // add resulting plan to the top of the stack of plans, removing any extra steps
        extendPlanList(result->step_plan);

        publishFootsteps();

        // try to stitch all plans in the list together
        //sendStitchStepPlanGoal(getStepPlanList());
    }
}

// action goal for EditStep
void FootstepManager::sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan& step_plan, vigir_footstep_planning_msgs::Step& step, unsigned int plan_mode)
{
    // Fill in goal here
    vigir_footstep_planning_msgs::EditStepGoal action_goal;
    action_goal.step_plan = step_plan;
    action_goal.edit_step.plan_mode = plan_mode;
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
        ROS_ERROR("EditStep: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeEditStep()
{
    ROS_ERROR("EditStep: Status changed to active.");
}

void FootstepManager::feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback)
{
    ROS_ERROR("EditStep: Feedback received.");
}

void FootstepManager::doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
    ROS_ERROR("EditStep: Got action response. %s", result->status.error_msg.c_str());

    if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
    {
        // first need to figure out which step plan contains the step index used in the result
        unsigned int step_plan_index;
        findStepPlan(result->step_plans[0].steps[0].step_index, step_plan_index);

        // save the index of the step plan
        std::vector<vigir_footstep_planning_msgs::StepPlan>::iterator step_plan_it = getStepPlanList().begin()+step_plan_index;
        // remove the plan
        getStepPlanList().erase(step_plan_it);

        // and add resulting plan(s) to the list again using the previous index
        getStepPlanList().insert(getStepPlanList().begin()+step_plan_index, result->step_plans.begin(), result->step_plans.end());

        publishFootsteps();
    }
}

// action goal for stitchstepplan
void FootstepManager::sendStitchStepPlanGoal(std::vector<vigir_footstep_planning_msgs::StepPlan>& step_plan_list)
{
    if(step_plan_list.size() < 2)
        return;

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
        ROS_ERROR("StitchStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeStitchStepPlan()
{
    ROS_ERROR("StitchStepPlan: Status changed to active.");
}

void FootstepManager::feedbackStitchStepPlan(const vigir_footstep_planning_msgs::StitchStepPlanFeedbackConstPtr& feedback)
{
    ROS_ERROR("StitchStepPlan: Feedback received.");
}

void FootstepManager::doneStitchStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StitchStepPlanResultConstPtr& result)
{
    ROS_ERROR("StitchStepPlan: Got action response. %s", result->status.error_msg.c_str());

    if(result->status.error == vigir_footstep_planning_msgs::ErrorStatus::NO_ERROR)
    {
        // create a new plan list for our stitched step plan
        addNewPlanList();

        // add new step plan to the list
        getStepPlanList().push_back(result->step_plan);

        publishFootsteps();
    }
}

// action goal for updatestepplan
void FootstepManager::sendUpdateStepPlanGoal(vigir_footstep_planning_msgs::StepPlan& step_plan)
{
    // Fill in goal here
    vigir_footstep_planning_msgs::UpdateStepPlanGoal action_goal;
    action_goal.step_plan = step_plan;
    action_goal.parameter_set_name.data = selected_footstep_parameter_set_;
    action_goal.mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_REPLAN;

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
        ROS_ERROR("StitchStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeUpdateStepPlan()
{
    ROS_ERROR("UpdateStepPlan: Status changed to active.");
}

void FootstepManager::feedbackUpdateStepPlan(const vigir_footstep_planning_msgs::UpdateStepPlanFeedbackConstPtr& feedback)
{
    ROS_ERROR("UpdateStepPlan: Feedback received.");
}

void FootstepManager::doneUpdateStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result)
{
    ROS_ERROR("UpdateStepPlan: Got action response. %s", result->status.error_msg.c_str());
}

// action goal for executestep
void FootstepManager::sendExecuteStepPlanGoal()
{
    // need to make sure we only have one step plan, and that plan has steps
    if(getStepPlanList().size() != 1 || !getStepPlan().steps.size())
        return;

    // Fill in goal here
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal;
    action_goal.step_plan = getStepPlan();
    // and send it to the server
    if(execute_step_plan_client_->isServerConnected())
    {
        execute_step_plan_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneExecuteStepPlan, this, _1, _2),
                                            boost::bind(&FootstepManager::activeExecuteStepPlan, this),
                                            boost::bind(&FootstepManager::feedbackExecuteStepPlan, this, _1));
    }
    else
    {
        ROS_ERROR("ExecuteStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeExecuteStepPlan()
{
    ROS_ERROR("ExecuteStepPlan: Status changed to active.");
}

void FootstepManager::feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
    ROS_ERROR("ExecuteStepPlan: Feedback received.");
}

void FootstepManager::doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
    ROS_ERROR("ExecuteStepPlan: Got action response. %d", result->status.status);

    if(result->status.status == vigir_footstep_planning_msgs::FootstepExecutionStatus::NO_ERROR)
    {
        cleanStacks();

        publishFootsteps();
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
        ROS_ERROR("GetAllParameterSets: Server not connected!");
    }
}

void FootstepManager::activeGetAllParameterSets()
{
    ROS_ERROR("GetAllParameterSets: Status changed to active.");
}

void FootstepManager::feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback)
{
    ROS_ERROR("GetAllParameterSets: Feedback received.");
}

void FootstepManager::doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result)
{
    ROS_ERROR("GetAllParameterSets: Got action response. %s", result->status.error_msg.c_str());

    if(result->status.error == vigir_footstep_planning_msgs::FootstepExecutionStatus::NO_ERROR)
    {
        footstep_parameter_set_list_.clear();
        for(int i = 0; i < result->param_sets.size(); i++)
            footstep_parameter_set_list_.push_back(result->param_sets[i].name.data);

        if(selected_footstep_parameter_set_ == "" && footstep_parameter_set_list_.size() > 0)
            selected_footstep_parameter_set_ = footstep_parameter_set_list_[0];

        this->publishFootstepParameterSetList();
    }
}

// utilities
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

void FootstepManager::cleanStacks()
{
    footstep_plans_undo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();
    footstep_plans_redo_stack_ = std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> >();

    addNewPlanList();
}

// returns step and the step plan it's contained in based on a step_index
bool FootstepManager::findStep(const unsigned int& step_index, vigir_footstep_planning_msgs::Step &step, unsigned int& step_plan_index)
{
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
    // first we need to remove any extra steps in the existing step list based on the new step plan step index
    for(int i = 0; i < getStepPlanList().size(); i++)
    {
        for(int j = 0; j < getStepPlanList()[i].steps.size(); j++)
        {
            if(new_step_plan.steps[0].step_index <= getStepPlanList()[i].steps[j].step_index)
            {
                // delete [j,end] since we already have these in the new plan
                getStepPlanList()[i].steps.erase(getStepPlanList()[i].steps.begin()+j, getStepPlanList()[i].steps.end());
                getStepPlanList()[i].cost.erase(getStepPlanList()[i].cost.begin()+j, getStepPlanList()[i].cost.end());
                // use service because the contents of the stepplan message may change, and we need synchronous call for this
                /*for(int d = getStepPlanList()[i].steps.size()-1; d >= j; d--)
                {
                    vigir_footstep_planning_msgs::EditStepService srv;
                    srv.request.step_plan = getStepPlanList()[i];
                    srv.request.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_REMOVE;
                    srv.request.edit_step.step = getStepPlanList()[i].steps[j];
                    if(edit_step_service_client_.call(srv))
                    {
                        getStepPlanList()[i] = srv.response.step_plans[0];
                    }
                    else
                    {
                        ROS_ERROR("Failed to call EditStep service.");
                    }
                }*/
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
    getStepPlanList().push_back(new_step_plan);
}

}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_footstep_manager, FootstepManager, ocs_footstep::FootstepManager, nodelet::Nodelet)
