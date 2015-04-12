#include "footstep_manager.h"

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/Foot.h>
#include <vigir_footstep_planning_msgs/EditStepService.h>

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

    // planner will always create new plans if this is unchanged
    start_step_index_ = -1;

    // initialize step plan list
    addNewPlanList();

    // creates publishers and subscribers for the interaction loop
    footstep_list_pub_               = nh.advertise<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, false );
    footstep_update_sub_             = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/step_update", 1, &FootstepManager::processFootstepPoseUpdate, this );
    footstep_undo_req_sub_           = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/undo", 1, &FootstepManager::processUndoRequest, this );
    footstep_redo_req_sub_           = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/redo", 1, &FootstepManager::processRedoRequest, this );
    footstep_start_index_pub_        = nh.subscribe<std_msgs::Int32>( "/flor/ocs/footstep/set_start_index", 1, &FootstepManager::processSetStartIndex, this );
    footstep_execute_req_sub_        = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/execute", 1, &FootstepManager::processExecuteFootstepRequest, this );
    footstep_stitch_req_sub_         = nh.subscribe<std_msgs::Bool>( "/flor/ocs/footstep/stitch", 1, &FootstepManager::processStitchPlansRequest, this );
    footstep_param_set_list_pub_     = nh.advertise<flor_ocs_msgs::OCSFootstepParamSetList>( "/flor/ocs/footstep/parameter_set_list", 1, false );
    footstep_param_set_selected_sub_ = nh.subscribe<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected", 5, &FootstepManager::processFootstepParamSetSelected, this );

    // footstep request coming from the OCS
    footstep_goal_pose_fb_pub_  = nh.advertise<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_feedback", 1, false );
    footstep_goal_pose_fb_sub_  = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_feedback", 1, &FootstepManager::processFootstepPlanGoalFeedback, this );
    footstep_plan_goal_sub_     = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoal>( "/flor/ocs/footstep/plan_goal", 1, &FootstepManager::processFootstepPlanGoal, this );
    footstep_plan_request_sub_  = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanRequest>( "/flor/ocs/footstep/plan_request", 1, &FootstepManager::processFootstepPlanRequest, this );
    footstep_plan_update_sub_   = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanUpdate>( "/flor/ocs/footstep/plan_update", 1, &FootstepManager::processFootstepPlanUpdate, this );

    // creates publishers for visualization messages - latched because we want to be able to see planned footsteps in new stations or if something goes wrong
    footstep_array_pub_         = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_array", 1, true );
    footstep_body_bb_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/footsteps_path_body_array", 1, true );
    footstep_path_pub_          = nh.advertise<nav_msgs::Path>( "/flor/ocs/footstep/path", 1, true );
    plan_goal_array_pub_        = nh.advertise<visualization_msgs::MarkerArray>( "/flor/ocs/footstep/plan_goal_array", 1, true );

    // use this to track current feet pose
    lower_body_state_sub_       = nh.subscribe("/flor/state/lower_body_world", 1, &FootstepManager::processLowerBodyState, this);

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
    get_all_parameter_sets_client_ = new GetAllParameterSetsClient("get_all_parameter_sets", true);
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

    // subscribe to the onboard planner in case something generates behaviors on the onboard side
    onboard_step_plan_request_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlanRequest>( "onboard_step_plan_request", 1, &FootstepManager::processOnboardStepPlanRequest, this );
    onboard_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlan>( "onboard_step_plan", 1, &FootstepManager::processOnboardStepPlan, this );

    // also subscribe to the ocs planner
    ocs_step_plan_request_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlanRequest>( "ocs_step_plan_request", 1, &FootstepManager::processOCSStepPlanRequest, this );
    ocs_step_plan_sub_ = nh.subscribe<vigir_footstep_planning_msgs::StepPlan>( "ocs_step_plan", 1, &FootstepManager::processOCSStepPlan, this );

    // point cloud visualization for plan request feedback
    planner_plan_request_feedback_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>( "/flor/ocs/footstep/plan_request_feedback", 1, true );

    ROS_INFO("FootstepManager initialized!");

    // initialize service for deleting footsteps
    //edit_step_service_client_ = nh.serviceClient<vigir_footstep_planning_msgs::EditStepService>("/vigir/global_footstep_planner/edit_step");

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
void FootstepManager::processSetStartIndex(const std_msgs::Int32::ConstPtr &msg)
{
    start_step_index_ = msg->data;

    publishFootsteps();
}

void FootstepManager::processExecuteFootstepRequest(const std_msgs::Bool::ConstPtr& msg)
{
    sendExecuteStepPlanGoal();
}

void FootstepManager::processStitchPlansRequest(const std_msgs::Bool::ConstPtr& msg)
{
    sendStitchStepPlanGoal(getStepPlanList());
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

void FootstepManager::feetToFootMarkerArray(vigir_footstep_planning_msgs::Feet& input, visualization_msgs::MarkerArray& foot_array_msg)
{
    foot_array_msg.markers.clear();

    for(int i = 0; i < 2; i++)
    {
        visualization_msgs::Marker marker;
        // create step so we can use utility function
        vigir_footstep_planning_msgs::Step step;
        step.foot = i ? input.right : input.left;
        step.header = input.left.header;
        stepToMarker(step, marker);

        marker.id = foot_array_msg.markers.size();
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.6;
        marker.color.a = 0.5;
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
            visualization_msgs::Marker marker;
            stepToMarker(input[i].steps[j], marker);

            marker.id = foot_array_msg.markers.size();
            marker.color.r = input[i].steps[j].risk/0.5 * 0.6;
            marker.color.g = start_step_index_ == input[i].steps[j].step_index ? 0.0 : fabs(0.5-input[i].steps[j].risk)/0.5 * 0.6;
            marker.color.b = start_step_index_ == input[i].steps[j].step_index ? 0.6 : 0.0;
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
            // need something better than this to compute color, but for now we only have two known modes (green, yellow)
            marker.color.r = input[i].mode == 4 ? 0.4 : 0.0; // TO DO: READ PARAMETER MAP FOR MODE -> COLOR
            marker.color.g = input[i].mode == 4 ? 0.4 : 0.0;
            marker.color.b = input[i].mode == 4 ? 0.0 : 0.4;
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

void FootstepManager::processFootstepPlanGoal(const flor_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal)
{
    // uses goal pose to request
    goal_pose_ = plan_goal->goal_pose;

    // and then the end feet poses
    calculateGoal();

    // then update feet using the footstep planner
    sendUpdateFeetGoal(goal_);
}

void FootstepManager::processFootstepPlanGoalFeedback(const flor_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal)
{
    // only one that sends feedback is the manager, so return to avoid infinite loop
    if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::FEEDBACK)
        return;

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
    }

    // need to update feet poses
    updateGoalVisMsgs();

    // send
    plan_goal_array_pub_.publish(footstep_goal_array_);

    // and update the interactive markers
    publishGoalMarkerFeedback();
}

void FootstepManager::calculateGoal()
{
    //end estimates for foot distance
    double end_yaw = tf::getYaw(goal_pose_.pose.orientation);
    double shift_x = -sin(end_yaw) * (0.5 * foot_separation);
    double shift_y =  cos(end_yaw) * (0.5 * foot_separation);

    goal_.header.frame_id = "/world";
    goal_.header.stamp = ros::Time::now();

    goal_.left.header.frame_id = "/world";
    goal_.left.header.stamp = ros::Time::now();
    goal_.left.pose.position.x = goal_pose_.pose.position.x + shift_x;
    goal_.left.pose.position.y = goal_pose_.pose.position.y + shift_y;
    goal_.left.pose.position.z = lower_body_state_.left_foot_pose.position.z;//goal_pose_.pose.position.z;
    goal_.left.pose.orientation = goal_pose_.pose.orientation;

    goal_.right.header.frame_id = "/world";
    goal_.right.header.stamp = ros::Time::now();
    goal_.right.pose.position.x = goal_pose_.pose.position.x - shift_x;
    goal_.right.pose.position.y = goal_pose_.pose.position.y - shift_y;
    goal_.right.pose.position.z = lower_body_state_.right_foot_pose.position.z;//goal_pose_.pose.position.z;
    goal_.right.pose.orientation = goal_pose_.pose.orientation;

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

void FootstepManager::processFootstepPlanRequest(const flor_ocs_msgs::OCSFootstepPlanRequest::ConstPtr& plan_request)
{
    last_plan_request_ = *plan_request;

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
    // first we calculate start feet poses
    vigir_footstep_planning_msgs::Feet start;

    start.header = lower_body_state_.header;
    //start left
    start.left.header = lower_body_state_.header;
    start.left.foot_index = vigir_footstep_planning_msgs::Foot::LEFT;
    start.left.pose = lower_body_state_.left_foot_pose;
    //start right
    start.right.header = lower_body_state_.header;
    start.right.foot_index = vigir_footstep_planning_msgs::Foot::RIGHT;
    start.right.pose = lower_body_state_.right_foot_pose;

    // since lower body state reports feet in robot frame (ankle), need to transform it to planner frame
    foot_pose_transformer_->transformToPlannerFrame(start);

    sendStepPlanRequestGoal(start, goal_);
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

    // since lower body state reports feet in robot frame (ankle), need to transform it to planner frame
    foot_pose_transformer_->transformToPlannerFrame(start);

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
    footstep_goal_pose_fb_pub_.publish(cmd);
}

void FootstepManager::publishGoalMarkerFeedback()
{
    flor_ocs_msgs::OCSFootstepPlanGoalUpdate cmd;
    cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::FEEDBACK;
    // for stepplan goal
    cmd.goal_pose = goal_pose_;
    // and for goal feetposes
    cmd.left_foot.pose = goal_.left.pose;
    cmd.right_foot.pose = goal_.right.pose;
    footstep_goal_pose_fb_pub_.publish(cmd);
}

void FootstepManager::publishFootsteps()
{
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
}

void FootstepManager::publishFootstepParameterSetList()
{
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
    if (vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("UpdateFeet: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
    }
    else
    {
        ROS_INFO("UpdateFeet: Got action response.\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // update the goal feet
        goal_ = result->feet;

        //convert to sole for visualization
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
}

// action goal for StepPlanRequest
void FootstepManager::sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index, const unsigned char start_foot)
{
    vigir_footstep_planning_msgs::StepPlanRequest request;

    //convert transform to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(start);
    foot_pose_transformer_->transformToRobotFrame(goal);

    request.header.frame_id = "/world";
    request.header.stamp = ros::Time::now();

    request.start = start;
    request.goal = goal;
    request.start_step_index = start_step_index;

    request.start_foot_selection = start_foot;

    // default planning mode is 2D, but will get that from the OCS
    request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

    // need to get the following from the OCS as well
    request.max_planning_time = last_plan_request_.max_time;
    request.max_number_steps = last_plan_request_.max_steps;
    request.max_path_length_ratio = last_plan_request_.path_length_ratio;

    // and then use the selected parameter set
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
        ROS_INFO("StepPlanRequest: Server not connected!");
    }
}

// action callbacks for step plan request
void FootstepManager::activeStepPlanRequest()
{
    ROS_INFO("StepPlanRequest: Status changed to active.");
}

void FootstepManager::feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
    ROS_INFO("StepPlanRequest: Feedback received.");

    //planner_plan_request_feedback_cloud_pub_.publish(feedback.cloud);
}

void FootstepManager::doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
    ROS_INFO("StepPlanRequest: Got action response. %s", result->status.error_msg.c_str());

    if(!vigir_footstep_planning::hasError(result->status))
    {
        boost::mutex::scoped_lock lock(step_plan_mutex_);

        if(result->step_plan.header.stamp.nsec != last_ocs_step_plan_stamp_.nsec || result->step_plan.header.stamp.sec != last_ocs_step_plan_stamp_.sec)
        {
            vigir_footstep_planning_msgs::StepPlan plan = result->step_plan;
            processNewStepPlan(plan);

            last_ocs_step_plan_stamp_ = result->step_plan.header.stamp;
        }
        else
        {
            ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", result->step_plan.header.stamp.sec, result->step_plan.header.stamp.nsec);
        }
    }
}

// action goal for EditStep
void FootstepManager::sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step, unsigned int plan_mode)
{
    //convert to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(step_plan);
    foot_pose_transformer_->transformToRobotFrame(step.foot);

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
    if (vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("EditStep: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
    }
    else
    {
        ROS_INFO("EditStep: Got action response.\n%s", vigir_footstep_planning::toString(result->status).c_str());

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


        vigir_footstep_planning_msgs::EditStepResult result_copy = *result;
        //convert all step plans transforms to be relative to sole frame for visualization
        //Brian::can optimize to just transform one plan?
        for(int i=0;i<result_copy.step_plans.size();i++)
        {
            foot_pose_transformer_->transformToPlannerFrame(result_copy.step_plans[i]);
        }

        // first need to figure out which step plan contains the step index used in the result
        unsigned int step_plan_index;
        findStepPlan(result_copy.step_plans[0].steps[0].step_index, step_plan_index);

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
    ROS_INFO("StitchStepPlan: Got action response. %s", result->status.error_msg.c_str());

    if(!vigir_footstep_planning::hasError(result->status))
    {
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
    ROS_INFO("UpdateStepPlan: Got action response. %s", result->status.error_msg.c_str());
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

    //convert transform to ankle for planner, might be redundant here?
    foot_pose_transformer_->transformToRobotFrame(action_goal.step_plan);

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
        ROS_INFO("ExecuteStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeExecuteStepPlan()
{
    ROS_INFO("ExecuteStepPlan: Status changed to active.");
}

void FootstepManager::feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("ExecuteStepPlan: Feedback received.");
}

void FootstepManager::doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
    ROS_INFO("ExecuteStepPlan: Got action response. %d", result->status.status);

    if(result->status.status == vigir_footstep_planning_msgs::FootstepExecutionStatus::NO_ERROR)
    {
        cleanStacks();
        publishFootsteps();
        publishFootstepList();
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

    if(result->status.error == vigir_footstep_planning_msgs::FootstepExecutionStatus::NO_ERROR)
    {
        footstep_parameter_set_list_.clear();
        for(int i = 0; i < result->param_sets.size(); i++)
            footstep_parameter_set_list_.push_back(result->param_sets[i]);

        if(selected_footstep_parameter_set_ == "" && footstep_parameter_set_list_.size() > 0)
            selected_footstep_parameter_set_ = footstep_parameter_set_list_[0].name.data;

        this->publishFootstepParameterSetList();
    }
}

void FootstepManager::processNewStepPlan(vigir_footstep_planning_msgs::StepPlan& step_plan)
{
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
}

// onboard action callbacks
void FootstepManager::processOnboardStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{

}

void FootstepManager::processOnboardStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    boost::mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan->header.stamp.nsec != last_onboard_step_plan_stamp_.nsec || step_plan->header.stamp.sec != last_onboard_step_plan_stamp_.sec)
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        last_onboard_step_plan_stamp_ = step_plan->header.stamp;
    }
    else
    {
        ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    }
}

// onboard action callbacks
void FootstepManager::processOCSStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{

}

void FootstepManager::processOCSStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    boost::mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan->header.stamp.nsec != last_ocs_step_plan_stamp_.nsec || step_plan->header.stamp.sec != last_ocs_step_plan_stamp_.sec)
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        last_ocs_step_plan_stamp_ = step_plan->header.stamp;
    }
    else
    {
        ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
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
