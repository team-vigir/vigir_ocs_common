#include <boost/algorithm/string/predicate.hpp>

#include <render_panel_custom.h>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"

#include "footstep_vis_manager.h"

#include <QMessageBox>
#include <QApplication>

namespace vigir_ocs
{
FootstepVisManager::FootstepVisManager(rviz::VisualizationManager *manager) :
    QObject(NULL),
    manager_(manager)
{
    // creates all the rviz displays
    //for step plan goal
    goal_steps_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps array", true );
    goal_steps_array_->subProp( "Marker Topic" )->setValue( "/flor/ocs/footstep/plan_goal_array" );

    //and results
    footsteps_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps array", true );
    footsteps_array_->subProp( "Marker Topic" )->setValue( "/flor/ocs/footstep/footsteps_array" );

    footsteps_path_body_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps Path Body", true );
    footsteps_path_body_array_->subProp( "Marker Topic" )->setValue( "/flor/ocs/footstep/footsteps_path_body_array" );

    goal_pose_ = manager_->createDisplay( "rviz/Pose", "Goal pose", false );
    goal_pose_->subProp( "Topic" )->setValue( ("/flor/ocs/footstep/"+ros::this_node::getName()+"/goal_pose").c_str() );
    goal_pose_->subProp( "Shape" )->setValue( "Axes" );

    planner_start_ = manager_->createDisplay( "rviz/Pose", "Start pose", false );
    planner_start_->subProp( "Topic" )->setValue( "/ros_footstep_planner/start" );
    planner_start_->subProp( "Shape" )->setValue( "Axes" );

    planned_path_ = manager_->createDisplay( "rviz/Path", "Planned path", true );
    planned_path_->subProp( "Topic" )->setValue( "/flor/ocs/footstep/path" );

    // footstep planner feedback topics
    planner_terrain_classifier_cloud_processed_ = manager_->createDisplay( "rviz/PointCloud2", "Terrain classifier (OCS) cloud processed", true );
    planner_terrain_classifier_cloud_processed_->subProp( "Topic" )->setValue( "/vigir/ocs/terrain_classifier/cloud_processed" );

    planner_plan_request_feedback_cloud_ = manager_->createDisplay( "rviz/PointCloud2", "Terrain classifier (OCS) cloud processed", true );
    planner_plan_request_feedback_cloud_->subProp( "Topic" )->setValue( "/flor/ocs/footstep/plan_request_feedback" );

    // creates publishers and subscribers for the interaction loop
    footstep_update_pub_             = nh_.advertise<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/step_update", 1, false );
    footstep_list_sub_               = nh_.subscribe<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 5, &FootstepVisManager::processFootstepList, this );
    footstep_undo_req_pub_           = nh_.advertise<std_msgs::Int8>( "/flor/ocs/footstep/undo", 1, false );
    footstep_has_undo_sub_           = nh_.subscribe<std_msgs::UInt8>( "/flor/ocs/footstep/undos_available", 5, &FootstepVisManager::processUndosAvailable, this );
    footstep_redo_req_pub_           = nh_.advertise<std_msgs::Int8>( "/flor/ocs/footstep/redo", 1, false );
    footstep_has_redo_sub_           = nh_.subscribe<std_msgs::UInt8>( "/flor/ocs/footstep/redos_available", 5, &FootstepVisManager::processRedosAvailable, this );
    footstep_start_index_pub_        = nh_.advertise<std_msgs::Int32>( "/flor/ocs/footstep/set_start_index", 1, false );
    footstep_execute_req_pub_        = nh_.advertise<std_msgs::Int8>( "/flor/ocs/footstep/execute", 1, false );
    footstep_stitch_req_pub_         = nh_.advertise<std_msgs::Int8>( "/flor/ocs/footstep/stitch", 1, false );
    footstep_plan_parameters_pub_    = nh_.advertise<flor_ocs_msgs::OCSFootstepPlanParameters>( "/flor/ocs/footstep/plan_parameters", 1, false );
    footstep_plan_parameters_sub_    = nh_.subscribe<flor_ocs_msgs::OCSFootstepPlanParameters>( "/flor/ocs/footstep/plan_parameters_feedback", 5, &FootstepVisManager::processFootstepPlanParameters, this );
    footstep_param_set_list_sub_     = nh_.subscribe<flor_ocs_msgs::OCSFootstepParamSetList>( "/flor/ocs/footstep/parameter_set_list", 5, &FootstepVisManager::processFootstepParamSetList, this );
    footstep_param_set_selected_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected", 1, false );
    footstep_param_set_selected_sub_ = nh_.subscribe<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected_feedback", 5, &FootstepVisManager::processFootstepParamSet, this );

    // publishers and subscribers for the plan request
    footstep_goal_sub_               = nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ocs/footstep/"+ros::this_node::getName()+"/goal_pose", 5, &FootstepVisManager::processGoalPose, this );
    footstep_plan_goal_pub_          = nh_.advertise<flor_ocs_msgs::OCSFootstepPlanGoal>( "/flor/ocs/footstep/plan_goal", 1, false );
    footstep_goal_pose_fb_pub_       = nh_.advertise<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_update", 1, false );
    footstep_goal_pose_fb_sub_       = nh_.subscribe<flor_ocs_msgs::OCSFootstepPlanGoalUpdate>( "/flor/ocs/footstep/goal_pose_update_feedback", 5, &FootstepVisManager::processGoalPoseFeedback, this );
    footstep_plan_request_pub_       = nh_.advertise<std_msgs::Int8>( "/flor/ocs/footstep/plan_request", 1, false );
    footstep_plan_update_pub_        = nh_.advertise<flor_ocs_msgs::OCSFootstepPlanUpdate>( "/flor/ocs/footstep/plan_update", 1, false );

    // publishers and subscribers for the interactive markers
    interactive_marker_add_pub_      = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 5, false );
    interactive_marker_update_pub_   = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 100, false );
    interactive_marker_feedback_sub_ = nh_.subscribe( "/flor/ocs/interactive_marker_server/feedback", 100, &FootstepVisManager::onMarkerFeedback, this );
    interactive_marker_remove_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 5, false );

    //initialize to default values in case requesting a plan before updating any values
    // NOTE: tried to emit signal from footstep_config on init, but was unable to be received as something else was not initialized in time
    max_time_ = 0;
    max_steps_ = 0;
    path_length_ratio_ = 0;
    edit_mode_ = 0;
    use_3d_planning_ = false;

    start_step_index_ = -1;
    need_plan_update_ = false;
    has_valid_step_plan_ = false;
    has_undo_ = 0;
    has_redo_ = 0;
    num_step_plans_ = 0;

    // initialize displays for goals
    display_goal_marker_ = NULL;
    display_goal_footstep_marker_[0] = NULL;
    display_goal_footstep_marker_[1] = NULL;
    has_goal_ = false;

    //
    button_down_ = false;

    // make sure we send step plan request if needed based on time
    timer_.start(66, this);
}

FootstepVisManager::~FootstepVisManager()
{
}

void FootstepVisManager::timerEvent(QTimerEvent *event)
{
    //ROS_INFO("Need plan update? %s", need_plan_update_ ? "yes" : "no");
    if(need_plan_update_ && !button_down_ && (boost::posix_time::second_clock::local_time()-double_click_timer_).total_milliseconds() > 100)
        requestStepPlan();
}


void FootstepVisManager::setStartingFootstep(int footstep_id)
{
    //set to plan from the footstep obtained via context
    start_step_index_ = footstep_id;

    std_msgs::Int32 cmd;
    cmd.data = start_step_index_;
    footstep_start_index_pub_.publish(cmd);
}

void FootstepVisManager::clearStartingFootstep()
{
    //set to plan from the footstep obtained via context
    start_step_index_ = -1;

    std_msgs::Int32 cmd;
    cmd.data = start_step_index_;
    footstep_start_index_pub_.publish(cmd);
}

void FootstepVisManager::requestStitchFootstepPlans()
{
    // send request to footstep manager
    std_msgs::Int8 cmd;
    cmd.data = 1;
    footstep_stitch_req_pub_.publish(cmd);
}

void FootstepVisManager::lockFootstep(int footstep_id)
{

}

void FootstepVisManager::unlockFootstep(int footstep_id)
{

}

void FootstepVisManager::removeFootstep(int footstep_id)
{

}

void FootstepVisManager::setEnabled(bool enabled)
{
    goal_pose_->setEnabled( enabled );
    planner_start_->setEnabled( enabled );
    planned_path_->setEnabled( enabled );
    footsteps_array_->setEnabled( enabled );
}

void FootstepVisManager::enableFootstepGoalMarker(int footstep_id, bool enabled)
{
    if(footstep_id < 0 || footstep_id > 1)
        return;
    if(display_goal_footstep_marker_[footstep_id])
        display_goal_footstep_marker_[footstep_id]->setEnabled( enabled && has_goal_ );
}

void FootstepVisManager::enableFootstepGoalDisplays(bool feet_markers, bool plan_markers, bool feet_array)
{
    // clear markers
    for(int i = 0; i < 2; i++)
        if(display_goal_footstep_marker_[i])
            display_goal_footstep_marker_[i]->setEnabled( feet_markers );

    if(display_goal_marker_)
        display_goal_marker_->setEnabled( plan_markers );

    // and reset footstep array
    goal_steps_array_->setEnabled( feet_array );
}

void FootstepVisManager::enableFootstepMarker(int footstep_id, bool enabled)
{
    if(footstep_id < 0 || footstep_id >= display_footstep_marker_list_.size())
        return;
    display_footstep_marker_list_[footstep_id]->setEnabled( enabled );
}

void FootstepVisManager::enableFootstepMarkers(bool enabled)
{
    for(int i = 0; i < display_footstep_marker_list_.size(); i++)
        display_footstep_marker_list_[i]->setEnabled( enabled );
    for(int i = 0; i < 2; i++)
        if(display_goal_footstep_marker_[i])
            display_goal_footstep_marker_[i]->setEnabled( enabled && has_goal_ );
}

void FootstepVisManager::enableStepPlanMarkers(bool enabled)
{
    for(int i = 0; i < display_step_plan_marker_list_.size(); i++)
        display_step_plan_marker_list_[i]->setEnabled( enabled );
    if(display_goal_marker_)
        display_goal_marker_->setEnabled( enabled && has_goal_ );
}

void FootstepVisManager::setFootstepParameterSet(QString selected)
{
    ROS_INFO("setFootstepParameterSet");
    std_msgs::String cmd;
    cmd.data = selected.toStdString();
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepVisManager::requestFootstepListUndo()
{
    // send request to footstep manager
    std_msgs::Int8 cmd;
    cmd.data = 1;
    footstep_undo_req_pub_.publish(cmd);
}

void FootstepVisManager::requestFootstepListRedo()
{
    // send request to footstep manager
    std_msgs::Int8 cmd;
    cmd.data = 1;
    footstep_redo_req_pub_.publish(cmd);
}

void FootstepVisManager::requestExecuteStepPlan()
{
    int option = QMessageBox::information( NULL, "Step Plan Execution Confirmation",
                                          "Are you sure you want to execute the current step plan?",
                                          "Execute", "Cancel",
                                          0, 1 );
    if(option == 0)
    {
        // send request to footstep manager
        std_msgs::Int8 cmd;
        cmd.data = 1;
        footstep_execute_req_pub_.publish(cmd);
    }
}

void FootstepVisManager::requestStepPlan()
{
    need_plan_update_ = false;

    std_msgs::Int8 cmd;
    cmd.data = 1;
    footstep_plan_request_pub_.publish(cmd);

    NotificationSystem::Instance()->notifyPassive("Planning Footsteps");
}

void FootstepVisManager::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    // enable visibility of goal
    has_goal_ = true;
    enableFootstepGoalDisplays( false, true, true );

    // enable update of footstep plan
    double_click_timer_ = boost::posix_time::second_clock::local_time();
    need_plan_update_ = true;

    flor_ocs_msgs::OCSFootstepPlanGoal cmd;
    cmd.goal_pose = *pose;
    footstep_plan_goal_pub_.publish(cmd);
}

void FootstepVisManager::processGoalPoseFeedback(const flor_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr &plan_goal)
{
    // only do something if we're getting feedback
    if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::FEEDBACK)
    {
        // create/update step plan goal marker
        {
        std::string step_pose_string = "/step_plan_goal_marker";

        // if needed, we create a marker
        if(!display_goal_marker_)
        {
            // create a marker server for this footstep
            flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
            marker.name  = std::string("Step Plan Goal");
            marker.topic = step_pose_string;
            marker.frame = manager_->getFixedFrame().toStdString();
            marker.scale = 0.5;
            marker.point.x = plan_goal->goal_pose.pose.position.x;
            marker.point.y = plan_goal->goal_pose.pose.position.y;
            marker.point.z = plan_goal->goal_pose.pose.position.z;
            marker.mode = flor_ocs_msgs::OCSInteractiveMarkerAdd::WAYPOINT_3DOF;
            interactive_marker_add_pub_.publish(marker);

            display_goal_marker_ = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker - Step Plan Goal", true );
            display_goal_marker_->subProp( "Update Topic" )->setValue( (step_pose_string+"/pose_marker/update").c_str() );
            display_goal_marker_->subProp( "Show Axes" )->setValue( true );
            display_goal_marker_->subProp( "Show Visual Aids" )->setValue( true );
        }

        // update interactive marker pose
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.client_id = ros::this_node::getName();
        cmd.topic = step_pose_string;
        cmd.pose = plan_goal->goal_pose;
        interactive_marker_update_pub_.publish(cmd);
        }
        // create/update footstep goal markers - 0 left, 1 right
        for(int i = 0; i < 2; i++)
        {
            std::string pose_string = i ? "/footstep_goal_right_marker" : "/footstep_goal_left_marker";

            // if needed, we create a marker
            if(!display_goal_footstep_marker_[i])
            {
                // create a marker server for this footstep
                flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
                marker.name  = std::string("Footstep Goal ")+(i ? "Right" : "Left");
                marker.topic = pose_string;
                marker.frame = manager_->getFixedFrame().toStdString();
                marker.scale = 0.2;
                marker.point.x = i ? plan_goal->right_foot.pose.position.x : plan_goal->left_foot.pose.position.x;
                marker.point.y = i ? plan_goal->right_foot.pose.position.y : plan_goal->left_foot.pose.position.y;
                marker.point.z = i ? plan_goal->right_foot.pose.position.z : plan_goal->left_foot.pose.position.z;
                marker.mode = flor_ocs_msgs::OCSInteractiveMarkerAdd::OBJECT_6DOF;
                interactive_marker_add_pub_.publish(marker);

                rviz::Display* im = manager_->createDisplay( "rviz/InteractiveMarkers", (std::string("Interactive marker - Footstep  Goal ")+(i ? "Right" : "Left")).c_str(), false );
                im->subProp( "Update Topic" )->setValue( (pose_string+"/pose_marker/update").c_str() );
                im->subProp( "Show Axes" )->setValue( true );
                im->subProp( "Show Visual Aids" )->setValue( true );
                display_goal_footstep_marker_[i] = im;
            }

            // update interactive marker pose
            flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
            cmd.client_id = ros::this_node::getName();
            cmd.topic = pose_string;
            cmd.pose.pose = i ? plan_goal->right_foot.pose : plan_goal->left_foot.pose;
            interactive_marker_update_pub_.publish(cmd);
        }
    }
    else if(plan_goal->mode == flor_ocs_msgs::OCSFootstepPlanGoalUpdate::CLEAR)
    {
        has_goal_ = false;

        enableFootstepGoalDisplays( false, false, false );
    }
}

void FootstepVisManager::processFootstepList(const flor_ocs_msgs::OCSFootstepList::ConstPtr& msg)
{
    footstep_list_ = *msg;

    updateInteractiveMarkers();
}

void FootstepVisManager::processFootstepParamSetList(const flor_ocs_msgs::OCSFootstepParamSetList::ConstPtr& msg)
{
    Q_EMIT populateFootstepParameterSetBox(msg->param_set);
}

void FootstepVisManager::processFootstepParamSet(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("processFootstepParamSet");
    Q_EMIT setFootstepParameterSetBox(msg->data);
}

void FootstepVisManager::processUndosAvailable(const std_msgs::UInt8::ConstPtr& msg)
{
    has_undo_ = msg->data;
}

void FootstepVisManager::processRedosAvailable(const std_msgs::UInt8::ConstPtr& msg)
{
    has_redo_ = msg->data;
}

void FootstepVisManager::updateInteractiveMarkers()
{
    // first we create/update the step plan markers, since these are the ones we'll be seing/interacting with first
    num_step_plans_ = 0;
    for(int i = 0; i < footstep_list_.footstep_id_list.size(); i++)
    {
        // also check step plan ID so that we always have markers for the end points of step plans
        //if(i == footstep_list_.footstep_id_list.size()-1 || footstep_list_.step_plan_id_list[i] != footstep_list_.step_plan_id_list[i+1]) // to create a marker for the last steps as well
        if(footstep_list_.step_plan_id_list[i] != footstep_list_.step_plan_id_list[i+1] && i+1 < footstep_list_.footstep_id_list.size()) // to use the specialized goal marker at the end
        {
            // only do something if it's a new step plan
            std::string step_pose_string = "/step_plan_"+boost::lexical_cast<std::string>(num_step_plans_++)+"_marker";

            // if needed, we create a marker
            if(footstep_list_.step_plan_id_list[i] >= display_step_plan_marker_list_.size())
            {
                // create a marker server for this footstep
                flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
                marker.name  = std::string("Step Plan ")+boost::lexical_cast<std::string>(i);
                marker.topic = step_pose_string;
                marker.frame = manager_->getFixedFrame().toStdString();
                marker.scale = 0.5;
                marker.point.x = (footstep_list_.pose[i].pose.position.x+footstep_list_.pose[i-1].pose.position.x)/2.0;
                marker.point.y = (footstep_list_.pose[i].pose.position.y+footstep_list_.pose[i-1].pose.position.y)/2.0;
                marker.point.z = (footstep_list_.pose[i].pose.position.z+footstep_list_.pose[i-1].pose.position.z)/2.0;
                marker.mode = flor_ocs_msgs::OCSInteractiveMarkerAdd::WAYPOINT_3DOF;
                interactive_marker_add_pub_.publish(marker);

                rviz::Display* im = manager_->createDisplay( "rviz/InteractiveMarkers", (std::string("Interactive marker - Step Plan ")+boost::lexical_cast<std::string>(i)).c_str(), true );
                im->subProp( "Update Topic" )->setValue( (step_pose_string+"/pose_marker/update").c_str() );
                im->subProp( "Show Axes" )->setValue( true );
                im->subProp( "Show Visual Aids" )->setValue( true );
                display_step_plan_marker_list_.push_back(im);
            }

            display_step_plan_marker_list_[num_step_plans_-1]->setEnabled( true );

            // update interactive marker pose
            flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
            cmd.client_id = ros::this_node::getName();
            cmd.topic = step_pose_string;
            cmd.pose.pose.position.x = (footstep_list_.pose[i].pose.position.x+footstep_list_.pose[i-1].pose.position.x)/2.0;
            cmd.pose.pose.position.y = (footstep_list_.pose[i].pose.position.y+footstep_list_.pose[i-1].pose.position.y)/2.0;
            cmd.pose.pose.position.z = (footstep_list_.pose[i].pose.position.z+footstep_list_.pose[i-1].pose.position.z)/2.0;
            Ogre::Quaternion q1(footstep_list_.pose[i-1].pose.orientation.w,footstep_list_.pose[i-1].pose.orientation.x,footstep_list_.pose[i-1].pose.orientation.y,footstep_list_.pose[i-1].pose.orientation.z);
            Ogre::Quaternion q2(footstep_list_.pose[i].pose.orientation.w,footstep_list_.pose[i].pose.orientation.x,footstep_list_.pose[i].pose.orientation.y,footstep_list_.pose[i].pose.orientation.z);
            Ogre::Quaternion qr = Ogre::Quaternion::Slerp(0.5,q1,q2);
            cmd.pose.pose.orientation.w = qr.w;
            cmd.pose.pose.orientation.x = qr.x;
            cmd.pose.pose.orientation.y = qr.y;
            cmd.pose.pose.orientation.z = qr.z;
            interactive_marker_update_pub_.publish(cmd);
        }
        // even though we don't add the intermediate marker to the end, we still need to account for it
        if(i == footstep_list_.footstep_id_list.size()-1)
            num_step_plans_++;
    }

    // check if we need to disable any step plan markers
    for(int i = num_step_plans_; i < display_step_plan_marker_list_.size(); i++)
        display_step_plan_marker_list_[i]->setEnabled( false );

    // check if it is possible to execute footstep plan without specifying a plan
    has_valid_step_plan_ = (num_step_plans_ == 1 ? true : false); // this should be set based on validation as well

    // then we create/update the individual step markers
    for(int i = 0; i < footstep_list_.footstep_id_list.size(); i++)
    {
        std::string pose_string = "/footstep_"+boost::lexical_cast<std::string>(i)+"_marker";

        // if needed, we create a marker
        if(i >= display_footstep_marker_list_.size())
        {
            // create a marker server for this footstep
            flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
            marker.name  = std::string("Footstep ")+boost::lexical_cast<std::string>(i);
            marker.topic = pose_string;
            marker.frame = manager_->getFixedFrame().toStdString();
            marker.scale = 0.2;
            marker.point.x = footstep_list_.pose[i].pose.position.x;
            marker.point.y = footstep_list_.pose[i].pose.position.y;
            marker.point.z = footstep_list_.pose[i].pose.position.z;
            marker.mode = flor_ocs_msgs::OCSInteractiveMarkerAdd::OBJECT_6DOF;
            interactive_marker_add_pub_.publish(marker);

            rviz::Display* im = manager_->createDisplay( "rviz/InteractiveMarkers", (std::string("Interactive marker - Footstep ")+boost::lexical_cast<std::string>(i)).c_str(), false );
            im->subProp( "Update Topic" )->setValue( (pose_string+"/pose_marker/update").c_str() );
            im->subProp( "Show Axes" )->setValue( true );
            im->subProp( "Show Visual Aids" )->setValue( true );
            display_footstep_marker_list_.push_back(im);
        }

        //display_footstep_marker_list_[i]->setEnabled( true );

        // update interactive marker pose
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.client_id = ros::this_node::getName();
        cmd.topic = pose_string;
        cmd.pose = footstep_list_.pose[i];
        interactive_marker_update_pub_.publish(cmd);
    }

    // make sure extra markers are disabled
    for(int i = footstep_list_.footstep_id_list.size(); i < display_footstep_marker_list_.size(); i++)
    {
        display_footstep_marker_list_[i]->setEnabled( false );
    }
}

void FootstepVisManager::onMarkerFeedback(const flor_ocs_msgs::OCSInteractiveMarkerUpdate& msg)
{
    // goal feedback
    if(boost::starts_with(msg.topic,"/step_plan_goal") || boost::starts_with(msg.topic,"/footstep_goal"))
    {
        flor_ocs_msgs::OCSFootstepPlanGoalUpdate cmd;
        if(boost::starts_with(msg.topic,"/step_plan_goal"))
        {
            cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::GOAL;
            cmd.goal_pose = msg.pose;
        }
        else if(boost::starts_with(msg.topic,"/footstep_goal_left"))
        {
            cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::LEFT;
            cmd.left_foot = msg.pose;
        }
        else if(boost::starts_with(msg.topic,"/footstep_goal_right"))
        {
            cmd.mode = flor_ocs_msgs::OCSFootstepPlanGoalUpdate::RIGHT;
            cmd.right_foot = msg.pose;
        }
        footstep_goal_pose_fb_pub_.publish(cmd);
    }
    else if(boost::starts_with(msg.topic,"/step_plan_"))
    {
        try
        {
            flor_ocs_msgs::OCSFootstepPlanUpdate cmd;
            int start_idx = strlen("/step_plan_");
            int end_idx = msg.topic.substr(start_idx, msg.topic.size()-start_idx).find("_marker");
            cmd.step_plan_id = boost::lexical_cast<int>(msg.topic.substr(start_idx,end_idx).c_str());
            cmd.pose = msg.pose;
            //convert back to ankle

            footstep_plan_update_pub_.publish(cmd);
        }
        catch( boost::bad_lexical_cast const& )
        {
            ROS_ERROR("Error: input string was not valid");
        }
    }
    else if(boost::starts_with(msg.topic,"/footstep_"))
    {
        try
        {
            flor_ocs_msgs::OCSFootstepUpdate cmd;
            int start_idx = strlen("/footstep_");
            int end_idx = msg.topic.substr(start_idx, msg.topic.size()-start_idx).find("_marker");
            cmd.footstep_id = boost::lexical_cast<int>(msg.topic.substr(start_idx,end_idx).c_str());
            cmd.pose = msg.pose;
            //convert back to ankle

            footstep_update_pub_.publish(cmd);
        }
        catch( boost::bad_lexical_cast const& )
        {
            ROS_ERROR("Error: input string was not valid");
        }
    }

    if(msg.client_id == ros::this_node::getName())
    {
        // on mouse release, update plan
        if(msg.event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
        {
            ROS_INFO("%s: button down",ros::this_node::getName().c_str());
            button_down_ = true;
        }
        else if(button_down_ && msg.event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        {
            ROS_INFO("%s: button up",ros::this_node::getName().c_str());
            double_click_timer_ = boost::posix_time::second_clock::local_time();
            need_plan_update_ = true;
            button_down_ = false;
        }
    }
}

void FootstepVisManager::updateFootstepParamaters(double max_time,int max_steps,double path_length_ratio,int edit_mode)
{
    //update all paramaters from ui
    max_time_ = max_time;
    max_steps_ = max_steps;
    path_length_ratio_ = path_length_ratio;
    edit_mode_ = edit_mode;

    sendFootstepPlanParameters();
}

void FootstepVisManager::update3dPlanning(bool use_3d_planning)
{
    //update all paramaters from ui
    use_3d_planning_ = use_3d_planning;

    sendFootstepPlanParameters();
}

void FootstepVisManager::sendFootstepPlanParameters()
{
    flor_ocs_msgs::OCSFootstepPlanParameters cmd;
    cmd.max_time = max_time_;
    cmd.max_steps = max_steps_;
    cmd.path_length_ratio = path_length_ratio_;
    cmd.edit_mode = edit_mode_;
    cmd.use_3d_planning = use_3d_planning_;
    footstep_plan_parameters_pub_.publish(cmd);
}

void FootstepVisManager::processFootstepPlanParameters(const flor_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg)
{
    Q_EMIT set3dPlanning(msg->use_3d_planning);
    Q_EMIT setFootstepParamaters(msg->max_time, msg->max_steps, msg->path_length_ratio, msg->edit_mode);
}

}
