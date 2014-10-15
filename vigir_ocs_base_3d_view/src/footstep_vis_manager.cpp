#include <boost/algorithm/string/predicate.hpp>

#include <render_panel_custom.h>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"

#include "footstep_vis_manager.h"

namespace vigir_ocs
{
FootstepVisManager::FootstepVisManager(rviz::VisualizationManager *manager) :
    QObject(NULL),
    manager_(manager)
{
    // creates all the rviz displays
    footsteps_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps array", true );
    footsteps_array_->subProp( "Marker Topic" )->setValue( "/flor/ocs/footstep/footsteps_array" );

    footsteps_path_body_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps Path Body", true );
    footsteps_path_body_array_->subProp( "Marker Topic" )->setValue( "/flor/ocs/footstep/footsteps_path_body_array" );

    goal_pose_walk_ = manager_->createDisplay( "rviz/Pose", "Goal pose", true );
    goal_pose_walk_->subProp( "Topic" )->setValue( "/flor/ocs/footstep/goal_pose" );
    goal_pose_walk_->subProp( "Shape" )->setValue( "Axes" );

    planner_start_ = manager_->createDisplay( "rviz/Pose", "Start pose", true );
    planner_start_->subProp( "Topic" )->setValue( "/ros_footstep_planner/start" );
    planner_start_->subProp( "Shape" )->setValue( "Axes" );

    planned_path_ = manager_->createDisplay( "rviz/Path", "Planned path", true );
    planned_path_->subProp( "Topic" )->setValue( "/flor/ocs/footstep/path" );

    // creates publishers and subscribers for the interaction loop
    footstep_update_pub_     = nh_.advertise<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/update", 1, false );
    footstep_list_sub_       = nh_.subscribe<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, &FootstepVisManager::processFootstepList, this );
    footstep_undo_req_pub_   = nh_.advertise<std_msgs::Bool>( "/flor/ocs/footstep/undo", 1, false );
    footstep_redo_req_pub_   = nh_.advertise<std_msgs::Bool>( "/flor/ocs/footstep/redo", 1, false );
    footstep_exec_req_pub_   = nh_.advertise<std_msgs::Bool>( "/flor/ocs/footstep/execute", 1, false );

    // publishers and subscribers for the plan request
    footstep_goal_sub_               = nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ocs/footstep/goal_pose", 5, &FootstepVisManager::processGoalPose, this );
    footstep_plan_request_pub_       = nh_.advertise<flor_ocs_msgs::OCSFootstepPlanRequest>( "/flor/ocs/footstep/plan_request", 1, false );
    footstep_param_set_list_sub_     = nh_.subscribe<flor_ocs_msgs::OCSFootstepParamSetList>( "/flor/ocs/footstep/parameter_set_list", 5, &FootstepVisManager::processFootstepParamSetList, this );
    footstep_param_set_selected_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/footstep/parameter_set_selected", 1, false );

    // publishers and subscribers for the interactive markers
    interactive_marker_add_pub_      = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 5, false );
    interactive_marker_update_pub_   = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 100, false );
    interactive_marker_feedback_sub_ = nh_.subscribe( "/flor/ocs/interactive_marker_server/feedback", 5, &FootstepVisManager::onMarkerFeedback, this );
    interactive_marker_remove_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 5, false );

    //initialize to default values in case requesting a plan before updating any values
    // NOTE: tried to emit signal from footstep_config on init, but was unable to be received as something else was not initialized in time
    max_time_ = 0;
    max_steps_ = 0;
    path_length_ratio_ = 0;
    interaction_mode_ = 0;
    pattern_generation_enabled_ = 0;
}

FootstepVisManager::~FootstepVisManager()
{
}

void FootstepVisManager::setEnabled(bool enabled)
{
    goal_pose_walk_->setEnabled( enabled );
    goal_pose_step_->setEnabled( enabled );
    planner_start_->setEnabled( enabled );
    planned_path_->setEnabled( enabled );
    footsteps_array_->setEnabled( enabled );
}

void FootstepVisManager::setRequestMode(unsigned char mode, int start_index)
{
    request_mode_ = mode;
    start_step_index_ = start_index;
}

void FootstepVisManager::enableMarker(int footstep_id, bool enabled)
{
    if(footstep_id < 0 || footstep_id >= display_footstep_marker_list_.size())
        return;
    display_footstep_marker_list_[footstep_id]->setEnabled( enabled );
}

void FootstepVisManager::enableMarkers(bool enabled)
{
    for(int i = 0; i < display_footstep_marker_list_.size(); i++)
        display_footstep_marker_list_[i]->setEnabled( enabled );
}

void FootstepVisManager::setFootstepParameterSet(QString selected)
{
    std_msgs::String cmd;
    cmd.data = selected.toStdString();
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepVisManager::requestFootstepListUndo()
{
    // send request to footstep manager
    std_msgs::Bool cmd;
    cmd.data = true;
    footstep_undo_req_pub_.publish(cmd);
}

void FootstepVisManager::requestFootstepListRedo()
{
    // send request to footstep manager
    std_msgs::Bool cmd;
    cmd.data = true;
    footstep_redo_req_pub_.publish(cmd);
}

void FootstepVisManager::requestExecuteStepPlan()
{
    // send request to footstep manager
    std_msgs::Bool cmd;
    cmd.data = true;
    footstep_exec_req_pub_.publish(cmd);
}

void FootstepVisManager::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    flor_ocs_msgs::OCSFootstepPlanRequest cmd;
    cmd.goal_pose = *pose;
    cmd.mode = request_mode_;
    cmd.start_index = start_step_index_;
    //set footstep paramaters from ui
    cmd.max_time = max_time_;
    cmd.max_steps = max_steps_;
    cmd.path_length_ratio = path_length_ratio_;
    cmd.interaction_mode = interaction_mode_;
    cmd.pattern_generation_enabled = pattern_generation_enabled_;
    ROS_ERROR("PLAN time: %f steps: %d ratio:%f intmode: %d pattern %d",max_time_,max_steps_,path_length_ratio_,interaction_mode_,pattern_generation_enabled_);

    footstep_plan_request_pub_.publish(cmd);
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

void FootstepVisManager::updateInteractiveMarkers()
{
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

        // update interactive marker pose
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = pose_string;
        cmd.pose = footstep_list_.pose[i];
        interactive_marker_update_pub_.publish(cmd);

        // also check step plan ID so that we always have markers for the end points of step plans
        if((i+1 < footstep_list_.footstep_id_list.size() && footstep_list_.step_plan_id_list[i] != footstep_list_.step_plan_id_list[i+1]) || i == footstep_list_.footstep_id_list.size()-1)
        {
            // only do something if it's a new step plan
            std::string step_pose_string = "/step_plan_"+boost::lexical_cast<std::string>(display_step_plan_marker_list_.size())+"_marker";

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

            // update interactive marker pose
            flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
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
    }

}

void FootstepVisManager::onMarkerFeedback(const flor_ocs_msgs::OCSInteractiveMarkerUpdate& msg)
{
    if(boost::starts_with(msg.topic,"/footstep_"))
    {
        try
        {
            flor_ocs_msgs::OCSFootstepUpdate cmd;
            int start_idx = msg.topic.find("/footstep_") + strlen("/footstep_");
            int end_idx = msg.topic.substr(start_idx, msg.topic.size()-start_idx).find("_marker");
            cmd.footstep_id = boost::lexical_cast<int>(msg.topic.substr(start_idx,end_idx).c_str());
            cmd.pose = msg.pose;
            footstep_update_pub_.publish(cmd);
        }
        catch( boost::bad_lexical_cast const& )
        {
            ROS_ERROR("Error: input string was not valid");
        }

    }
}

void FootstepVisManager::updateFootstepParamaters(double maxTime,int maxSteps,double pathLengthRatio,int interactionMode,bool patternGeneration)
{
    //update all paramaters from ui
    max_time_ = maxTime;
    max_steps_ = maxSteps;
    path_length_ratio_ = pathLengthRatio;
    interaction_mode_ = interactionMode;
    pattern_generation_enabled_ = patternGeneration;
    ROS_ERROR("UPDATE time: %f steps: %d ratio:%f intmode: %d pattern %d",max_time_,max_steps_,path_length_ratio_,interaction_mode_,pattern_generation_enabled_);
}


}
