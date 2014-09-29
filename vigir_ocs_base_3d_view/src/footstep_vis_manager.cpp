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
    goal_pose_walk_->subProp( "Topic" )->setValue( "/goal_pose_step" );
    goal_pose_walk_->subProp( "Shape" )->setValue( "Axes" );

    planner_start_ = manager_->createDisplay( "rviz/Pose", "Start pose", true );
    planner_start_->subProp( "Topic" )->setValue( "/ros_footstep_planner/start" );
    planner_start_->subProp( "Shape" )->setValue( "Axes" );

    planned_path_ = manager_->createDisplay( "rviz/Path", "Planned path", true );
    planned_path_->subProp( "Topic" )->setValue( "/flor/ocs/footstep/path" );

    // creates publishers and subscribers for the interaction loop
    footstep_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSFootstepUpdate>( "/flor/ocs/footstep/update", 1, false );
    footstep_list_sub_   = nh_.subscribe<flor_ocs_msgs::OCSFootstepList>( "/flor/ocs/footstep/list", 1, &FootstepVisManager::processFootstepList, this );

    // publishers and subscribers for the interactive markers
    interactive_marker_add_pub_      = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 5, false );
    interactive_marker_update_pub_   = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 100, false );
    interactive_marker_feedback_sub_ = nh_.subscribe( "/flor/ocs/interactive_marker_server/feedback", 5, &FootstepVisManager::onMarkerFeedback, this );
    interactive_marker_remove_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 5, false );
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

void FootstepVisManager::processFootstepList(const flor_ocs_msgs::OCSFootstepList::ConstPtr& msg)
{
    footstep_list_ = *msg;

    updateInteractiveMarkers();
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
}
