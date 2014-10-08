/*
 * FootstepVisManager class definition.
 *
 * Author: Felipe Bacim.
 */

#ifndef FOOTSTEP_VIS_MANAGER_H
#define FOOTSTEP_VIS_MANAGER_H

#include <QObject>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <flor_ocs_msgs/OCSFootstepList.h>
#include <flor_ocs_msgs/OCSFootstepUpdate.h>
#include <flor_ocs_msgs/OCSFootstepPlanRequest.h>
#include <flor_ocs_msgs/OCSFootstepParamSetList.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>

namespace rviz
{
class Display;
class VisualizationManager;
class RenderPanel;
}

namespace vigir_ocs
{

// Class "FootstepVisManager" implements every function related to visualization of and interaction with footsteps.
class FootstepVisManager: public QObject
{
    Q_OBJECT
public:

    /**
      * @param manager Pointer to the baseview visualization manager for this rviz instance
      * Class "FootstepVisManager" implements every function related to visualization of and interaction with footsteps.
      */
    FootstepVisManager( rviz::VisualizationManager* manager );
    virtual ~FootstepVisManager();

    /**
      * Create a footstep plan request with the given goal
      */
    void processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose);

    /**
      * Receives list of footsteps and creates/removes interactive markers
      */
    void processFootstepList(const flor_ocs_msgs::OCSFootstepList::ConstPtr& msg);

    void processFootstepParamSetList(const flor_ocs_msgs::OCSFootstepParamSetList::ConstPtr& msg);

    /**
      * ROS Callback: receives interactive marker pose updates
      */
    void onMarkerFeedback( const flor_ocs_msgs::OCSInteractiveMarkerUpdate& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

    /**
      * Select footstep using the context menu
      */
    void selectContextMenu();

    /**
      * Sends an undo request to the footstep manager, a.k.a. restore previous state
      */
    void requestFootstepListUndo();

    /**
      * Sends a redo request to the footstep manager
      */
    void requestFootstepListRedo();

    /**
      * Sends a footstep plan execute request to the footstep manager
      */
    void requestExecuteStepPlan();

    /**
      * Sets the request mode for new footstep plans
      */
    void setRequestMode(unsigned char mode = flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN, int start_index = -1);

public Q_SLOTS:
    /**
      * Set visibility of all footstep-related displays
      */
    void setEnabled(bool enabled);

    /**
      * Set visibility of individual footstep interactive marker
      */
    void enableMarker(int footstep_id, bool enabled);

    /**
      * Set visibility of all footstep interactive markers
      */
    void enableMarkers(bool enabled);

    void setFootstepParameterSet(QString selected);
    /**
      * Update all Footstep parameters from ui
      */
    void updateFootstepParamaters(double,int,double,int,int,bool);

Q_SIGNALS:

    /**
      * Set visibility of all footstep interactive markers
      */
    void populateFootstepParameterSetBox(std::vector<std::string>);

private:
    void updateInteractiveMarkers();

    ros::NodeHandle nh_;

    ros::Publisher footstep_update_pub_;
    ros::Subscriber footstep_list_sub_;
    ros::Publisher footstep_undo_req_pub_;
    ros::Publisher footstep_redo_req_pub_;
    ros::Publisher footstep_exec_req_pub_;

    ros::Subscriber footstep_goal_sub_;
    ros::Publisher footstep_plan_request_pub_;
    ros::Subscriber footstep_param_set_list_sub_;
    ros::Publisher footstep_param_set_selected_pub_;

    ros::Publisher interactive_marker_add_pub_;
    ros::Publisher interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;
    ros::Publisher interactive_marker_remove_pub_;

    flor_ocs_msgs::OCSFootstepList footstep_list_;

    rviz::VisualizationManager* manager_;

    // marker for individual footsteps
    std::vector<rviz::Display*> display_footstep_marker_list_;
    // marker for step plans
    std::vector<rviz::Display*> display_step_plan_marker_list_;

    // new displays for walking
    rviz::Display* footsteps_array_;
    rviz::Display* goal_pose_walk_;
    rviz::Display* goal_pose_step_;
    rviz::Display* planner_start_;
    rviz::Display* planned_path_;
    rviz::Display* footsteps_path_body_array_;

    unsigned char request_mode_;
    int start_step_index_;

    // footstep parameters set from ui
    float max_time_;
    int max_steps_;
    float path_length_ratio_;
    int interaction_mode_;
    int planning_paramater_;
    bool pattern_generation_enabled_;

};

}
#endif // FOOTSTEP_VIS_MANAGER_H
