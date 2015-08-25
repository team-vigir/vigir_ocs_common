/*
 * FootstepVisManager class definition.
 *
 * Author: Felipe Bacim.
 */

#ifndef FOOTSTEP_VIS_MANAGER_H
#define FOOTSTEP_VIS_MANAGER_H

#include <QObject>
#include <QBasicTimer>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <vigir_interactive_marker_server_custom/interactive_marker_server_custom.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepList.h>
#include <vigir_ocs_msgs/OCSFootstepUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoal.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoalUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanParameters.h>
#include <vigir_ocs_msgs/OCSFootstepPlanUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepParamSetList.h>
#include <vigir_ocs_msgs/OCSFootstepSyncStatus.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <stdlib.h>
#include "notification_system.h"

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

    // Class "FootstepVisManager" implements every function related to visualization of and interaction with footsteps.
    FootstepVisManager( rviz::VisualizationManager* manager );
    virtual ~FootstepVisManager();

    // Create a footstep plan request with the given goal
    void processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose);

    // Update goal pose marker
    void processGoalPoseFeedback(const vigir_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal);

    // Receives list of footsteps and creates/removes interactive markers
    void processFootstepList(const vigir_ocs_msgs::OCSFootstepList::ConstPtr& msg);

    // Receives a list of all the parameter sets
    void processFootstepParamSetList(const vigir_ocs_msgs::OCSFootstepParamSetList::ConstPtr& msg);

    // Receives the latest selected parameter
    void processFootstepParamSet(const std_msgs::String::ConstPtr& msg);

    // Receives the latest parameters used for planning from the manager
    void processFootstepPlanParameters(const vigir_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg);

    // Receives the number of available undos and redos
    void processUndosAvailable(const std_msgs::UInt8::ConstPtr& msg);
    void processRedosAvailable(const std_msgs::UInt8::ConstPtr& msg);

    // Update sync status
    void processSyncStatus(const vigir_ocs_msgs::OCSFootstepSyncStatus::ConstPtr& msg);

    // Sends the latest parameters selected in the OCS
    void sendFootstepPlanParameters();

    // ROS Callback: receives interactive marker pose updates
    void onMarkerFeedback( const vigir_ocs_msgs::OCSInteractiveMarkerUpdate& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

    // Select footstep using the context menu
    void selectContextMenu();

    // Sends an undo request to the footstep manager, a.k.a. restore previous state
    void requestFootstepListUndo();

    // Sends a redo request to the footstep manager
    void requestFootstepListRedo();

    // Sends a step plan request, and will get the two goal footsteps
    void requestStepPlan();

    // Sends a footstep plan validate request to the footstep manager
    void requestValidateStepPlan();

    // Sends a footstep plan execute request to the footstep manager
    void requestExecuteStepPlan();

    // Sends a request to send the plan that is currently visualized in the OCS
    void requestSendOCSStepPlan();

    // Sends a footstep plan stitch request to the footstep manager
    void requestStitchFootstepPlans();

    // Set individual footstep parameters used for planning
    void setStartingFootstep(int footstep_id);
    void clearStartingFootstep();
    void lockFootstep(int footstep_id);
    void unlockFootstep(int footstep_id);
    void removeFootstep(int footstep_id);

    // verify state of plans
    bool hasGoal() { return has_goal_; }
    unsigned char hasUndoAvailable() { return has_undo_; }
    unsigned char hasRedoAvailable() { return has_redo_; }
    bool hasValidStepPlan() { return has_valid_step_plan_; }
    bool hasStartingFootstep() { return (start_step_index_ >= 0); }
    bool canValidate() { return can_validate_; }
    unsigned int numStepPlans() { return num_step_plans_; }
    unsigned char getValidateMode() { return validate_mode_; }

    virtual void timerEvent(QTimerEvent *event);

public Q_SLOTS:
    // Set visibility of all footstep-related displays
    void setEnabled(bool enabled);

    // Set visibility of individual footstep goal interactive marker
    void enableFootstepGoalMarker(int footstep_id, bool enabled);

    // Set visibility of all footstep goal markers
    void enableFootstepGoalDisplays(bool feet_markers, bool plan_markers, bool feet_array);

    // Set visibility of individual footstep interactive marker
    void enableFootstepMarker(int footstep_id, bool enabled);

    // Set visibility of all footstep interactive markers
    void enableFootstepMarkers(bool enabled);

    // Set visibility of all step plan interactive markers
    void enableStepPlanMarkers(bool enabled);

    // Set the footstep parameter set
    void setFootstepParameterSet(QString selected);

    // Update all Footstep parameters from ui
    void updateFootstepParamaters(double,int,double,int);

    // Update 3d planning parameter from ui
    void update3dPlanning(bool);

Q_SIGNALS:
    // Set visibility of all footstep interactive markers
    void populateFootstepParameterSetBox(std::vector<std::string>);
    void setFootstepParameterSetBox(std::string);
    void setFootstepParamaters(double,int,double,int);
    void set3dPlanning(bool);

private:
    void updateInteractiveMarkers();

    ros::NodeHandle nh_;

    ros::Publisher footstep_update_pub_;
    ros::Subscriber footstep_list_sub_;
    ros::Publisher footstep_undo_req_pub_;
    ros::Subscriber footstep_has_undo_sub_;
    ros::Publisher footstep_redo_req_pub_;
    ros::Subscriber footstep_has_redo_sub_;
    ros::Publisher footstep_start_index_pub_;
    ros::Publisher footstep_validate_req_pub_;
    ros::Publisher footstep_execute_req_pub_;
    ros::Publisher footstep_send_ocs_plan_req_pub_;
    ros::Publisher footstep_stitch_req_pub_;
    ros::Publisher footstep_plan_parameters_pub_;
    ros::Subscriber footstep_plan_parameters_sub_;
    ros::Subscriber footstep_param_set_list_sub_;
    ros::Publisher footstep_param_set_selected_pub_;
    ros::Subscriber footstep_param_set_selected_sub_;

    ros::Subscriber footstep_goal_sub_;
    ros::Publisher footstep_goal_pose_fb_pub_;
    ros::Subscriber footstep_goal_pose_fb_sub_;
    ros::Publisher footstep_plan_goal_pub_;
    ros::Publisher footstep_plan_request_pub_;
    ros::Publisher footstep_plan_update_pub_;

    ros::Publisher interactive_marker_add_pub_;
    ros::Publisher interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;
    ros::Publisher interactive_marker_remove_pub_;

    ros::Subscriber sync_status_sub_;

    vigir_ocs_msgs::OCSFootstepList footstep_list_;

    rviz::VisualizationManager* manager_;

    // marker for individual footsteps
    std::vector<rviz::Display*> display_footstep_marker_list_;
    // marker for step plans
    std::vector<rviz::Display*> display_step_plan_marker_list_;

    // marker for goal
    rviz::Display* display_goal_marker_;
    rviz::Display* display_goal_footstep_marker_[2]; //0 left 1 right

    // new displays for walking
    rviz::Display* footsteps_array_;
    rviz::Display* goal_steps_array_;
    rviz::Display* goal_pose_;
    rviz::Display* planner_start_;
    rviz::Display* planned_path_;
    rviz::Display* footsteps_path_body_array_;
    rviz::Display* planner_terrain_classifier_cloud_processed_;
    rviz::Display* planner_plan_request_feedback_cloud_;

    int start_step_index_;

    // footstep parameters set from ui
    float max_time_;
    int max_steps_;
    float path_length_ratio_;
    int edit_mode_;
    bool use_3d_planning_;

    // variables that determine the state of the footstep plan
    bool has_goal_;
    bool has_valid_step_plan_;
    bool can_validate_;
    unsigned char has_undo_;
    unsigned char has_redo_;
    bool need_plan_update_;
    unsigned int num_step_plans_;
    unsigned char validate_mode_;

    // avoid detecting double click as dragging interactive marker
    boost::posix_time::ptime double_click_timer_;
    bool button_down_;

    QBasicTimer timer_;
};

}
#endif // FOOTSTEP_VIS_MANAGER_H
