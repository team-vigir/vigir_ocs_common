/*
 * FootstepVisManager class definition.
 *
 * Author: Felipe Bacim.
 */

#ifndef FOOTSTEP_VIS_MANAGER_H
#define FOOTSTEP_VIS_MANAGER_H

#include <QObject>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <flor_interactive_marker_server_custom/interactive_marker_server_custom.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerAdd.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <flor_ocs_msgs/OCSFootstepList.h>
#include <flor_ocs_msgs/OCSFootstepUpdate.h>

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
      * Receives list of footsteps and creates/removes interactive markers
      */
    void processFootstepList(const flor_ocs_msgs::OCSFootstepList::ConstPtr& msg);

    /**
      * ROS Callback: receives interactive marker pose updates
      */
    void onMarkerFeedback( const flor_ocs_msgs::OCSInteractiveMarkerUpdate& msg );//std::string topic_name, geometry_msgs::PoseStamped pose);

    /**
      * Select footstep using the context menu
      */
    void selectContextMenu();

public Q_SLOTS:
    /**
      * Set visibility of all footstep-related displays
      */
    void setEnabled(bool enabled);

Q_SIGNALS:
    // add qsignals here

private:
    void updateInteractiveMarkers();

    ros::NodeHandle nh_;

    ros::Publisher footstep_update_sub_;
    ros::Subscriber footstep_list_pub_;

    ros::Publisher interactive_marker_add_pub_;
    ros::Publisher interactive_marker_update_pub_;
    ros::Subscriber interactive_marker_feedback_sub_;
    ros::Publisher interactive_marker_remove_pub_;

    flor_ocs_msgs::OCSFootstepList footstep_list_;

    rviz::VisualizationManager* manager_;

    std::vector<rviz::Display*> display_footstep_marker_list_;

    // new displays for walking
    rviz::Display* footsteps_array_;
    rviz::Display* goal_pose_walk_;
    rviz::Display* goal_pose_step_;
    rviz::Display* planner_start_;
    rviz::Display* planned_path_;
    rviz::Display* footsteps_path_body_array_;
};

}
#endif // FOOTSTEP_VIS_MANAGER_H
