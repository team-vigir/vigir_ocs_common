/* 
 * Base3DView class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef BASE_3D_VIEW_H
#define BASE_3D_VIEW_H

#include <QWidget>
#include <QMouseEvent>
#include <OGRE/OgreVector3.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>

namespace rviz
{
class Display;
class Tool;
class RenderPanel;
class RenderPanelCustom;
class VisualizationManager;
class FrameManager;
}

namespace vigir_ocs
{
class MouseEventHandler;

// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Base3DView: public QWidget
{
    Q_OBJECT
public:
    Base3DView( std::string base_frame = "/pelvis", QWidget* parent = 0 );
    virtual ~Base3DView();

    void processNewMap(const nav_msgs::OccupancyGrid::ConstPtr& pose);

public Q_SLOTS:
    // displays
    void robotModelToggled( bool );
    void lidarPointCloudToggled( bool );
    void stereoPointCloudToggled( bool );
    void laserScanToggled( bool );
    void markerArrayToggled( bool );
    void gridMapToggled( bool );
    void footstepPlanningToggled( bool );
    void simulationRobotToggled( bool );
    // tools
    void cameraToggled( bool );
    void selectToggled( bool );
    void select3DToggled( bool );
    void markerRobotToggled( bool );
    void markerTemplateToggled( bool );
    void vectorPressed();

    void newSelection( Ogre::Vector3 );
    void insertTemplate( QString );
    void templatePathChanged( QString );
    void insertWaypoint();

    void createContextMenu( int, int );
    // sends back the context
    void setContext( int );

Q_SIGNALS:
    void setRenderPanel( rviz::RenderPanel* );
    void resetSelection();
    void setMarkerScale( float );
    // send position of the mouse when clicked to create context menu
    void queryContext( int, int );

protected:
    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);
    void transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame);

    rviz::VisualizationManager* manager_;
    rviz::VisualizationManager* manager_simulation_;
    rviz::RenderPanel* render_panel_;

    rviz::Display* robot_model_;
    rviz::Display* interactive_marker_robot_[4];
    rviz::Display* interactive_marker_template_;
    rviz::Display* octomap_;
    rviz::Display* laser_scan_;
    rviz::Display* lidar_point_cloud_viewer_;
    rviz::Display* stereo_point_cloud_viewer_;
    rviz::Display* selection_3d_display_;
    rviz::Display* template_display_;
    rviz::Display* waypoints_display_;
    rviz::Display* achieved_waypoints_display_;
    rviz::Display* octomap_roi_;
    rviz::Display* point_cloud_request_viewer_;

    // new displays for walking
    rviz::Display* footsteps_array_;
    std::vector<rviz::Display*> ground_map_;
    rviz::Display* goal_pose_;
    rviz::Display* planner_start_;
    rviz::Display* planned_path_;

    rviz::Display* hand_model_;

    // for simulation
    rviz::Display* ghost_robot_model_;

    rviz::Tool* interactive_markers_tool_;
    rviz::Tool* selection_tool_;
    rviz::Tool* move_camera_tool_;
    rviz::Tool* set_goal_tool_;

    Ogre::Vector3 selection_position_;

    ros::NodeHandle n_;

    ros::Publisher template_add_pub_;
    ros::Publisher waypoint_add_pub_;

    ros::Publisher octomap_roi_pub_;

    ros::Subscriber ground_map_sub_;
    
    vigir_ocs::MouseEventHandler* mouse_event_handler_;

    std::string base_frame_;

    bool selected_;
    QString selected_template_path_;

    int active_context_;
};
}
#endif // BASE_3D_VIEW_H
