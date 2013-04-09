/* 
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef MAIN_3D_VIEW_H
#define MAIN_3D_VIEW_H

#include <QWidget>
#include <QMouseEvent>
#include <OGRE/OgreVector3.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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
class SelectionHandler;
}

// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Main3DView: public QWidget
{
    Q_OBJECT
public:
    Main3DView( QWidget* parent = 0 );
    virtual ~Main3DView();

public Q_SLOTS:
    // displays
    void robotModelToggled( bool );
    void lidarPointCloudToggled( bool );
    void stereoPointCloudToggled( bool );
    void laserScanToggled( bool );
    void markerArrayToggled( bool );
    // tools
    void cameraToggled( bool );
    void selectToggled( bool );
    void select3DToggled( bool );
    void markerRobotToggled( bool );
    void markerTemplateToggled( bool );
    void vectorToggled( bool );

    void newSelection( Ogre::Vector3 );
    void insertTemplate( QString );
    void insertWaypoint();

Q_SIGNALS:
    void setRenderPanel( rviz::RenderPanel* );
    void rightClickEvent( int, int );

private:
    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);

    rviz::VisualizationManager* manager_;
    rviz::RenderPanelCustom* render_panel_;

    rviz::Display* robot_model_;
    rviz::Display* interactive_marker_robot_[4];
    rviz::Display* octomap_;
    rviz::Display* laser_scan_;
    rviz::Display* lidar_point_cloud_viewer_;
    rviz::Display* stereo_point_cloud_viewer_;
    rviz::Display* template_display_;
    rviz::Display* selection_3d_display_;
    rviz::Display* waypoints_display_;
    rviz::Display* achieved_waypoints_display_;
    rviz::Display* octomap_roi_;

    rviz::Tool* interactive_markers_tool_;
    rviz::Tool* selection_tool_;
    rviz::Tool* move_camera_tool_;
    rviz::Tool* set_goal_tool_;

    Ogre::Vector3 selection_position_;

    ros::NodeHandle n_;

    ros::Publisher template_add_pub_;
    ros::Publisher waypoint_add_pub_;

    ros::Publisher octomap_roi_pub_;
    
    vigir_ocs::SelectionHandler* selection_handler_;
};
#endif // MAIN_3D_VIEW_H
