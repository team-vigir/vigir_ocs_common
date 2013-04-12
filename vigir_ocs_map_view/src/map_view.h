/* 
 * MapView class definition.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials and the .
 *
 * Latest changes (12/08/2012):
 */

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <OGRE/OgreVector3.h>

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
// Class "MapView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class MapView: public QWidget
{
    Q_OBJECT
public:
    MapView( QWidget* parent = 0 );
    virtual ~MapView();

public Q_SLOTS:
    // displays
    void robotModelToggled( bool );
    void lidarPointCloudToggled( bool );
    void stereoPointCloudToggled( bool );
    void laserScanToggled( bool );
    void markerArrayToggled( bool );
    void markerTemplateToggled( bool );
    // tools
    void cameraToggled( bool );
    void selectToggled( bool selected );
    void markerRobotToggled( bool );
    void waypointToggled( bool );
    void vectorToggled( bool );
    void joystickToggled( bool );

    void newSelection( Ogre::Vector3 );
    void insertTemplate( QString );
    void insertWaypoint();

Q_SIGNALS:
    void setRenderPanel( rviz::RenderPanel* );

private:
    void transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose);

    rviz::VisualizationManager* manager_;
    rviz::RenderPanelCustom* render_panel_;

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

    rviz::Tool* interactive_markers_tool_;
    rviz::Tool* selection_tool_;
    rviz::Tool* move_camera_tool_;
    rviz::Tool* set_goal_tool_;

    Ogre::Vector3 selection_position_;

    ros::NodeHandle n_;

    ros::Publisher template_add_pub_;
    ros::Publisher waypoint_add_pub_;

    vigir_ocs::SelectionHandler* selection_handler_;
};
#endif // map_view_H
