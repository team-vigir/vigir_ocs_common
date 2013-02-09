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

namespace rviz
{
class Display;
class Tool;
class RenderPanel;
class VisualizationManager;
class FrameManager;
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
  void markerToggled( bool );
  void vectorToggled( bool );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;

  rviz::Display* robot_model_;
  rviz::Display* interactive_marker_[4];
  rviz::Display* marker_array_;
  rviz::Display* laser_scan_;
  rviz::Display* lidar_point_cloud_viewer_;
  rviz::Display* stereo_point_cloud_viewer_;

  rviz::Tool* interactive_markers_tool_;
  rviz::Tool* selection_tool_;
  rviz::Tool* move_camera_tool_;
  rviz::Tool* set_goal_tool_;

};
#endif // MAIN_3D_VIEW_H
