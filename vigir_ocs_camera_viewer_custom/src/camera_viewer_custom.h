/* 
 * CameraViewerCustom class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/04/2012):
 */

#ifndef CAMERA_VIEWER_H
#define CAMERA_VIEWER_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
class Tool;
}

// Class "CameraViewerCustom" implements the QWidget that can be added to any QT application.
class CameraViewerCustom: public QWidget
{
Q_OBJECT
public:
  CameraViewerCustom( QWidget* parent = 0 );
  virtual ~CameraViewerCustom();

Q_SIGNALS:
  void setFullImageResolution( int );
  void setCropImageResolution( int );
  void setCameraSpeed( int );

public Q_SLOTS:
  void changeCameraTopic( int );
  void changeFullImageResolution( int );
  void changeCropImageResolution( int );
  void changeCameraSpeed( int );

private Q_SLOTS:
  void select( int, int, int, int );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* camera_viewer_;

  rviz::Tool* selection_tool_;
};
#endif // CAMERA_VIEWER_H
