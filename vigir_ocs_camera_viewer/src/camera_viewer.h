/* 
 * CameraViewer class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/04/2012):
 * - created class
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
}

// Class "CameraViewer" implements the QWidget that can be added to any QT application.
class CameraViewer: public QWidget
{
Q_OBJECT
public:
  CameraViewer( QWidget* parent = 0 );
  virtual ~CameraViewer();

// TODO: create slots for possible UI stuff
//private Q_SLOTS:

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* camera_viewer_;
};
#endif // CAMERA_VIEWER_H
