/* 
 * PointCloudViewer class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/06/2012):
 * - created class
 */

#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "PointCloudViewer" implements the QWidget that can be added to any QT application.
class PointCloudViewer: public QWidget
{
Q_OBJECT
public:
  PointCloudViewer( QWidget* parent = 0 );
  virtual ~PointCloudViewer();

// TODO: create slots for possible UI stuff
//private Q_SLOTS:

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* point_cloud_viewer_;
};
#endif // POINT_CLOUD_VIEWER_H
