/* 
 * MarkerArray class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (11/28/2012):
 * - created class
 */

#ifndef MARKER_ARRAY_H
#define MARKER_ARRAY_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "MarkerArray" implements the QWidget for the MarkerArray topic.
class MarkerArray: public QWidget
{
Q_OBJECT
public:
  MarkerArray( QWidget* parent = 0 );
  virtual ~MarkerArray();

// TODO: create slots for possible UI stuff
//private Q_SLOTS:
//  setCollision( bool );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* robot_model_;
  rviz::Display* marker_array_;
};
#endif // MARKER_ARRAY_H
