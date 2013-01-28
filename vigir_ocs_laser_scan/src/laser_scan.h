/* 
 * LaserScan class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (11/28/2012):
 * - created class
 */

#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "LaserScan" implements the QWidget that can reads the LaserScan topic.
class LaserScan: public QWidget
{
Q_OBJECT
public:
  LaserScan( QWidget* parent = 0 );
  virtual ~LaserScan();

// TODO: create slots for possible UI stuff
//private Q_SLOTS:
//  setCollision( bool );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* robot_model_;
  rviz::Display* laser_scan_;
};
#endif // LASER_SCAN_H
