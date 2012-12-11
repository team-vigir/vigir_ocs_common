/* 
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (11/28/2012):
 * - created class
 */

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "RobotModel" implements the QWidget that can be added to any QT application.
class RobotModel: public QWidget
{
Q_OBJECT
public:
  RobotModel( QWidget* parent = 0 );
  virtual ~RobotModel();

// TODO: create slots for possible UI stuff
//private Q_SLOTS:
//  setCollision( bool );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* robot_model_;
};
#endif // ROBOT_MODEL_H
