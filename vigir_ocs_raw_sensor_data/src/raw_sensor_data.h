/* 
 * RawSensorData class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/12/2012):
 * - created class
 */

#ifndef RAW_SENSOR_DATA_H
#define RAW_SENSOR_DATA_H

#include <QWidget>
#include <QTableWidget>	
#include <sensor_msgs/JointState.h>
#include <ros/subscriber.h>
//#include <nodelet/nodelet.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "RawSensorData" implements the QWidget that can be added to any QT application.
class RawSensorData: public QWidget
{
Q_OBJECT
public:
  RawSensorData( QWidget* parent = 0 );
  virtual ~RawSensorData();

	void updateTable( const sensor_msgs::JointStateConstPtr& joint_states );
	
// TODO: create slots for possible UI stuff
//private Q_SLOTS:

private:

  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* robot_model_;
  
  ros::Subscriber joint_states_;

  QTableWidget* table;
};
#endif // RAW_SENSOR_DATA_H
