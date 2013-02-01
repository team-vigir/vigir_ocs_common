/* 
 * RawSensorDataInput class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/12/2012):
 *
 */

#ifndef RAW_SENSOR_DATA_INPUT_H
#define RAW_SENSOR_DATA_INPUT_H

#include <QWidget>
#include <QTableWidget>	
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

// Class "RawSensorDataInput" implements the QWidget that can be added to any QT application.
class RawSensorDataInput: public QWidget
{
Q_OBJECT
public:
  RawSensorDataInput( QWidget* parent = 0 );
  virtual ~RawSensorDataInput();

	void updateTable( const sensor_msgs::JointStateConstPtr& joint_states );
	
private Q_SLOTS:
  void positionChanged( int row, int column );

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* robot_model_;
  
  ros::Subscriber joint_states_;

  QTableWidget* table;
  
	std_msgs::Float64 cmd;

	ros::NodeHandle n_;
	ros::Publisher pos_pub_;
	
	std::vector<std::string> joint_list_;

	std::string topic;
};
#endif // RAW_SENSOR_DATA_INPUT_H
