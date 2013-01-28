/* 
 * Joystick class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the pr2 teleop tutorial.
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QWidget>

#include <geometry_msgs/Twist.h>
#include <ros/publisher.h>
#include <ros/ros.h>

// Class "Joystick" implements the widget that can send linear/angular velocity messages to the robot to move it.
class Joystick: public QWidget
{
Q_OBJECT
public:
  Joystick( QWidget* parent = 0 );
  virtual ~Joystick();

private Q_SLOTS:
  void turnLeftPressed();
  void turnLeftReleased();
  void turnRightPressed();
  void turnRightReleased();
  void forwardPressed();
  void forwardReleased();
  void backwardPressed();
  void backwardReleased();

private:  
	double walk_vel, run_vel, yaw_rate, yaw_rate_run;
	geometry_msgs::Twist cmd;

	ros::NodeHandle n_;
	ros::Publisher vel_pub_;
};
#endif // JOYSTICK_H
