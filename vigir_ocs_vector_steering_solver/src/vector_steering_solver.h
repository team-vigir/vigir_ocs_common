/* 
 * VectorSteeringSolver class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Latest changes (12/12/2012):
 *
 */

#ifndef VECTOR_STEERING_SOLVER_H
#define VECTOR_STEERING_SOLVER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <vector>
#include <string>

// Class "VectorSteeringSolver" implements a solver for the vector steering tool.
// Subscribes to vector goal messages and publishes movement vector.
class VectorSteeringSolver
{
public:
  VectorSteeringSolver();
  virtual ~VectorSteeringSolver();

	void solveVector( const geometry_msgs::PoseStampedConstPtr& pose );
	
private:
	double walk_vel, run_vel, yaw_rate, yaw_rate_run;
	geometry_msgs::Twist cmd_twist_;
	geometry_msgs::Pose cmd_pose_;

	ros::NodeHandle n_;
	
	ros::Publisher vel_pub_;
	ros::Publisher pose_pub_;
  ros::Subscriber steering_vector_;
};
#endif // VECTOR_STEERING_SOLVER_H
