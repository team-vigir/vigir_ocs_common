/* 
 * VectorSteeringSolver class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 */

#include <ros/ros.h>

#include "vector_steering_solver.h"

// Constructor for VectorSteeringSolver.  This does most of the work of the class.
VectorSteeringSolver::VectorSteeringSolver()
{	
	// We subscribe to the /move_base_simple/goal messages
	steering_vector_ = n_.subscribe<geometry_msgs::PoseStamped>( "/move_base_simple/goal", 2, &VectorSteeringSolver::solveVector, this );

  // Spin once to register subscribers
	ros::spinOnce();
	
	// Initialize publisher  
  vel_pub_ = n_.advertise<geometry_msgs::Twist>( "atlas/cmd_vel", 10, true );
  // Initialize publisher  
  pose_pub_ = n_.advertise<geometry_msgs::Pose>( "atlas/set_pose", 10, true );

	// Initialize velocity variables
  ros::NodeHandle n_private("~");
  n_private.param( "walk_vel", walk_vel, 0.5 );
  n_private.param( "run_vel", run_vel, 1.0 );
  n_private.param( "yaw_rate", yaw_rate, 1.0 );
  n_private.param( "yaw_run_rate", yaw_rate_run, 1.5 );
}

// Destructor.
VectorSteeringSolver::~VectorSteeringSolver()
{

}

void VectorSteeringSolver::solveVector( const geometry_msgs::PoseStampedConstPtr& pose )
{
	// publish pose first
	// need to change position to current robot position?
	cmd_pose_.position.x = pose->pose.position.x;
	cmd_pose_.position.y = pose->pose.position.y;
	cmd_pose_.position.z = pose->pose.position.z;
	cmd_pose_.orientation.x = pose->pose.orientation.x;
	cmd_pose_.orientation.y = pose->pose.orientation.y;
	cmd_pose_.orientation.z = pose->pose.orientation.z;
	cmd_pose_.orientation.w = pose->pose.orientation.w;
	pose_pub_.publish( cmd_pose_ );
	
	// then publish speed
	cmd_twist_.linear.x = run_vel;
	vel_pub_.publish( cmd_twist_ );
}


