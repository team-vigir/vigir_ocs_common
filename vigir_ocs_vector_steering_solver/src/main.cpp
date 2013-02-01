// 

#include <ros/ros.h>
#include "vector_steering_solver.h"

int main(int argc, char **argv)
{
  ros::init( argc, argv, "vector_steering_solver", ros::init_options::AnonymousName );

  VectorSteeringSolver* vector_steering_solver = new VectorSteeringSolver();

	ros::spin();
}

