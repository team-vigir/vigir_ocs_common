// test app for the robot_model widget

#include <QApplication>
#include <ros/ros.h>
#include "robot_model.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "robot_model", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  RobotModel* robot_model = new RobotModel();
  robot_model->show();

  app.exec();

  delete robot_model;
}
