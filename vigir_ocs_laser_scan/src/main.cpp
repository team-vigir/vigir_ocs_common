// test app for the robot_model widget

#include <QApplication>
#include <ros/ros.h>
#include "laser_scan.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "laser_scan", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  LaserScan* laser_scan = new LaserScan();
  laser_scan->show();

  app.exec();

  delete laser_scan;
}
