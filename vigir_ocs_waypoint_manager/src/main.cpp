#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/waypoint_manager_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "waypoint_manager", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  WaypointManagerWidget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
