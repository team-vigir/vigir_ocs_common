// test app for the marker_array widget

#include <QApplication>
#include <ros/ros.h>
#include "marker_array.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "marker_array", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  MarkerArray* marker_array = new MarkerArray();
  marker_array->show();

  app.exec();

  delete marker_array;
}
