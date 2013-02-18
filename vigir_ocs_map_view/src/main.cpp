#include <QtGui/QApplication>
#include <ros/ros.h>
#include "ui/map_view_widget.h"

int main(int argc, char *argv[])
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "map_view", ros::init_options::AnonymousName );
  }

  QApplication a(argc, argv);
  MapViewWidget w;
  w.show();
  
  return a.exec();
}
