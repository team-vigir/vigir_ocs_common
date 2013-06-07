#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/bandwidth_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "bandwidth_widget", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  BandwidthWidget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
