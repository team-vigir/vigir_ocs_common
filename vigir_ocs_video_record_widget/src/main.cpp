#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/video_record_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "video_record_widget", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  video_record_widget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
