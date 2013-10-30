#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/image_video_manager_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "image_video_manager", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  ImageVideoManagerWidget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
