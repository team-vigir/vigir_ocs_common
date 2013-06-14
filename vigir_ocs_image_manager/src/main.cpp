#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/image_manager_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "image_manager", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  ImageManagerWidget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
