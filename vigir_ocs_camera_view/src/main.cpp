#include <QApplication>
#include <ros/ros.h>
#include "ui/camera_view_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_view", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  CameraViewWidget w;
  w.show();

  return a.exec();
}
