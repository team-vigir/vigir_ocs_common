#include <QApplication>
#include <ros/ros.h>
#include "ui/camera_viewer_custom_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_viewer_custom", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  CameraViewerCustomWidget w;
  w.show();

  return a.exec();
}
