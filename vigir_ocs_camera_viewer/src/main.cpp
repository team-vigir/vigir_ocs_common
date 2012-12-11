// test app for the camera_viewer widget

#include <QApplication>
#include <ros/ros.h>
#include "camera_viewer.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_viewer", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  CameraViewer* camera_viewer = new CameraViewer();
  camera_viewer->show();

  app.exec();

  delete camera_viewer;
}
