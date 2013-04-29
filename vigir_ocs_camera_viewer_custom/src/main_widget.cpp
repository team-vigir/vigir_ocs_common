// test app for the camera_viewer widget
#include <QApplication>
#include <ros/ros.h>
#include "camera_viewer_custom.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_viewer_custom", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::CameraViewerCustom* camera_viewer_custom = new vigir_ocs::CameraViewerCustom();
  camera_viewer_custom->show();

  app.exec();

  delete camera_viewer_custom;
}
