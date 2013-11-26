// test app for the camera_viewer widget
#include <QApplication>
#include <ros/ros.h>
#include "camera_view.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_view", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::CameraView* camera_view = new vigir_ocs::CameraView();
  camera_view->show();

  app.exec();

  delete camera_view;
}
