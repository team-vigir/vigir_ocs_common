// test app for the camera_viewer widget

#include <QApplication>
#include <ros/ros.h>
#include "camera_viewer_custom.h"

// For the plugin to work, we need to add this to rviz/src/rviz/default_plugin/init.cpp
//#include "image_display_custom.h"
//extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
//{
//	reg->registerDisplay<AxesDisplay>("rviz::ImageDisplayCustom");


int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "camera_viewer_custom", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  CameraViewerCustom* camera_viewer_custom = new CameraViewerCustom();
  camera_viewer_custom->show();

  app.exec();

  delete camera_viewer_custom;
}
