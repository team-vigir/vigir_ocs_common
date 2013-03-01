// test app for the camera_viewer widget
#include <QApplication>
#include <ros/ros.h>
#include "image_viewer_custom.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "image_viewer_custom", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  ImageViewerCustom* image_viewer_custom = new ImageViewerCustom();
  image_viewer_custom->show();

  app.exec();

  delete image_viewer_custom;
}
