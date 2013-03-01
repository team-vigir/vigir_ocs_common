#include <QApplication>
#include <ros/ros.h>
#include "ui/image_viewer_custom_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "image_viewer_custom", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  ImageViewerCustomWidget w;
  w.show();

  return a.exec();
}
