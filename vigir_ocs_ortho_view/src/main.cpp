#include <QtGui/QApplication>
#include <ros/ros.h>
#include "ui/ortho_view_widget.h"

int main(int argc, char *argv[])
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "ortho_view", ros::init_options::AnonymousName );
  }

  QApplication a(argc, argv);
  OrthoViewWidget w;
  w.show();
  
  return a.exec();
}
