#include <QtGui/QApplication>
#include <ros/ros.h>
#include "ui/main_3d_view_widget.h"

int main(int argc, char *argv[])
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "main_3d_view", ros::init_options::AnonymousName );
  }

  QApplication a(argc, argv);
  Main3DViewWidget w;
  w.show();
  
  return a.exec();
}
