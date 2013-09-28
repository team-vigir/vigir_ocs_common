#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/ghost_control_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "ghost_control_widget", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  GhostControlWidget w;
  //w.show();
  w.setMinimumSize(300,100);
  w.hide();

  return a.exec();
}
