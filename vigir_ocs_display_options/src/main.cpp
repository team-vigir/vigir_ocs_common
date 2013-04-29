#include <QApplication>
#include <ros/ros.h>
#include "ui/display_options_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "display_options", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  DisplayOptionsWidget w;
  w.show();

  return a.exec();
}
