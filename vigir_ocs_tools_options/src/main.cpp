#include <QApplication>
#include <ros/ros.h>
#include "ui/tools_options_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "tools_options", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  ToolsOptionsWidget w;
  w.show();

  return a.exec();
}
