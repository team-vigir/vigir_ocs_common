#include <QApplication>
#include <ros/ros.h>
#include "ui/joystick_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "joystick", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  JoystickWidget widget;
  widget.show();

  return app.exec();
}
