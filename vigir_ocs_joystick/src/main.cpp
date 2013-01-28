// test app for the joystick widget

#include <QApplication>
#include <ros/ros.h>
#include "joystick.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "joystick", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  Joystick* joystick = new Joystick();
  joystick->show();

  app.exec();

  delete joystick;
}
