// test app for the raw_sensor_data widget

#include <QApplication>
#include <ros/ros.h>
#include "raw_sensor_data_input.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "raw_sensor_data_input", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  RawSensorDataInput* raw_sensor_data_input = new RawSensorDataInput();
  raw_sensor_data_input->show();

  app.exec();

  delete raw_sensor_data_input;
}
