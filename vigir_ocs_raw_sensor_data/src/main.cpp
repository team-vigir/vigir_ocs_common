// test app for the raw_sensor_data widget

#include <QApplication>
#include <ros/ros.h>
#include "raw_sensor_data.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "raw_sensor_data", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  RawSensorData* raw_sensor_data = new RawSensorData();
  raw_sensor_data->show();

  app.exec();

  delete raw_sensor_data;
}
