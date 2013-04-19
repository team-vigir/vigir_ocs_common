// test app for the map_view widget

#include <QApplication>
#include <ros/ros.h>
#include "map_view.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "map_view", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::MapView* map_view = new vigir_ocs::MapView();
  map_view->show();

  app.exec();

  delete map_view;
}
