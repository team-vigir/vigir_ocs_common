// test app for the main_3d_view widget

#include <QApplication>
#include <ros/ros.h>
#include "main_3d_view.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "main_3d_view", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::Main3DView* main_3d_view = new vigir_ocs::Main3DView();
  main_3d_view->show();

  app.exec();

  delete main_3d_view;
}
