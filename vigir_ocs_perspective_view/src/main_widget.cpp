// test app for the perspective_view widget

#include <QApplication>
#include <ros/ros.h>
#include "perspective_view.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "perspective_view", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::PerspectiveView* perspective_view = new vigir_ocs::PerspectiveView();
  perspective_view->show();

  app.exec();

  delete perspective_view;
}
