// test app for the ortho_view widget

#include <QApplication>
#include <ros/ros.h>
#include "ortho_view.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "ortho_view", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  vigir_ocs::OrthoView* ortho_view = new vigir_ocs::OrthoView();
  ortho_view->show();

  app.exec();

  delete ortho_view;
}
