// test app for the point_cloud_viewer widget

#include <QApplication>
#include <ros/ros.h>
#include "point_cloud_viewer.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "point_cloud_viewer", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  PointCloudViewer* point_cloud_viewer = new PointCloudViewer();
  point_cloud_viewer->show();

  app.exec();

  delete point_cloud_viewer;
}
