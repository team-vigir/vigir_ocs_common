// test app 

#include <QApplication>
#include <QHBoxLayout>
#include <ros/ros.h>
#include "camera_viewer.h"
#include "main_3d_view.h"
#include "point_cloud_viewer.h"
#include "raw_sensor_data.h"
#include "robot_model.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "integration_test", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  QWidget* integrated_view = new QWidget();

  QHBoxLayout* main_layout = new QHBoxLayout;
  main_layout->addWidget( new CameraViewer() );
  main_layout->addWidget( new Main3DView() );
  main_layout->addWidget( new PointCloudViewer() );
  main_layout->addWidget( new RawSensorData() );
  main_layout->addWidget( new RobotModel() );

	integrated_view->setLayout( main_layout );

  integrated_view->show();

  app.exec();

  delete integrated_view;
  delete main_layout;
}
