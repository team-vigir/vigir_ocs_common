#include <ros/ros.h>
#include <QtGui/QApplication>
#include "glancehub.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "glance_hub_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    glancehub w;
    w.show();

    return a.exec();
}
