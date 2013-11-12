#include <QtGui/QApplication>
#include "glancehubsbar.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "glance_hub_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    glancehubsbar w;
    w.show();

    return a.exec();
}
