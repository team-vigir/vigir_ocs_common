#include <QtGui/QApplication>
#include "robotStatus.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "status_window", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    robotStatus w;
    w.show();
    
    return a.exec();
}
