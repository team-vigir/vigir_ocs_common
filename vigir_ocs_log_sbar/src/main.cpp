#include <QtGui/QApplication>
#include "logSbar.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "logSbar_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    LogSbar w;
    w.show();

    return a.exec();
}
