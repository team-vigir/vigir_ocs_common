#include <QtGui/QApplication>
#include "joint_limit.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "joint_limit", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    joint_limit w;
    w.show();
    w.hide();

    return a.exec();
}
