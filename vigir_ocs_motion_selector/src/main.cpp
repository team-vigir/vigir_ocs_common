#include <QtGui/QApplication>
#include "motion_selector.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
            {
              ros::init( argc, argv, "motion_selector", ros::init_options::AnonymousName );
            }
    QApplication a(argc, argv);
    motion_selector w;
    w.show();

    return a.exec();
}
