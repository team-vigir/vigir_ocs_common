#include <QtGui/QApplication>
#include "behavior_relay.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "Behavior_Relay_Widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    BehaviorRelay w;
    w.show();

    return a.exec();
}
