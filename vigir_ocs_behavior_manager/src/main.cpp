#include <QtGui/QApplication>
#include "behavior_manager.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "Behavior_Notification_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    BehaviorManager w;
    w.show();

    return a.exec();
}
