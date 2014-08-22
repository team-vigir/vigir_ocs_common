#include <QtGui/QApplication>
#include "glancehubSbar.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "glance_hub_sbar_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    glancehubSbar w;
    w.show();

    return a.exec();
}
