#include <QtGui/QApplication>
#include "widget.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "time_widget");
    }

    QApplication a(argc, argv);
    Widget w;
    w.show();
    
    return a.exec();
}
