#include <QtGui/QApplication>
#include "ui/widget.h"
#include<ros/ros.h>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    if (!ros::isInitialized())
    {
        ros::init(argc, argv,"supwidget");
    }
    Widget w;
    w.show();
    
    return a.exec();
}
