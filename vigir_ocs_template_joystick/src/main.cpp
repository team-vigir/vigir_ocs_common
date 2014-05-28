#include <ros/ros.h>
#include <QtGui/QApplication>
#include "ui/mainwindow.h"
//#include "ui/GlWidget.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "template_joystick", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    


    return a.exec();
}
