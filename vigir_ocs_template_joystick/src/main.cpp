#include <ros/ros.h>
#include <QtGui/QApplication>
#include "ui/JoystickWidget.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "template_joystick", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    a.setOrganizationName("OCS");
    JoystickWidget w;
    w.show();
    w.hide();
    
    return a.exec();
}
