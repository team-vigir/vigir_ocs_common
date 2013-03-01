#include <QtGui/QApplication>
#include "status_window.h"
#include "jointList.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "status_window", ros::init_options::AnonymousName );
    }

    QApplication a(argc, argv);
    status_window w;
    //jointList j;
   //j.show();
    w.show();
    
    return a.exec();
}
