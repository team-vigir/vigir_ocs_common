#include <QtGui/QApplication>
#include "statusBar.h"
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "statusBar_widget", ros::init_options::AnonymousName );
    }
    QApplication a(argc, argv);
    StatusBar w;
    w.show();

    return a.exec();
}
