#include <QtGui/QApplication>
#include "graspWidget.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "grasp_widget", ros::init_options::AnonymousName );
    }

    QApplication a(argc, argv);
    graspWidget w;
    w.show();
    
    return a.exec();
}
