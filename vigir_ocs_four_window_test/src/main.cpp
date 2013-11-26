#include <QtGui/QApplication>
#include <ros/ros.h>
#include "ui/four_window_test_widget.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "four_window_test", ros::init_options::AnonymousName );
    }

    QApplication a(argc, argv);
    FourWindowTestWidget w;
    if(argc >= 5)
    {
        try
        {
            int x = atoi(argv[1]);
            int y = atoi(argv[2]);
            int width = atoi(argv[3]);
            int height = atoi(argv[4]);
            w.setGeometry(x,y,width,height);
        }
        catch(...)
        {
        }
    }
    w.show();

    return a.exec();
}
