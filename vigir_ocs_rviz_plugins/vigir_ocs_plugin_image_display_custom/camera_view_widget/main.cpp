#include <QtGui/QApplication>
#include "camera_view_widget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Camera_View_Widget w;
    w.show();
    
    return a.exec();
}
