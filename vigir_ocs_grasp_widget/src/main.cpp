#include <QtGui/QApplication>
#include "graspWidget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    graspWidget w;
    w.show();
    
    return a.exec();
}
