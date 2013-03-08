#include <QtGui/QApplication>
#include "robotStatus.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    robotStatus w;
    w.show();
    
    return a.exec();
}
