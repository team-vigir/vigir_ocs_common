#include <QtGui/QApplication>
#include "jointList.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    jointList w;
    w.show();
    
    return a.exec();
}
