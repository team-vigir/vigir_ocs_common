#include <QtGui/QApplication>
#include <ros/ros.h>
#include "ui/map_view_widget.h"

int main(int argc, char *argv[])
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "map_view", ros::init_options::AnonymousName );
  }

  QApplication a(argc, argv);
  QPixmap pixmap(1,1);
  pixmap.fill(QColor(52,156,219));
  QIcon icon(pixmap);
  a.setWindowIcon(icon);
  MapViewWidget w;
  w.show();
  
  return a.exec();
}
