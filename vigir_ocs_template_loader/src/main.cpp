#include <QApplication>
#include <ros/ros.h>
#include "ui/template_loader_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "template_loader", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  TemplateLoaderWidget w;
  w.show();

  return a.exec();
}
