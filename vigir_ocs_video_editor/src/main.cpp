#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/video_editor_widget.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "video_editor_manager", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );

  VideoEditorWidget w;
  w.show();
  w.setMinimumSize(300,100);

  return a.exec();
}
