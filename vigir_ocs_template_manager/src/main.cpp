#include <QApplication>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "ui/template_manager_widget.h"

class Spinner {
public:
    Spinner() :
            shutdown_required(false)
    {
        m_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Spinner::spin, this)));
    }

    ~Spinner() {
            shutdown_required = true;
            m_thread->join();
    }

    void spin() {
            ros::Rate loop(10);
            sleep(1);
            while ( ros::ok() && !shutdown_required ) {
                    ros::spinOnce();
                    loop.sleep();
            }
    }

    private:
    bool shutdown_required;
    boost::shared_ptr<boost::thread> m_thread;
};

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "template_manager", ros::init_options::AnonymousName );
  }

  QApplication a( argc, argv );
  TemplateManagerWidget w;
  w.show();
  w.setMinimumSize(300,100);

  Spinner spinner; // this will create the spinner

  int result = a.exec();

  return result;
}
