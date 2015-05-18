#include <ros/ros.h>
#include "interactive_marker_server_nodelet.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "interactive_marker_server", ros::init_options::AnonymousName );
    }

    ocs_interactive_marker_server::InteractiveMarkerServerNodelet w;

    while(ros::ok())
    {
        ros::spinOnce();
        usleep(1000);
    }

}
