#include "ros/ros.h"
#include "space_mouse.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "space_mouse_node");

    vigir_ocs::SpaceMouse space_mouse;
    
    while(ros::ok())
    {
        //hotkey_relay.onUpdate();
	    ros::spinOnce();
        usleep(3000);
    }

    return 0;
}
