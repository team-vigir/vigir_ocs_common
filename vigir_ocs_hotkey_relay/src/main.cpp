#include "ros/ros.h"
#include "hotkey_relay.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vigir_ocs_hotkey_node");

    vigir_ocs::HotkeyRelay hotkey_relay;
    
    while(ros::ok())
    {
        //hotkey_relay.onUpdate();
	    ros::spinOnce();
        usleep(3000);
    }

    return 0;
}
