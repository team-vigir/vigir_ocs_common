#include "ros/ros.h"
#include "global_hotkey.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vigir_ocs_hotkey_node");

    vigir_ocs::GlobalHotkey global_hotkey;
    
    while(ros::ok())
    {
        global_hotkey.onUpdate();
	    ros::spinOnce();
        usleep(3000);
    }

    return 0;
}
