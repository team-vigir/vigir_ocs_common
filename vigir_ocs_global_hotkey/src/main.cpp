#include "ros/ros.h"
#include "global_hotkey.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vigir_ocs_hotkey_nodelet");

    ocs_global_hotkey::GlobalHotkey global_hotkey;
    
    while(1)
    {
    	global_hotkey.onUpdate();
	    ros::spinOnce();
		usleep(30000);
    }

    return 0;
}
