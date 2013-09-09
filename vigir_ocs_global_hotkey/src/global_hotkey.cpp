#include "global_hotkey.h"

namespace ocs_global_hotkey
{
GlobalHotkey::GlobalHotkey()
{
    ros::NodeHandle nh_out(n_, "hotkey");

    // create publishers for visualization
    hotkey_pressed_pub_  = nh_out.advertise<std_msgs::Int32>( "pressed", 1, false );
    hotkey_released_pub_ = nh_out.advertise<std_msgs::Int32>( "released", 1, false );
    
    int depth,screen,connection;
	display_name_ = XOpenDisplay(NULL);
	screen = DefaultScreen(display_name_);
	//depth = DefaultDepth(display_name_,screen);
	//connection = ConnectionNumber(display_name_);
	//printf("Keylogger started\n\nInfo about X11 connection:\n");
	//printf(" The display is::%s\n",XDisplayName((char*)display_name_));
	//printf(" Width::%d\tHeight::%d\n", DisplayWidth(display_name_,screen), DisplayHeight(display_name_,screen));
	//printf(" Connection number is %d\n",connection);
	
	memset(keys,false,4096*sizeof(bool));
}

GlobalHotkey::~GlobalHotkey()
{
	XCloseDisplay(display_name_);
}

void GlobalHotkey::onUpdate()
{
	bool new_keys[4096];
	memset(new_keys,false,4096*sizeof(bool));
	
	char keys_return[32];
	XQueryKeymap(display_name_,keys_return);
	for(int i = 0; i < 32; i++) 
	{
		if (keys_return[i] != 0) 
		{
			int pos = 0;
			int num = keys_return[i];
			while (pos < 8) 
			{
				if ((num & 0x01) == 1) 
				{
					//printf("%d ",i*8+pos);
					new_keys[i*8+pos] = true;
				}
				pos++; num /= 2;
			}
		}
	}
	//printf("\n");
	
	for(int i = 0; i < 4096; i++)
	{
		if(keys[i] == false && new_keys[i] == true)
		{
			printf("key pressed (%d)\n",i);
			keys[i] = true;
		}
		else if(keys[i] == true && new_keys[i] == false)
		{
			printf("key released (%d)\n",i);
			keys[i] = false;
		}
	}
}

void GlobalHotkey::publishKeyPressed(int k)
{
    std_msgs::Int32 key;
    //key.
	
    // publish complete list of templates and poses
    //waypoint_list_pub_.publish( waypoint_list_ );
}

void GlobalHotkey::publishKeyReleased(int k)
{
    //waypoint_achieved_list_.header.frame_id = "/world";
    //waypoint_achieved_list_.header.stamp = ros::Time::now();
    // publish complete list of templates and poses
    //waypoint_achieved_list_pub_.publish( waypoint_achieved_list_ );
}

}
