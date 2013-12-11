#include "global_hotkey.h"

namespace vigir_ocs
{
GlobalHotkey::GlobalHotkey() :
    registering_keystrokes_(true)
{
    ros::NodeHandle nh_out(n_, "/flor/ocs");

    // create publishers for visualization
    key_event_pub_  = nh_out.advertise<flor_ocs_msgs::OCSKeyEvent>( "key_event", 1, false );
    
    int depth,screen,connection;
	display_name_ = XOpenDisplay(NULL);
    //screen = DefaultScreen(display_name_);
	//depth = DefaultDepth(display_name_,screen);
	//connection = ConnectionNumber(display_name_);
	//printf("Keylogger started\n\nInfo about X11 connection:\n");
	//printf(" The display is::%s\n",XDisplayName((char*)display_name_));
	//printf(" Width::%d\tHeight::%d\n", DisplayWidth(display_name_,screen), DisplayHeight(display_name_,screen));
	//printf(" Connection number is %d\n",connection);
	
    memset(old_keys_,false,4096*sizeof(bool));
}

GlobalHotkey::~GlobalHotkey()
{
    if(registering_keystrokes_)
        XCloseDisplay(display_name_);
}

bool GlobalHotkey::isActive()
{
    return registering_keystrokes_;
}

void GlobalHotkey::calculateCursorPosition(int &x, int &y)
{
    int number_of_screens;
    int i;
    Bool result;
    Window *root_windows;
    Window window_returned;
    int root_x, root_y;
    int win_x, win_y;
    unsigned int mask_return;

    number_of_screens = XScreenCount(display_name_);
    //fprintf(stderr, "There are %d screens available in this X session\n", number_of_screens);
    root_windows = (Window*)malloc(sizeof(Window) * number_of_screens);
    for(i = 0; i < number_of_screens; i++)
    {
        root_windows[i] = XRootWindow(display_name_, i);
    }
    for(i = 0; i < number_of_screens; i++)
    {
        result = XQueryPointer(display_name_, root_windows[i], &window_returned,
                &window_returned, &root_x, &root_y, &win_x, &win_y,
                &mask_return);
        if(result)
            break;
    }
    if(result != True)
    {
        fprintf(stderr, "No mouse found.\n");
        x = -1;
        y = -1;
    }
    else
    {
        //printf("Mouse is at (%d,%d)\n", root_x, root_y);
        x = root_x;
        y = root_y;
    }
}

void GlobalHotkey::onUpdate()
{
	bool new_keys[4096];
	memset(new_keys,false,4096*sizeof(bool));
	
    // grab keys
	char keys_return[32];
	XQueryKeymap(display_name_,keys_return);
	for(int i = 0; i < 32; i++) 
	{
        //if (keys_return[i] != 0)
		{
			int pos = 0;
            unsigned char num = keys_return[i];
			while (pos < 8) 
			{
				if ((num & 0x01) == 1) 
				{
                    //printf("%d ",i*8+pos);
					new_keys[i*8+pos] = true;
				}
				pos++; num /= 2;
			}

            printf("%x ",(unsigned char)keys_return[i]);
		}
	}
    printf("\n");

    // RUN 'xev' TO GET KEYCODES

    int x,y;
    calculateCursorPosition(x,y);
	
    if(registering_keystrokes_)
    {
        for(int i = 0; i < 4096; i++)
        {
            // check if key was pressed/released before updating old_keys_ array
            if(!old_keys_[i] && new_keys[i])
            {
                //printf("key pressed (%d); cursor at %d, %d\n",i,x,y);
                publishKeyPressed(i,x,y);
            }
            else if(old_keys_[i] && !new_keys[i])
            {
                //printf("key released (%d); cursor at %d, %d\n",i,x,y);
                publishKeyReleased(i,x,y);
            }
        }
    }

    // check if state has changed
    int state_changed = memcmp(old_keys_, new_keys, 4096*sizeof(bool));

    // save key states
    memcpy(old_keys_, new_keys, 4096*sizeof(bool));

    //printf("key states(%d): %d, %d, %d\n", state_changed, old_keys_[37], old_keys_[64], old_keys_[45]);

    if(old_keys_[37] && old_keys_[64] && old_keys_[45] && state_changed != 0) // ctrl + alt + k
        registering_keystrokes_ = !registering_keystrokes_;
}

void GlobalHotkey::publishKeyPressed(int k, int x, int y)
{
    flor_ocs_msgs::OCSKeyEvent cmd;

    cmd.key = k;
    cmd.state = 1;
    cmd.cursor_x = x;
    cmd.cursor_y = y;

    // publish key event and cursor position information
    key_event_pub_.publish(cmd);
}

void GlobalHotkey::publishKeyReleased(int k, int x, int y)
{
    flor_ocs_msgs::OCSKeyEvent cmd;

    cmd.key = k;
    cmd.state = 0;
    cmd.cursor_x = x;
    cmd.cursor_y = y;

    // publish key event and cursor position information
    key_event_pub_.publish(cmd);
}

}
