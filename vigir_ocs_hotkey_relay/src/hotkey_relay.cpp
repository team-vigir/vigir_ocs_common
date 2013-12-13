#include "hotkey_relay.h"

namespace vigir_ocs
{
HotkeyRelay::HotkeyRelay() 
{
    ros::NodeHandle nh_out(n_, "/flor/ocs");

    // create publishers for visualization
    key_event_pub_  = nh_out.advertise<flor_ocs_msgs::OCSKeyEvent>( "key_event", 1, false );

    key_event_sub_ = nh_out.subscribe<flor_ocs_msgs::OCSKeyEvent>( "secondary/key_event", 5, &HotkeyRelay::processKeyEvent, this );
	
    memset(old_keys_,false,4096*sizeof(bool));
}

HotkeyRelay::~HotkeyRelay()
{
}

void HotkeyRelay::processKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());



    if(key_event->key == 37 && key_event->state) // send ctrl
    {
        publishKeyPressed(37,key_event->cursor_x,key_event->cursor_y);
    }
    else if(key_event->key == 10 && key_event->state && ctrl_is_pressed) // ctrl+1
    {
        publishKeyPressed(10,key_event->cursor_x,key_event->cursor_y); // send 1
        // sleep
        publishKeyReleased(10,key_event->cursor_x,key_event->cursor_y); // send 1
    }

}

void HotkeyRelay::publishKeyPressed(int k, int x, int y)
{
    flor_ocs_msgs::OCSKeyEvent cmd;

    cmd.key = k;
    cmd.state = 1;
    cmd.cursor_x = x;
    cmd.cursor_y = y;

    // publish key event and cursor position information
    key_event_pub_.publish(cmd);
}

void HotkeyRelay::publishKeyReleased(int k, int x, int y)
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
