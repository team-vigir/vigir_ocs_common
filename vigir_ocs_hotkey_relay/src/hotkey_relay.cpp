#include "hotkey_relay.h"

namespace vigir_ocs
{
HotkeyRelay::HotkeyRelay() 
{
    ros::NodeHandle nh_out(n_, "/flor/ocs");

    // create publishers for visualization
    key_event_pub_  = nh_out.advertise<flor_ocs_msgs::OCSHotkeyRelay>( "hotkey_relay", 1, false );

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

    if(key_event->key == 10 && key_event->state && ctrl_is_pressed) // ctrl+1
    {
        flor_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = 1;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->key == 15 && key_event->state && ctrl_is_pressed) // ctrl+6
    {
        flor_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = flor_ocs_msgs::OCSHotkeyRelay::CLEAR_IMAGE_SELECTED;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->key == 18 && key_event->state && ctrl_is_pressed) // ctrl+9
    {
        // rainbow color
        flor_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = flor_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_RAINBOW;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->key == 19 && key_event->state && ctrl_is_pressed) // ctrl+0
    {
        // intensity
        flor_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = flor_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_INTENSITY;
        key_event_pub_.publish(cmd);
    }
}
}
