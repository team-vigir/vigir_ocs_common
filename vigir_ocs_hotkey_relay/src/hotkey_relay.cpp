/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#include "hotkey_relay.h"

namespace vigir_ocs
{
HotkeyRelay::HotkeyRelay() 
{
    ros::NodeHandle nh_out(n_, "/flor/ocs");

    // create publishers for visualization
    key_event_pub_  = nh_out.advertise<vigir_ocs_msgs::OCSHotkeyRelay>( "hotkey_relay", 1, false );

    key_event_sub_ = nh_out.subscribe( "secondary/key_event", 5, &HotkeyRelay::processKeyEvent, this );
	
    memset(old_keys_,false,4096*sizeof(bool));
}

HotkeyRelay::~HotkeyRelay()
{
}

void HotkeyRelay::processKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->keycode);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());

    if(key_event->keycode == 10 && key_event->state && ctrl_is_pressed) // ctrl+1
    {
        vigir_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = 1;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->keycode == 15 && key_event->state && ctrl_is_pressed) // ctrl+6
    {
        vigir_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = vigir_ocs_msgs::OCSHotkeyRelay::CLEAR_IMAGE_SELECTED;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->keycode == 18 && key_event->state && ctrl_is_pressed) // ctrl+9
    {
        // rainbow color
        vigir_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = vigir_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_RAINBOW;
        key_event_pub_.publish(cmd);
    }
    else if(key_event->keycode == 19 && key_event->state && ctrl_is_pressed) // ctrl+0
    {
        // intensity
        vigir_ocs_msgs::OCSHotkeyRelay cmd;
        cmd.relay_code = vigir_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_INTENSITY;
        key_event_pub_.publish(cmd);
    }
}
}
