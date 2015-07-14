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
@TODO_ADD_AUTHOR_INFO
/*
 * Hotkey Manager class definition.
 *
 * Author: Brian Wright.
 */

#ifndef HOTKEY_MANAGER_H
#define HOTKEY_MANAGER_H

#include <ros/ros.h>
#include <algorithm>
#include <vigir_ocs_msgs/OCSKeyEvent.h>

/*
 *HotkeyManager to centralize where Hotkeys are processed
 *
 * Note: cannot have a hotkey correspond to two different actions within the same main window. There will be a hotkey manager for map,main,and camera.
 *
 * naming convention for keyCombo = key+key+....+key
 * so ctrl and q is  ctrl+q
 *
 *
 * list of hotkeys..
 * https://external.torcrobotics.com/redmine-drc/projects/drc/wiki/Keyboard_shortcut
 *
 * Could upgrade this manager by flagging bits in an int based on keys pressed.(gets rid of sorting)
 */

class HotkeyManager
{

public:
	static HotkeyManager* Instance();
    void addHotkeyFunction(std::string keyCombo, boost::function<void()> function);

private: 
    HotkeyManager();  // Private so that it can  not be called
    HotkeyManager(HotkeyManager const&){};             // copy constructor is private
    HotkeyManager& operator=(HotkeyManager const&){};  // assignment operator is private
    static HotkeyManager* instance_;

    void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr key_event);
    void callHotkeyFunction(std::string keyCombo);
    void printallHotkeyCombinations();

    std::vector<std::string> keys_pressed_list_;
    ros::NodeHandle nh_;
    ros::Subscriber key_event_sub_;

    void split(const std::string &s, char delim, std::vector<std::string> &elems) ;

    //cant put boost function pointers directly as map type
    typedef boost::function< void () > callback;

    std::map<std::string,callback> hotkey_functions_;

};

#endif // HOTKEY_MANAGER_H
