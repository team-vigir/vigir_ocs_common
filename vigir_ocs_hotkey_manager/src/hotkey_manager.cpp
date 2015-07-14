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

#include "hotkey_manager.h"

HotkeyManager* HotkeyManager::instance_ = 0;

HotkeyManager::HotkeyManager()
{
    key_event_sub_ = nh_.subscribe( "/flor/ocs/key_event", 5, &HotkeyManager::processNewKeyEvent, this );
}

HotkeyManager * HotkeyManager::Instance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new HotkeyManager();

    return instance_;
}

void HotkeyManager::processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr key_event)
{
    // store key state
    if(key_event->state) //add if pressed
    {
        //treat Control_l and Control_r the same, (also for shift and alt)
        if(key_event->keystr.find("Control") != std::string::npos)
            keys_pressed_list_.push_back("ctrl");
        else if(key_event->keystr.find("Shift") != std::string::npos)
            keys_pressed_list_.push_back("shift");
        else if(key_event->keystr.find("Alt") != std::string::npos)
            keys_pressed_list_.push_back("alt");
        else if(key_event->keystr == "Escape")//may as well keep Escape to lowercase
            keys_pressed_list_.push_back("esc");
        else
            keys_pressed_list_.push_back(key_event->keystr);

        //process hotkeys in a generic manner
        //sort keys pressed, make sure combinations will always be in same order regardless of how they were hard coded
        std::sort (keys_pressed_list_.begin(), keys_pressed_list_.end());

        std::string keyCombo;
        //build expression to determine which keys are pressed.
        for(int i=0;i<keys_pressed_list_.size();i++)
        {
            std::string key = keys_pressed_list_[i];

            if(i == keys_pressed_list_.size()-1) // dont add plus on the end
                keyCombo = keyCombo + key;
            else
                keyCombo = keyCombo + key + "+";
        }

        //ROS_ERROR("Keys pressed: %s", keyCombo.c_str());

        callHotkeyFunction(keyCombo);
    }
    else // remove if released
    {
        // renamed the keys we inserted, so make sure to erase by new name
        if(key_event->keystr.find("Control") != std::string::npos)
            keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), "ctrl"), keys_pressed_list_.end());
        else if(key_event->keystr.find("Shift") != std::string::npos)
            keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), "shift"), keys_pressed_list_.end());
        else if(key_event->keystr.find("Alt") != std::string::npos)
            keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), "alt"), keys_pressed_list_.end());
        else if(key_event->keystr == "Escape")//may as well keep Escape to lowercase
            keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), "esc"), keys_pressed_list_.end());
        else
            keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keystr), keys_pressed_list_.end());
    }
}

void HotkeyManager::callHotkeyFunction(std::string keyCombo)
{
    //gets the boost function then call binded function
    if(hotkey_functions_.find(keyCombo) != hotkey_functions_.end())
        hotkey_functions_[keyCombo]();
//    else
//        ROS_ERROR("keycombo %s does not exist", keyCombo.c_str());
}

//split std::string and throw tokens into vector, idea from http://stackoverflow.com/questions/236129/split-a-string-in-c
void HotkeyManager::split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
}


void HotkeyManager::addHotkeyFunction(std::string keyCombo, boost::function<void()> function)
{        
    //tokenize and throw into vector
    std::vector<std::string> combo;
    split(keyCombo,'+',combo);

    //sort the designated key combination to make sure combinations will always be in same order regardless of how they were hard coded
    std::sort (combo.begin(), combo.end());

    std::string mappedCombo;
    //build final combination
    for(int i=0;i<combo.size();i++)
    {
        std::string key(combo[i]);

        if(i == combo.size()-1) // dont add plus on the end
            mappedCombo = mappedCombo + key;
        else
            mappedCombo = mappedCombo + key +"+";
    }

    if(hotkey_functions_.find(mappedCombo) == hotkey_functions_.end())
        hotkey_functions_[mappedCombo] = function;
    else
        ROS_ERROR("tried to add duplicating hotkey functions for combo: %s", keyCombo.c_str());
}

void HotkeyManager::printallHotkeyCombinations()
{
    std::map<std::string, callback>::iterator iter;
    for (iter = hotkey_functions_.begin(); iter != hotkey_functions_.end(); ++iter)
    {
        ROS_ERROR("Combo: %s", iter->first.c_str() );
    }
}




