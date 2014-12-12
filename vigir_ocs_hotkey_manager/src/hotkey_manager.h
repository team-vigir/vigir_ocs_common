/*
 * Hotkey Manager class definition.
 *
 * Author: Brian Wright.
 */

#ifndef HOTKEY_MANAGER_H
#define HOTKEY_MANAGER_H

#include <ros/ros.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>

/*
 * Note: cannot have a hotkey correspond to two different actions within the same main window. There will be a hotkey manager for map,main,and camera.
 *
 * naming convention for keyCombo = key+key   so ctrl and q is  ctrl+q
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

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event);
    void callHotkeyFunction(std::string keyCombo);

    std::vector<std::string> keys_pressed_list_;
    ros::NodeHandle nh_;
    ros::Subscriber key_event_sub_;


    //cant put boost function pointers directly as map type
    typedef boost::function< void () > callback;

    std::map<std::string,callback> hotkey_functions_;

};

#endif // HOTKEY_MANAGER_H
