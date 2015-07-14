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
#include "global_hotkey.h"

// taken from http://stackoverflow.com/questions/10157826/xkb-how-to-convert-a-keycode-to-keysym
KeySym KeyCodeToKeySym(Display * display, KeyCode keycode, unsigned int event_mask)
{
    KeySym keysym = NoSymbol;

    //Get the map
    XkbDescPtr keyboard_map = XkbGetMap(display, XkbAllClientInfoMask, XkbUseCoreKbd);
    if (keyboard_map)
    {
        //What is diff between XkbKeyGroupInfo and XkbKeyNumGroups?
        unsigned char info = XkbKeyGroupInfo(keyboard_map, keycode);
        unsigned int num_groups = XkbKeyNumGroups(keyboard_map, keycode);

        //Get the group
        unsigned int group = 0x00;
        switch (XkbOutOfRangeGroupAction(info))
        {
            case XkbRedirectIntoRange:
                /* If the RedirectIntoRange flag is set, the four least significant
                 * bits of the groups wrap control specify the index of a group to
                 * which all illegal groups correspond. If the specified group is
                 * also out of range, all illegal groups map to Group1.
                 */
                group = XkbOutOfRangeGroupInfo(info);
                if (group >= num_groups)
                    group = 0;
            break;

            case XkbClampIntoRange:
                /* If the ClampIntoRange flag is set, out-of-range groups correspond
                 * to the nearest legal group. Effective groups larger than the
                 * highest supported group are mapped to the highest supported group;
                 * effective groups less than Group1 are mapped to Group1 . For
                 * example, a key with two groups of symbols uses Group2 type and
                 * symbols if the global effective group is either Group3 or Group4.
                 */
                group = num_groups - 1;
            break;

            case XkbWrapIntoRange:
                /* If neither flag is set, group is wrapped into range using integer
                 * modulus. For example, a key with two groups of symbols for which
                 * groups wrap uses Group1 symbols if the global effective group is
                 * Group3 or Group2 symbols if the global effective group is Group4.
                 */
            default:
                if (num_groups != 0)
                    group %= num_groups;
            break;
        }

        XkbKeyTypePtr key_type = XkbKeyKeyType(keyboard_map, keycode, group);
        unsigned int active_mods = event_mask & key_type->mods.mask;

        int i, level = 0;
        for (i = 0; i < key_type->map_count; i++)
            if (key_type->map[i].active && key_type->map[i].mods.mask == active_mods)
                level = key_type->map[i].level;

        keysym = XkbKeySymEntry(keyboard_map, keycode, level, group);
        XkbFreeClientMap(keyboard_map, XkbAllClientInfoMask, true);
    }

    return keysym;
}

namespace vigir_ocs
{
GlobalHotkey::GlobalHotkey() :
    registering_keystrokes_(true)
{
    ros::NodeHandle nh_out(n_, "/flor/ocs");

    // create publishers for visualization
    key_event_pub_  = nh_out.advertise<vigir_ocs_msgs::OCSKeyEvent>( "key_event", 1, false );

    display_name_ = XOpenDisplay(NULL);
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
    bool state_changed = false;
    std::vector<int> new_keys;
	
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
                    new_keys.push_back(i*8+pos);
				}
				pos++; num /= 2;
			}

            //printf("%x ",(unsigned char)keys_return[i]);
		}
	}
    //printf("\n");

    int x,y;
    calculateCursorPosition(x,y);
	
    if(registering_keystrokes_)
    {
        unsigned int event_mask = 0;//ShiftMask | LockMask;

        // check if key was pressed/released before updating old_keys_ array
        for(int i = 0; i < new_keys.size(); i++)
        {
            KeySym keysym = KeyCodeToKeySym(display_name_, new_keys[i], event_mask);

            if(std::find(old_keys_.begin(), old_keys_.end(), new_keys[i]) == old_keys_.end())
            {
                printf("key pressed (%d,%s); cursor at %d, %d\n",new_keys[i],XKeysymToString(keysym),x,y);
                publishKeyPressed(new_keys[i],x,y);
                state_changed = true;
            }
        }
        for(int i = 0; i < old_keys_.size(); i++)
        {
            KeySym keysym = KeyCodeToKeySym(display_name_, old_keys_[i], event_mask);

            if(std::find(new_keys.begin(), new_keys.end(), old_keys_[i]) == new_keys.end())
            {
                printf("key released (%d,%s); cursor at %d, %d\n",old_keys_[i],XKeysymToString(keysym),x,y);
                publishKeyReleased(old_keys_[i],x,y);
                state_changed = true;
            }
        }
    }

    // save key states
    old_keys_ = new_keys;

    //printf("key states(%d): %d, %d, %d\n", state_changed, old_keys_[37], old_keys_[64], old_keys_[45]);

//    if(state_changed &&
//       std::find(old_keys_.begin(), old_keys_.end(), 37) != old_keys_.end() &&
//       std::find(old_keys_.begin(), old_keys_.end(), 64) != old_keys_.end() &&
//       std::find(old_keys_.begin(), old_keys_.end(), 45) != old_keys_.end()) // ctrl + alt + k
//        registering_keystrokes_ = !registering_keystrokes_;
}

void GlobalHotkey::publishKeyPressed(int k, int x, int y)
{
    vigir_ocs_msgs::OCSKeyEvent cmd;

    cmd.keycode = k;
    KeySym keysym = KeyCodeToKeySym(display_name_, k, 0);
    cmd.keysym = keysym;
    cmd.keystr = XKeysymToString(keysym);
    cmd.state = 1;
    cmd.cursor_x = x;
    cmd.cursor_y = y;

    // publish key event and cursor position information
    key_event_pub_.publish(cmd);
}

void GlobalHotkey::publishKeyReleased(int k, int x, int y)
{
    vigir_ocs_msgs::OCSKeyEvent cmd;

    cmd.keycode = k;
    KeySym keysym = KeyCodeToKeySym(display_name_, k, 0);
    cmd.keysym = keysym;
    cmd.keystr = XKeysymToString(keysym);
    cmd.state = 0;
    cmd.cursor_x = x;
    cmd.cursor_y = y;

    // publish key event and cursor position information
    key_event_pub_.publish(cmd);
}

}
