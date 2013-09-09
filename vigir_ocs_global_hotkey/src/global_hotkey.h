#include <ros/ros.h>

#include <std_msgs/Int32.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace ocs_global_hotkey
{
    class GlobalHotkey
    {
      public:
        GlobalHotkey();
        ~GlobalHotkey();
        
        void onUpdate();

        void publishKeyPressed(int k);
        void publishKeyReleased(int k);

      private:
        ros::NodeHandle n_;
        
        ros::Publisher hotkey_pressed_pub_;
        ros::Publisher hotkey_released_pub_;

        Display *display_name_;
        
        bool keys[4096];
    };
}
