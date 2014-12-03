#include <ros/ros.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/X.h>
#include <X11/XKBlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace vigir_ocs
{
    class GlobalHotkey
    {
      public:
        GlobalHotkey();
        ~GlobalHotkey();
        
        void onUpdate();

        void calculateCursorPosition(int &x, int &y);

        void publishKeyPressed(int k, int x, int y);
        void publishKeyReleased(int k, int x, int y);

        bool isActive();

      private:
        ros::NodeHandle n_;
        
        ros::Publisher key_event_pub_;

        Display *display_name_;
        
        std::vector<int> old_keys_;
        bool registering_keystrokes_;
    };
}
