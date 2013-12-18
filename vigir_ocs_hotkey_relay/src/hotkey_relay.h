#include <ros/ros.h>

#include <flor_ocs_msgs/OCSHotkeyRelay.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace vigir_ocs
{
    class HotkeyRelay
    {
      public:
        HotkeyRelay();
        ~HotkeyRelay();

        void processKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& msg);

      private:
        ros::NodeHandle n_;
        
        ros::Publisher key_event_pub_;

        ros::Subscriber key_event_sub_;

        std::vector<int> keys_pressed_list_;

        bool old_keys_[4096];
    };
}
