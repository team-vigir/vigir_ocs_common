#ifndef NOTIFICATION_SYSTEM_H
#define NOTIFICATION_SYSTEM_H

#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"

//Singleton class to store state of robot to be conveniently referenced outside base3dview
class NotificationSystem{
   public:
       static NotificationSystem* Instance();
       void notify(std::string text, int row, int column, float r, float g, float b);
 
   private:
       NotificationSystem();  // Private so that it can  not be called
       NotificationSystem(NotificationSystem const&){};             // copy constructor is private
       NotificationSystem& operator=(NotificationSystem const&){};  // assignment operator is private

       static NotificationSystem* instance;       
       ros::NodeHandle nh;
       ros::Publisher notification_pub_;
};

#endif
