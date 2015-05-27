#ifndef COMMS_NOTIFICATION_SYSTEM_H
#define COMMS_NOTIFICATION_SYSTEM_H

#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"

//example use
//CommsNotificationSystem::Instance()->notifyCustom("your text",flor_ocs_msgs::OCSOverlayText::TOPROW,flor_ocs_msgs::OCSOverlayText::CENTERCOLUMN,.8f,.8f,.8f);

/**
  Places notification in top,center and center,center(errors)

Current Proposed Notifications (* means implemented)

Passive(doesn't have to be seen, top center white text)

    insert Template*
    remove Template*
    footstep plan started*
    grasp locked
    start walking
    stop walking
    manipulation started (arm moves to match ghost) moveit start?

    new pointcloud request*

    new camera image request(not for video/fps boost?)
    update head pitch

    robot mode change*

    changed interactive marker manipulation mode*

Warning(Has to be seen,  center, yellow text?) doug hates red text?

    manipulation fail(cannot start due to collision?)
    robot mode change fail
    footstep fail

    pointcloud request fail(invalid params?)- cant currently be detected


Error	(center red text)
    emergency stop activated*
  **/

class CommsNotificationSystem{
   public:
       static CommsNotificationSystem* Instance();
       void notifyPassive(std::string text);

   private:
       CommsNotificationSystem();  // Private so that it can  not be called
       CommsNotificationSystem(CommsNotificationSystem const&){};             // copy constructor is private
       CommsNotificationSystem& operator=(CommsNotificationSystem const&){};  // assignment operator is private

       static CommsNotificationSystem* instance;
       ros::NodeHandle nh;
       ros::Publisher notification_pub_;       
};

#endif
