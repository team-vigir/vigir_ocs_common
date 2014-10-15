#ifndef NOTIFICATION_SYSTEM_H
#define NOTIFICATION_SYSTEM_H

#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"

//example use
//NotificationSystem::Instance()->notifyCustom("your text",flor_ocs_msgs::OCSOverlayText::TOPROW,flor_ocs_msgs::OCSOverlayText::CENTERCOLUMN,.8f,.8f,.8f);

/**
Current Proposed Notifications (* means implemented)

Passive(doesn't have to be seen, top center white text)

    insert Template*
    remove Template*
    footstep plan started*
    grasp locked
    start walking
    stop walking
    manipulation started (arm moves to match ghost)

    new pointcloud request*

    new camera image request(not for video/fps boost?)
    update head pitch

    robot mode change*

    changed interactive marker manipulation mode

Warning(Has to be seen,  center, yellow text?) doug hates red text?

    manipulation fail(cannot start due to collision?)
    robot mode change fail
    footstep fail

    pointcloud request fail(invalid params?)- cant currently be detected


Error	(center red text)
    emergency stop activated*
  **/

//Singleton class to store state of robot to be conveniently referenced outside base3dview
class NotificationSystem{
   public:
       static NotificationSystem* Instance();
       void notifyPassive(std::string text);
       void notifyWarning(std::string text);
       void notifyError(std::string text);
       void notifyCustom(std::string text, int row, int column, float r, float g, float b);
   private:
       NotificationSystem();  // Private so that it can  not be called
       NotificationSystem(NotificationSystem const&){};             // copy constructor is private
       NotificationSystem& operator=(NotificationSystem const&){};  // assignment operator is private

       static NotificationSystem* instance;       
       ros::NodeHandle nh;
       ros::Publisher notification_pub_;       
};

#endif
