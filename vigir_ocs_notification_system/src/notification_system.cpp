
#include "notification_system.h"

// Allocating and initializing GlobalClass's
// static data member.  The pointer is being
// allocated - not the object inself
NotificationSystem* NotificationSystem::instance = 0;

NotificationSystem::NotificationSystem()
{    
    //create publisher for notifications
    notification_pub_ = nh.advertise<flor_ocs_msgs::OCSOverlayText>("/flor/ocs/overlay_text",5,false);
}

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
NotificationSystem* NotificationSystem::Instance()
{
   if (!instance)   // Only allow one instance of class to be generated.
       instance = new NotificationSystem();

   return instance;
}

//publishes
void NotificationSystem::notify(std::string text)
{
    flor_ocs_msgs::OCSOverlayText msg;
    msg.text = text;
    //messages probably shouldn't be multiple lines?
    msg.height = 20;
    msg.width = text.length() * 7;
    msg.font = "DejaVu Sans Mono";
    msg.top = 5;
    msg.left = 70;
    msg.line_width = 2;
    msg.text_size = 8;
    msg.fg_color.r = .8;
    msg.fg_color.g = .8;
    msg.fg_color.b = .8;
    msg.fg_color.a = .9;
    msg.bg_color.a = .5f;
    msg.action = flor_ocs_msgs::OCSOverlayText::ADD;
    notification_pub_.publish(msg);
}

void NotificationSystem::hide()
{
    flor_ocs_msgs::OCSOverlayText msg;
    // could just fade alpha instead? QPropertAnimation
    msg.action = flor_ocs_msgs::OCSOverlayText::DELETE;
    notification_pub_.publish(msg);
}

/*
  overload timerEvent and send hide.. would be nice to fade out and fade in  .. requires qt in singleton (maybe put rviz display in here?)
  */

