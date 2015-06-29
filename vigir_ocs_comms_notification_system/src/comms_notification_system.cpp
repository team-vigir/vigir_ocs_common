#include "comms_notification_system.h"

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

// Allocating and initializing GlobalClass's
// static data member.  The pointer is being
// allocated - not the object inself
CommsNotificationSystem* CommsNotificationSystem::instance = 0;

CommsNotificationSystem::CommsNotificationSystem()
{    
    //create publisher for notifications
    notification_pub_ = nh.advertise<vigir_ocs_msgs::OCSOverlayText>("flor/ocs/comms_status_overlay_text",5,false);
}

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
CommsNotificationSystem* CommsNotificationSystem::Instance()
{
   if (!instance)   // Only allow one instance of class to be generated.
       instance = new CommsNotificationSystem();

   return instance;
}

//publishes to show white text in top center
void CommsNotificationSystem::notifyPassive(std::string text)
{
    vigir_ocs_msgs::OCSOverlayText msg;
    msg.text = text;
    //messages probably shouldn't be multiple lines?
    msg.height = 18;
    msg.width = text.length() * 8;
    msg.font = "DejaVu Sans Mono";
    msg.line_width = 2;
    msg.text_size = 10;
    msg.fg_color.r = .8f;
    msg.fg_color.g = .8f;
    msg.fg_color.b = .8f;
    msg.fg_color.a = .9f;
    msg.bg_color.a = .5f;
    msg.action = vigir_ocs_msgs::OCSOverlayText::ADD;
    msg.fadeIn = 1.0f;
    //msg.fadeOut = 1.0f;
    msg.upTime = 2.0f;
    msg.row = vigir_ocs_msgs::OCSOverlayText::TOPROW;
    msg.column = vigir_ocs_msgs::OCSOverlayText::LEFTCOLUMN;
    notification_pub_.publish(msg);

}



