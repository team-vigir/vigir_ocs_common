
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

/*
  change to ber horizontally and vertically aligned top center and bottom
  use update function in place of timer   fade in, uptime, fade out . directly modify alpha in color
  overlay_text_display needs to get render_panel to be aligned correctly

  */

//publishes a standard message to have a notification appear and fade out in 3d view
void NotificationSystem::notify(std::string text, int row, int column, float r, float g, float b)
{
    flor_ocs_msgs::OCSOverlayText msg;
    msg.text = text;
    //messages probably shouldn't be multiple lines?
    msg.height = 18;
    msg.width = text.length() * 8;
    msg.font = "DejaVu Sans Mono";
    msg.line_width = 2;
    msg.text_size = 10;
    msg.fg_color.r = r;
    msg.fg_color.g = g;
    msg.fg_color.b = b;
    msg.fg_color.a = .9f;
    msg.bg_color.a = .5f;
    msg.action = flor_ocs_msgs::OCSOverlayText::ADD;
    msg.fadeIn = 1.0f;
    msg.fadeOut = 1.0f;
    msg.upTime = 2.0f;
    msg.row = row;
    msg.column = column;
    notification_pub_.publish(msg);
}



