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
@TODO_ADD_AUTHOR_INFO
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



