/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
//@TODO_ADD_AUTHOR_INFO
#ifndef COMMS_NOTIFICATION_SYSTEM_H
#define COMMS_NOTIFICATION_SYSTEM_H

#include <ros/ros.h>
#include "vigir_ocs_msgs/OCSOverlayText.h"

//example use
//CommsNotificationSystem::Instance()->notifyCustom("your text",vigir_ocs_msgs::OCSOverlayText::TOPROW,vigir_ocs_msgs::OCSOverlayText::CENTERCOLUMN,.8f,.8f,.8f);

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
