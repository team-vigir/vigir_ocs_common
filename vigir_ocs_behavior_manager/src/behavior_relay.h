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
//@TODO_ADD_AUTHOR_INFO
#ifndef BEHAVIOR_RELAY_H
#define BEHAVIOR_RELAY_H

#include <stdlib.h>
#include <ros/ros.h>
#include "vigir_ocs_msgs/OCSOverlayText.h"
#include "vigir_ocs_msgs/OCSBehaviorGoal.h"
#include "behavior_notification.h"
#include <boost/asio/ip/host_name.hpp>
#include <vigir_be_msgs/BehaviorInputAction.h>


/**
 * Subscribe to different topics in order to process whether a required action has been processed.
 *
 * Controls positioning of individual notifications
 *
 */


class BehaviorRelay: public QWidget
{
    Q_OBJECT

   public:
       explicit BehaviorRelay(QWidget *parent = 0);
       std::vector<BehaviorNotification*> getNotifications();
       int getMaxNotificationsShown(){return max_notifications_shown_;}

   private:
       BehaviorRelay(BehaviorRelay const&){};             // copy constructor is private
       BehaviorRelay& operator=(BehaviorRelay const&){};  // assignment operator is private
       void receiveBehaviorGoalCB(const vigir_ocs_msgs::OCSBehaviorGoalConstPtr msg);
       void receiveBehaviorResult(const vigir_ocs_msgs::OCSBehaviorGoalConstPtr msg);
       void cleanNotifications();      

       QWidget* parent_;
       ros::NodeHandle nh_;
       std::vector<BehaviorNotification*> behavior_notifications_;
       int max_notifications_shown_;       
       QString latest_behavior_action_text_;       

       ros::Subscriber behavior_goal_sub_;
       ros::Subscriber behavior_confirm_sub_;
       ros::Publisher  behavior_confirm_pub_;

Q_SIGNALS:
       void updateUI();
       void signalCreateNotification(QString);

public Q_SLOTS:
    void reportConfirmation(QString, int id);
    void reportAbort(QString action_text, int id);
    void createNotification(QString action_text, int id, int goal_type);
};



#endif //BEHAVIOR_RELAY_H
