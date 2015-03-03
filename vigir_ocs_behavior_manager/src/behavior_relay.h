#ifndef BEHAVIOR_RELAY_H
#define BEHAVIOR_RELAY_H

#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"
#include "behavior_notification.h"

#include <actionlib/server/simple_action_server.h>
#include <vigir_be_input/BehaviorInputAction.h>

/**
 * Subscribe to different topics in order to process whether a required action has been processed.
 *
 * Controls positioning of individual notifications
 *
 */

typedef actionlib::SimpleActionServer<vigir_be_input::BehaviorInputAction> BehaviorServer;

class BehaviorRelay: public QWidget
{
    Q_OBJECT

   public:
       explicit BehaviorRelay(QWidget *parent = 0);
       std::vector<BehaviorNotification*> getNotifications();

   private:
       BehaviorRelay(BehaviorRelay const&){};             // copy constructor is private
       BehaviorRelay& operator=(BehaviorRelay const&){};  // assignment operator is private
       void processBehaviorGoalCB(const vigir_be_input::BehaviorInputAction &goal);
       void cleanNotifications();

       QWidget* parent_;
       ros::NodeHandle nh_;
       std::vector<BehaviorNotification*> behavior_notifications_;
       BehaviorServer *behavior_server_;
Q_SIGNALS:
       void updateUI();
};

#endif //BEHAVIOR_RELAY_H
