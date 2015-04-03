#ifndef BEHAVIOR_RELAY_H
#define BEHAVIOR_RELAY_H

#include <stdlib.h>
#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"
#include "behavior_notification.h"

#include <actionlib/server/simple_action_server.h>
#include <vigir_be_input/BehaviorInputAction.h>
#include "complex_action_server.h"

/**
 * Subscribe to different topics in order to process whether a required action has been processed.
 *
 * Controls positioning of individual notifications
 *
 */

typedef actionlib::SimpleActionServer<vigir_be_input::BehaviorInputAction> BehaviorServer;
//typedef actionlib::ComplexActionServer<vigir_be_input::BehaviorInputAction> BehaviorServer;

class BehaviorRelay: public QWidget
{
    Q_OBJECT

   public:
       explicit BehaviorRelay(QWidget *parent = 0);
       std::vector<BehaviorNotification*> getNotifications();
       int getMaxNotificationsShown(){return max_notifications_shown;}

   private:
       BehaviorRelay(BehaviorRelay const&){};             // copy constructor is private
       BehaviorRelay& operator=(BehaviorRelay const&){};  // assignment operator is private
       void processBehaviorGoalCB(BehaviorServer *server/*,const vigir_be_input::BehaviorInputGoalConstPtr goal*/);
       void cleanNotifications();

       QWidget* parent_;
       ros::NodeHandle nh_;
       std::vector<BehaviorNotification*> behavior_notifications_;
       int max_notifications_shown;
       BehaviorServer* behavior_server_;
       QString latest_behavior_action_text_;

Q_SIGNALS:
       void updateUI();
       void signalCreateNotification(QString);

public Q_SLOTS:
    void reportConfirmation(QString);
    void reportAbort(QString action_text);
    void createNotification(QString);
};



#endif //BEHAVIOR_RELAY_H
