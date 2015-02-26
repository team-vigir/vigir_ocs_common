#ifndef BEHAVIOR_MANAGER_H
#define BEHAVIOR_MANAGER_H

#include <ros/ros.h>
#include "flor_ocs_msgs/OCSOverlayText.h"
#include "behavior_notification.h"

/**
 * Subscribe to different topics in order to process whether a required action has been processed.
 *
 * Controls positioning of individual notifications
 *
 */
class BehaviorManager: public QWidget
{
    Q_OBJECT
   public:
       BehaviorManager();
       static BehaviorManager* Instance();
       void createNotification();

   private:
       BehaviorManager(BehaviorManager const&){};             // copy constructor is private
       BehaviorManager& operator=(BehaviorManager const&){};  // assignment operator is private

       int x;
       int y;
       std::vector<BehaviorNotification*> behavior_notifications;
       static BehaviorManager* instance;       
       ros::NodeHandle nh;
       ros::Subscriber behavior_sub_;
};

#endif //BEHAVIOR_MANAGER_H
