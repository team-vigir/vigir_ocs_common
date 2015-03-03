#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{
    //BehaviorServer* behavior_server_ = new BehaviorServer(nh_, "/ocs/behavior_relay_ui", boost::bind(&BehaviorRelay::processBehaviorGoalCB, _1, behavior_server_), true);
    parent_ = parent;
    //ui test only
    BehaviorNotification* notification = new BehaviorNotification(parent_);
    notification->setActionText("action_text");
    behavior_notifications_.push_back(notification);
    Q_EMIT updateUI();
}

void BehaviorRelay::processBehaviorGoalCB(const vigir_be_input::BehaviorInputAction& goal)
{
    //build appropriate string for ui
    QString action_text = QString::fromStdString(goal.action_goal.goal.msg);

    BehaviorNotification* notification = new BehaviorNotification(parent_);
    notification->setActionText(action_text);

    behavior_notifications_.push_back(notification);

    if(behavior_notifications_.size() < 3)
        Q_EMIT updateUI();

    //Blocking action.. confirm has been clicked?
    while(!notification->getConfirmed())
    {
        //nothing just block
    }
    behavior_server_->setSucceeded();
    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification
}


//TODO:: MAKE SURE THIS IS SAFE
void BehaviorRelay::cleanNotifications()
{
    //store valid notifications
    std::vector<BehaviorNotification*> tmp;

    //delete invalid notifications
    for(int i=0;i<behavior_notifications_.size();i++)
    {
        if(behavior_notifications_[i]->getConfirmed())
        {
            delete behavior_notifications_[i];
        }
        else // still hasn't been confirmed, valid
            tmp.push_back(behavior_notifications_[i]);
    }
    //clear old vector
    behavior_notifications_.clear();
    //vector now contains only valid notifications
    behavior_notifications_ = tmp;
}

std::vector<BehaviorNotification*> BehaviorRelay::getNotifications()
{
    return behavior_notifications_;
}
