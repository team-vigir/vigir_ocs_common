#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{       
   //behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",boost::bind(&BehaviorRelay::processBehaviorGoalCB,this,_1),false);
    behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",NULL,false);
    behavior_server_ ->registerGoalCallback(boost::bind(&BehaviorRelay::processBehaviorGoalCB,this, behavior_server_));
    behavior_server_->start();

    parent_ = parent;
    max_notifications_shown = 3;

    connect(this,SIGNAL(signalCreateNotification(QString)),this,SLOT(createNotification(QString)));

    latest_behavior_action_text_ = "";
}

//this is being run on a seperate thread by the behavior server
void BehaviorRelay::processBehaviorGoalCB(BehaviorServer* server/*, const vigir_be_input::BehaviorInputGoalConstPtr goal*/)
{   
    const vigir_be_input::BehaviorInputGoalConstPtr& goal(server->acceptNewGoal());
   // ROS_ERROR("got a goal! %s", goal->msg.c_str());

    //build appropriate string for ui
    QString action_text = QString::fromStdString(goal->msg);

    //notify gui thread to build stuff, cant build in this thread
    Q_EMIT signalCreateNotification(action_text);

    if(behavior_notifications_.size() < max_notifications_shown + 1)
        Q_EMIT updateUI();

//    while(action_text != latest_behavior_action_text_)
//    {
//        //do nothing
//    }

//    vigir_be_input::BehaviorInputActionResult result;
//    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_OK;
//    behavior_server_->setSucceeded(result.result, qPrintable(action_text));

//    cleanNotifications();
//    Q_EMIT updateUI(); //remove and enqueue new notification

}

void BehaviorRelay::createNotification(QString action_text)
{
    BehaviorNotification* notification = new BehaviorNotification();
    connect(notification,SIGNAL(sendConfirmation(QString)),this,SLOT(reportConfirmation(QString)));
    connect(notification,SIGNAL(sendAbort(QString)),this,SLOT(reportAbort(QString)));
    notification->setActionText(action_text);

    behavior_notifications_.push_back(notification);
}

void BehaviorRelay::reportConfirmation(QString action_text)
{ 
    latest_behavior_action_text_ = action_text;

    vigir_be_input::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_OK;
    behavior_server_->setSucceeded(result.result, qPrintable(action_text));

    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification
}

void BehaviorRelay::reportAbort(QString action_text)
{
    vigir_be_input::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_ABORTED;    
    behavior_server_->setAborted(result.result, qPrintable(action_text));

    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification
}


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
        else // still hasn't been confirmed, so its valid
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
