#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{    
    behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",boost::bind(&BehaviorRelay::processBehaviorGoalCB,this,_1,_2),false);
    behavior_server_->start();

    parent_ = parent;
    max_notifications_shown_ = 3;

    qRegisterMetaType<BehaviorServer::GoalHandle>("GoalHandle");
    //connect execute thread to gui thread
    connect(this,SIGNAL(signalCreateNotification(QString,BehaviorServer::GoalHandle)),this,SLOT(createNotification(QString,BehaviorServer::GoalHandle)));

}

//this is being run on a seperate thread by the complex behavior server
void BehaviorRelay::processBehaviorGoalCB(vigir_be_msgs::BehaviorInputGoalConstPtr goal, BehaviorServer::GoalHandle goal_handle)
{    
    //build appropriate string for ui
    QString action_text = QString::fromStdString(goal->msg);

    //ui is shared among all threads, must protect
    boost::recursive_mutex::scoped_lock lock(lock_);

    //notify gui thread to build stuff, cant build in this thread
    Q_EMIT signalCreateNotification(action_text , goal_handle);

    if(behavior_notifications_.size() <= max_notifications_shown_)
        Q_EMIT updateUI();

}

//create notification and add to vector to be represented in main view
void BehaviorRelay::createNotification(QString action_text, const BehaviorServer::GoalHandle goal_handle)
{
    BehaviorNotification* notification = new BehaviorNotification();
    connect(notification,SIGNAL(sendConfirmation(QString,BehaviorServer::GoalHandle)),this,SLOT(reportConfirmation(QString,BehaviorServer::GoalHandle)));
    connect(notification,SIGNAL(sendAbort(QString,BehaviorServer::GoalHandle)),this,SLOT(reportAbort(QString,BehaviorServer::GoalHandle)));
    notification->setActionText(action_text);
    notification->setGoal(goal_handle);

    behavior_notifications_.push_back(notification);
}

void BehaviorRelay::reportConfirmation(QString action_text,BehaviorServer::GoalHandle goal_handle)
{    
    //build result
    vigir_be_msgs::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_msgs::BehaviorInputResult::RESULT_OK;
    //notify server to set goal to succeed
    behavior_server_->setSucceeded(result.result, qPrintable(action_text), goal_handle);

    cleanNotifications();    
    Q_EMIT updateUI(); //remove and enqueue new notification    
}

void BehaviorRelay::reportAbort(QString action_text, BehaviorServer::GoalHandle goal_handle)
{
    vigir_be_msgs::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_msgs::BehaviorInputResult::RESULT_ABORTED;
    behavior_server_->setAborted(result.result, qPrintable(action_text),goal_handle);

    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification
}


void BehaviorRelay::cleanNotifications()
{   
    //store valid notifications
    std::vector<BehaviorNotification*> tmp;

    for(int i=0;i<behavior_notifications_.size();i++)
    {
        //store valid notifications to save
        // still hasn't been confirmed, so its valid
        if(!behavior_notifications_[i]->getConfirmed())
        {            
            tmp.push_back(behavior_notifications_[i]);
        }
        else
        {
            //free notification
            delete behavior_notifications_[i];
        }
    }
    //calls only destructor of pointers
    behavior_notifications_.clear();    
    //vector now contains only valid notifications
    for(int i=0;i<tmp.size();i++)
        behavior_notifications_.push_back(tmp[i]);

}

std::vector<BehaviorNotification*> BehaviorRelay::getNotifications()
{
    return behavior_notifications_;
}
