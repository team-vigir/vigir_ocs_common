#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{
    //cannot seem to get execute callback to compile with arguments being passed, alternative is to use goal callback and grab params with acceptNewGoal
    //behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",boost::bind(&BehaviorRelay::processBehaviorGoalCB,this,_1,_2),false);
    behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",false);
    //behavior_server_ ->registerGoalCallback(boost::bind(&BehaviorRelay::processBehaviorGoalCB,this));
    behavior_server_->start();

    parent_ = parent;
    max_notifications_shown_ = 3;

    //connect execute thread to gui thread
    connect(this,SIGNAL(signalCreateNotification(QString,BehaviorServer::GoalHandlePtr)),this,SLOT(createNotification(QString,BehaviorServer::GoalHandlePtr)));


    //createNotification("heres a notificatino",NULL);
    //Q_EMIT updateUI();
}

//this is being run on a seperate thread by the behavior server
void BehaviorRelay::processBehaviorGoalCB(vigir_be_input::BehaviorInputGoalConstPtr goal, BehaviorServer::GoalHandlePtr goal_handle)
{
   //BehaviorServer::GoalHandlePtr goal_handle = behavior_server_->acceptNewGoal();
   //const vigir_be_input::BehaviorInputGoalConstPtr& goal = goal_handle->getGoal();
   // ROS_ERROR("got a goal! %s", goal->msg.c_str());

    //build appropriate string for ui
    QString action_text = QString::fromStdString(goal->msg);

    //notify gui thread to build stuff, cant build in this thread
    Q_EMIT signalCreateNotification(action_text , goal_handle);

    boost::recursive_mutex::scoped_lock lock(lock_);
    lock.lock();

    if(behavior_notifications_.size() <= max_notifications_shown_)
        Q_EMIT updateUI();

    lock.unlock();
}

void BehaviorRelay::createNotification(QString action_text, const BehaviorServer::GoalHandlePtr goal_handle)
{
    BehaviorNotification* notification = new BehaviorNotification();
    connect(notification,SIGNAL(sendConfirmation(QString,BehaviorServer::GoalHandlePtr)),this,SLOT(reportConfirmation(QString,BehaviorServer::GoalHandlePtr)));
    connect(notification,SIGNAL(sendAbort(QString,BehaviorServer::GoalHandlePtr)),this,SLOT(reportAbort(QString,BehaviorServer::GoalHandlePtr)));
    notification->setActionText(action_text);
    notification->setGoal(goal_handle);

    behavior_notifications_.push_back(notification);
}

void BehaviorRelay::reportConfirmation(QString action_text,BehaviorServer::GoalHandlePtr goal_handle)
{
    latest_behavior_action_text_ = action_text;

    vigir_be_input::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_OK;
    behavior_server_->setSucceeded(result.result, qPrintable(action_text), goal_handle);

    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification
}

void BehaviorRelay::reportAbort(QString action_text, BehaviorServer::GoalHandlePtr goal_handle)
{
    vigir_be_input::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_ABORTED;
    behavior_server_->setAborted(result.result, qPrintable(action_text),goal_handle);

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
