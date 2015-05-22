#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{    
    behavior_goal_sub_ = nh_.subscribe<flor_ocs_msgs::OCSBehaviorGoal>( "/vigir/ocs/behavior/send_goal", 5, &BehaviorRelay::receiveBehaviorGoalCB, this );
    behavior_confirm_sub_ = nh_.subscribe<flor_ocs_msgs::OCSBehaviorGoal>( "/vigir/ocs/behavior/confirm_goal", 5, &BehaviorRelay::receiveBehaviorResult, this );
    behavior_confirm_pub_  = nh_.advertise<flor_ocs_msgs::OCSBehaviorGoal>("/vigir/ocs/behavior/confirm_goal",1,false);

    parent_ = parent;
    max_notifications_shown_ = 5;

    qRegisterMetaType<BehaviorServer::GoalHandle>("GoalHandle");
}


//callback to receive goal and create notification
void BehaviorRelay::receiveBehaviorGoalCB(const flor_ocs_msgs::OCSBehaviorGoalConstPtr& msg)
{    
    //build appropriate string for ui
    QString action_text = QString::fromStdString(msg->action_text);

    //notify gui to build stuff
    createNotification(action_text, msg->id, msg->goal_type);

    if(behavior_notifications_.size() <= max_notifications_shown_)
        Q_EMIT updateUI();
}

//callback to receive confirmation of other goals on other instances of OCS, will redundantly notify itself
void BehaviorRelay::receiveBehaviorResult(const flor_ocs_msgs::OCSBehaviorGoalConstPtr& msg)
{
    for(int i =0;i<behavior_notifications_.size();i++)
    {
        BehaviorNotification * notification = behavior_notifications_[i];
        //goal matches any of our given goals?
        if(notification->getGoalId() == msg->id)
        {
            //delete goal, as it has been confirmed
            notification->deleteNotification();
        }
    }
    cleanNotifications(); //remove notification to be deleted
}


//create notification and add to vector to be represented in main view
void BehaviorRelay::createNotification(QString action_text, int goal_id, int goal_type)
{
    BehaviorNotification* notification = new BehaviorNotification(NULL,action_text, goal_id, goal_type);
    connect(notification,SIGNAL(sendConfirmation(QString,int)),this,SLOT(reportConfirmation(QString,int)));
    connect(notification,SIGNAL(sendAbort(QString,int)),this,SLOT(reportAbort(QString,int)));    
    behavior_notifications_.push_back(notification);
}

void BehaviorRelay::reportConfirmation(QString action_text, int id)//, BehaviorServer::GoalHandle goal_handle)
{    
    cleanNotifications();    
    Q_EMIT updateUI(); //remove and enqueue new notification    

    //publish to tell other views that this goal is obselete
    flor_ocs_msgs::OCSBehaviorGoal msg;
    msg.action_text = qPrintable(action_text); // convert to c_str
    msg.id = id;
    msg.result = true;
    msg.host = boost::asio::ip::host_name();
    behavior_confirm_pub_.publish(msg);
}

void BehaviorRelay::reportAbort(QString action_text, int id)//, BehaviorServer::GoalHandle goal_handle)
{
    cleanNotifications();
    Q_EMIT updateUI(); //remove and enqueue new notification

    //publish to tell other views that this goal is obselete
    flor_ocs_msgs::OCSBehaviorGoal msg;
    msg.action_text = qPrintable(action_text); // convert to c_str
    msg.id = id;
    msg.result = false;
    msg.host = boost::asio::ip::host_name();
    behavior_confirm_pub_.publish(msg);
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
