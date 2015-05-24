#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{    
    behavior_goal_sub_ = nh_.subscribe( "/vigir/ocs/behavior/send_goal", 5, &BehaviorRelay::receiveBehaviorGoalCB, this );
    behavior_confirm_sub_ = nh_.subscribe( "/vigir/ocs/behavior/confirm_goal", 5, &BehaviorRelay::receiveBehaviorResult, this );
    behavior_confirm_pub_  = nh_.advertise<flor_ocs_msgs::OCSBehaviorGoal>("/vigir/ocs/behavior/confirm_goal", 5, false);

    parent_ = parent;
    max_notifications_shown_ = 5;    
}


//callback to receive goal and create notification
void BehaviorRelay::receiveBehaviorGoalCB(const flor_ocs_msgs::OCSBehaviorGoalConstPtr msg)
{    
    //build appropriate string for ui
    QString action_text = QString::fromStdString(msg->action_text);

    //notify gui to build stuff
    createNotification(action_text, msg->id, msg->goal_type);

    if(behavior_notifications_.size() <= max_notifications_shown_)
        Q_EMIT updateUI();
}


//callback to receive confirmation of other goals on other instances of OCS, will redundantly notify itself
void BehaviorRelay::receiveBehaviorResult(const flor_ocs_msgs::OCSBehaviorGoalConstPtr msg)
{
    for(int i = 0;i<behavior_notifications_.size();i++)
    {
        BehaviorNotification * notification = behavior_notifications_[i];
        //goal matches any of our given goals?
        if(notification->getGoalId() == msg->id)
        {
            ROS_ERROR("queing deletion %s" , boost::asio::ip::host_name());
            //set goal to be deleted, as it has been confirmed
            notification->queueDeleteNotification();
            cleanNotifications(); //remove notification to be deleted
            Q_EMIT updateUI();
            return;
        }
    }
}


//create notification and add to vector to be represented in main view
void BehaviorRelay::createNotification(QString action_text, int goal_id, int goal_type)
{
    BehaviorNotification* notification = new BehaviorNotification(NULL, action_text, goal_id, goal_type);
    //setup button on notification
    connect(notification, SIGNAL(sendConfirmation(QString, int)), this, SLOT(reportConfirmation(QString, int)));
    connect(notification, SIGNAL(sendAbort(QString, int)), this, SLOT(reportAbort(QString, int)));
    behavior_notifications_.push_back(notification);
}


void BehaviorRelay::reportConfirmation(QString action_text, int id)
{    
    //publish to tell views that this goal is obselete, tells manager to confirm and grab data
    flor_ocs_msgs::OCSBehaviorGoal msg;
    msg.action_text = qPrintable(action_text); // convert to c_str
    msg.id = id;
    msg.result = true;
    msg.host = boost::asio::ip::host_name();
    behavior_confirm_pub_.publish(msg);
}


void BehaviorRelay::reportAbort(QString action_text, int id)
{
    //publish to tell views that this goal is obselete, tells manager to abort
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
            ROS_ERROR("deleting before");
            //free notification
            delete behavior_notifications_[i];
            ROS_ERROR("deleting after");
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
