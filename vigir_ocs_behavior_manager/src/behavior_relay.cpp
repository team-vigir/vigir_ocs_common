#include "behavior_relay.h"


BehaviorRelay::BehaviorRelay(QWidget* parent)
{
   // edit_step_as = SimpleActionServer<msgs::EditStepAction>::create(nh, "edit_step", boost::bind(&GlobalFootstepPlannerNode::editStepAction, this, boost::ref(edit_step_as)), true);
    behavior_server_ = new BehaviorServer(nh_,"/vigir/ocs/behavior_relay_ui",false);
    behavior_server_ ->registerGoalCallback(boost::bind(&BehaviorRelay::processBehaviorGoalCB,this,behavior_server_));
    behavior_server_->start();
            //BehaviorServer::create(nh_, "/vigir/ocs/behavior_relay_ui", boost::bind(&BehaviorRelay::processBehaviorGoalCB,this,boost::ref(behavior_server_)), true);
    parent_ = parent;
    max_notifications_shown = 3;
    //ui test only
//    BehaviorNotification* notification = new BehaviorNotification(parent_);
//    notification->setActionText("action_text 1");
//    BehaviorNotification* notification2 = new BehaviorNotification(parent_);
//    notification2->setActionText("action_text 2");

//    behavior_notifications_.push_back(notification);
//    behavior_notifications_.push_back(notification2);
//    Q_EMIT updateUI();
}

void BehaviorRelay::processBehaviorGoalCB(BehaviorServer* server)
{
    ROS_ERROR("got a goal!");
    const vigir_be_input::BehaviorInputGoalConstPtr& goal(server->acceptNewGoal());

    //build appropriate string for ui
    QString action_text = QString::fromStdString(goal->msg);

    BehaviorNotification* notification = new BehaviorNotification(this);
    connect(notification,SIGNAL(sendConfirmation(QString)),this,SLOT(reportConfirmation(QString)));
    notification->setActionText(action_text);

    behavior_notifications_.push_back(notification);

    if(behavior_notifications_.size() < max_notifications_shown + 1)
        Q_EMIT updateUI();


}
void BehaviorRelay::reportConfirmation(QString action_text)
{
    vigir_be_input::BehaviorInputActionResult result;
    result.result.result_code = vigir_be_input::BehaviorInputResult::RESULT_OK;
    //result.status = result.status.SUCCEEDED; // even need status?
    behavior_server_->setSucceeded(result.result, qPrintable(action_text));

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
