#ifndef BEHAVIOR_NOTIFICATION_H
#define BEHAVIOR_NOTIFICATION_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <QtCore>
#include <QMessageBox>
#include <QMouseEvent>
#include <QBasicTimer>
#include <QPropertyAnimation>
#include <vigir_be_msgs/BehaviorInputAction.h>
#include <vigir_be_msgs/BehaviorInputActionGoal.h>

#include "flor_ocs_msgs/OCSObjectSelection.h"
#include <QFrame>
#include "complex_action_server.h"
#include <boost/asio/ip/host_name.hpp>


namespace Ui
{
    class BehaviorNotification; //window name
}

class BehaviorNotification : public QWidget
{
    Q_OBJECT

public:
    ~BehaviorNotification();
    explicit BehaviorNotification(QWidget *parent = 0, QString action_text="", int goal_id=-1, int goal_type=-1);
    bool getConfirmed();
    void setActionText(QString);    
    void setPoint(QPoint point);
    void deleteNotification();   
    int getGoalId();

private:
    bool eventFilter(QObject* object,QEvent* event);
    void timerEvent(QTimerEvent *event);
    void setButtonStyle(QPushButton* btn);
    void objectSelectCB(const flor_ocs_msgs::OCSObjectSelection::ConstPtr msg);

    Ui::BehaviorNotification *ui;
    bool confirmed_;    
    int goal_id_;
    int goal_type_;
    QPropertyAnimation* confirm_fadein_;
    QBasicTimer timer;
    QPoint main_view_point_;
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;



public Q_SLOTS:
    void confirm();
    void abort();


private Q_SLOTS:


Q_SIGNALS:
    void sendConfirmation(QString,int);
    void sendAbort(QString,int);
};


#endif // BEHAVIOR_NOTIFICATION_H
