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
#include <QFrame>
#include "complex_action_server.h"

//typedef typename ActionServer<ActionSpec>::GoalHandlePtr GoalHandlePtr;

namespace Ui
{
    class BehaviorNotification; //window name
}

typedef actionlib::ComplexActionServer<vigir_be_msgs::BehaviorInputAction> BehaviorServer;

class BehaviorNotification : public QWidget
{
    Q_OBJECT

public:
    ~BehaviorNotification();
    explicit BehaviorNotification(QWidget *parent = 0);
    bool getConfirmed() { return confirmed_; }
    void setActionText(QString);
    void setGoal(BehaviorServer::GoalHandle );
    void setPoint(QPoint point){main_view_point_ = point;}

private:
    bool eventFilter(QObject* object,QEvent* event);
    void timerEvent(QTimerEvent *event);
    void setButtonStyle(QPushButton* btn);

    Ui::BehaviorNotification *ui;
    bool confirmed_;
    BehaviorServer::GoalHandle goal_;
    QPropertyAnimation* confirm_fadein_;
    QBasicTimer timer;
    QPoint main_view_point_;


public Q_SLOTS:
    void confirm();
    void abort();


private Q_SLOTS:


Q_SIGNALS:
    void sendConfirmation(QString,const BehaviorServer::GoalHandle);
    void sendAbort(QString,const BehaviorServer::GoalHandle);
};


#endif // BEHAVIOR_NOTIFICATION_H
