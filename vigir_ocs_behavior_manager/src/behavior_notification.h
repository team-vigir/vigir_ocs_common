/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
#include <flexbe_msgs/BehaviorInputAction.h>
#include <flexbe_msgs/BehaviorInputActionGoal.h>

#include "vigir_ocs_msgs/OCSObjectSelection.h"
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
    void queueDeleteNotification();
    int getGoalId();

private:
    bool eventFilter(QObject* object,QEvent* event);
    void timerEvent(QTimerEvent *event);
    void setButtonStyle(QPushButton* btn);
    void objectSelectCB(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr msg);

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
