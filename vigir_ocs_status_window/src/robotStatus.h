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
#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <QMainWindow>
#include <QWidget>
#include <QPoint>
#include <QPainter>
#include <QTableWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QFile>
#include <QComboBox>
#include <QtGui>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <vigir_ocs_msgs/OCSRobotStatus.h>
#include <vigir_ocs_msgs/OCSKeyEvent.h>

class completeRow
{
public:
    QTableWidgetItem* text;
    QTableWidgetItem* time;
    QTableWidgetItem* priority;

};



class robotStatus : public QWidget
{
    Q_OBJECT

public:
    explicit robotStatus(QWidget *parent = 0);
    ~robotStatus();
    //void msgRecieved( const )
    int getNumUnread();
    int getNumError();
    int getNumWarn();
    QString timeFromMsg(const ros::Time msg);
    void receivedMessage(const vigir_ocs_msgs::OCSRobotStatus::ConstPtr msg);
    void clearCalledMsg(const std_msgs::Bool::ConstPtr msg);
    void clearTable();
    void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr pose);
private Q_SLOTS:
    void on_clearButton_clicked();
    void on_msgTable_cellClicked(int row, int column);
    void on_radioButtons_updated();

private:
    int unreadMsgs;
    int numError;
    int numWarn;
    int maxRows;

    QTableWidget* msgTable;
    QPushButton* clearButton;
    QCheckBox* showOk;
    QCheckBox* showDebug;
    QCheckBox* showWarn;
    QCheckBox* showError;

    QStringList labels;
    std::string messagesPath;
    std::vector<completeRow*> messages;
    std::vector<std::string> errors;
    QFont bold;
    QFont normal;
    ros::Subscriber rosSubscriber;
    ros::Subscriber clearCalled;
    ros::Publisher callClear_pub;
    void loadFile();
    void updateTable();
    //QTreeWidget* jointTable;
    //QTreeWidgetItem joints[];
    //float positionLimits[];
    //float velocityLimits[];
    //float effortLimits[];

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;

Q_SIGNALS:
    void sendErrorData(QString,QString);
};

#endif // ROBOTSTATUS_H
