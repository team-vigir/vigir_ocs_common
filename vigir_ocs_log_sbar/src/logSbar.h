/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
#ifndef LOGSBAR_H
#define LOGSBAR_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/package.h>
#include <QPropertyAnimation>
#include <std_msgs/Int8.h>
#include "miniError.h"
#include "miniJoint.h"
#include <vigir_ocs_msgs/WindowCodes.h>

namespace Ui {
class LogSbar;
}

class LogSbar : public QMainWindow
{
    Q_OBJECT

private:
    Ui::LogSbar *ui;
    int numError;
    void setJointStatus(int);
    ros::NodeHandle n_;
    QPropertyAnimation * errorFadeIn;
    QPropertyAnimation * errorFadeOut;
    QPropertyAnimation * jointFadeIn;
    QPropertyAnimation * jointFadeOut;
    MiniError * miniError;
    MiniJoint * miniJoint;
    ros::Publisher window_control_pub_;

public:
    ~LogSbar();
    explicit LogSbar(QWidget *parent = 0);
    void resetErrorCount();
    Ui::LogSbar * getUi()
    {
        return ui;
    }
    QPropertyAnimation* getErrorFadeIn()
    {
        return errorFadeIn;
    }
    QPropertyAnimation* getErrorFadeOut()
    {
        return errorFadeOut;
    }
    QPropertyAnimation* getJointFadeIn()
    {
        return jointFadeIn;
    }
    QPropertyAnimation* getJointFadeOut()
    {
        return jointFadeOut;
    }
    MiniError * getMiniError()
    {
        return miniError;
    }
    MiniJoint * getMiniJoint()
    {
        return miniJoint;
    }
    ros::Publisher getWindowPublisher()
    {
        return window_control_pub_;
    }
    void notifyMiniError();
    void notifyMiniJoint();


public Q_SLOTS:
    void receiveErrorData(QString time, QString message);
    void receiveJointData(int,QString);

private Q_SLOTS:
    void hideErrorWindow();
    void hideJointWindow();

Q_SIGNALS:
    void makeErrorActive();
    void makeJointActive();
};

//classes for subwidgets to detect mouse enter/leave events easily within logSbar
class ErrorWidget : public QWidget
{
    Q_OBJECT
public:
    ~ErrorWidget();
    explicit ErrorWidget(QWidget *parent = 0);

private:
    LogSbar* myParent;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);
    void mousePressEvent(QMouseEvent*);
Q_SIGNALS:
    void toggleErrorLog();

};

class JointWidget : public QWidget
{
    Q_OBJECT
public:
    ~JointWidget();
    explicit JointWidget(QWidget *parent = 0);
private:
    LogSbar* myParent;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);    
Q_SIGNALS:
    void toggleJointList();
};

#endif // LOGSBAR_H

