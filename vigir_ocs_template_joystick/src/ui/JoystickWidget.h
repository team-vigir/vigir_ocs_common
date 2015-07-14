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
@TODO_ADD_AUTHOR_INFO
#ifndef JOYSTICKWIDGET_H
#define JOYSTICKWIDGET_H

#include <ros/ros.h>

#include <std_msgs/Int8.h>

#include <QMainWindow>
#include <QString>
#include <QWidget>
#include <QtGui>
#include <QComboBox>
#include <QSettings>
#include "../Controller.h"

namespace Ui
{
    class JoystickWidget;
}

class JoystickWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget *parent = 0);
    ~JoystickWidget();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

public Q_SLOTS:
    void setDirection(QString s);
    void disableLeftLabel();
    void disableRightLabel();
    void selectTemplate();
    void populateTemplateComboBox(int tempId);
    void updateModeBoxes(int manipulationMode,int objectMode);

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

private:
    ros::Subscriber window_control_sub;
    ros::Publisher window_control_pub;
    QRect geometry_;

    Ui::JoystickWidget *ui;
    //handle multiple keys at once
    QSet<Qt::Key> keysPressed;
    void processKeys();
    vigir_ocs::Controller * controller;
    QSignalMapper * mapper;
    QString icon_path_;
};

#endif // JOYSTICKWIDGET_H
