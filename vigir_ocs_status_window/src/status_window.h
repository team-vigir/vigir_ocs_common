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
#ifndef STATUS_WINDOW_H
#define STATUS_WINDOW_H

#include <QPainter>
#include <QPalette>
#include <QWidget>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <vigir_atlas_control_msgs/VigirAtlasControlMode.h>
#include <vigir_ocs_msgs/OCSRobotStability.h>
#include <vigir_ocs_msgs/OCSKeyEvent.h>

#include <jointList.h>
#include <robotStatus.h>

namespace Ui {
class status_window;
}

class status_window : public QWidget
{
    Q_OBJECT

public:
    explicit status_window(QWidget *parent = 0);
    ~status_window();

    void controlModeMsgReceived(const vigir_atlas_control_msgs::VigirAtlasControlMode::ConstPtr modeMsg);
    void stabilityMsgReceived(const vigir_ocs_msgs::OCSRobotStability::ConstPtr stabilityMsg);
    void updateButtonColor();
    QString getControllerStatus(uint8_t flag);

    void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr pose);

private Q_SLOTS:
    void on_showJointButton_clicked();
    void on_showRobotStatus_clicked();
protected:
    void timerEvent(QTimerEvent *event);
private:
    Ui::status_window *ui;
    ros::Subscriber mode_subscriber;
    ros::Subscriber stability_subscriber;
    robotStatus* rbtStatus;
    jointList* jntList;
    QBasicTimer timer;
    QBasicTimer timerColor;
    QString oldJointStyleSheet;
    QString oldRobotStyleSheet;

    std::vector<int> keys_pressed_list_;
    std::vector<std::string> allowed_control_modes_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;
};

#endif // STATUS_WINDOW_H
