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
#ifndef GLANCEHUBSBAR_H
#define GLANCEHUBSBAR_H

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <flor_control_msgs/FlorControlModeCommand.h>
#include <vigir_ocs_msgs/OCSRobotStatus.h>
#include <vigir_ocs_msgs/RobotStatusCodes.h>
#include <vigir_ocs_msgs/OCSFootstepStatus.h>
#include "glancehub.h"
#include "notification_system.h"

#include <QMainWindow>
#include <QPropertyAnimation>
#include <QDialog>
#include <QBasicTimer>
#include <QElapsedTimer>

namespace Ui {
class glancehubSbar;
}


class glancehubSbar : public QMainWindow
{
    Q_OBJECT

public:
    ~glancehubSbar();
    explicit glancehubSbar(QWidget *parent = 0);

public Q_SLOTS:
    void receiveMoveitStatus(bool);
    void receiveFootstepStatus(int);
    void receiveFlorStatus(int);
    void modeChanged(int);

    virtual bool eventFilter( QObject * o, QEvent * e );

private:
    void timerEvent(QTimerEvent *event);

    glancehub* ghub_;

    Ui::glancehubSbar *ui;
    ros::NodeHandle nh_;
    ros::Publisher mode_pub_;

    QString previous_selection_;
    QBasicTimer color_timer_;
    bool flashing_move_it_;
    bool flashing_footstep_;
    bool colored_moveit_;
    bool colored_footstep_;
    bool ignore_events_;
    int flash_footstep_counter_;
    int flash_moveit_counter_;
    QString flash_color_moveit_;
    QString flash_color_footstep_;
    QString white_;
    int max_flashes_;
    std::vector<std::string> allowed_control_modes_;

};

#endif // glancehub_H
