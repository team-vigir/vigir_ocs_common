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
#ifndef GLANCEHUB_H
#define GLANCEHUB_H

#include <vigir_atlas_control_msgs/VigirAtlasControlMode.h>
#include <vigir_ocs_msgs/OCSRobotStatus.h>
#include <vigir_ocs_msgs/OCSFootstepStatus.h>

#include <ros/subscriber.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <QMainWindow>
#include <QBasicTimer>
#include <QTableWidgetItem>
#include <QTableWidget>

namespace Ui {
class glancehub;
}


class glancehub : public QMainWindow
{
    Q_OBJECT

public:
    explicit glancehub(QWidget *parent = 0);
    ~glancehub();
    void controlModeMsgRcv(const vigir_atlas_control_msgs::VigirAtlasControlMode::ConstPtr msg);
    void robotStatusMoveit(const vigir_ocs_msgs::OCSRobotStatus::ConstPtr msg);
    //void robotStatusFootstep(const vigir_ocs_msgs::OCSRobotStatus::ConstPtr msg);
    void robotStatusFootstepComplete(const vigir_ocs_msgs::OCSFootstepStatus::ConstPtr msg);
    void loadFile();
    QString timeFromMsg(ros::Time stamp);
    QString getMoveitStat();
    QString getFootstepStat();

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glancehub *ui;
    ros::Subscriber control_mode_sub_;
    ros::Subscriber moveit_status_sub_;
    ros::Subscriber footstep_status_simple_sub_;
    ros::Subscriber footstep_status_sub_;
    ros::Subscriber obfsm_footstep_status_sub_;
    QBasicTimer timer;

    std::vector<std::string> allowed_control_modes_;

    std::vector<std::string> errors;
    std::string messagesPath;

Q_SIGNALS:
    void sendMoveitStatus(bool);
    void sendFootstepStatus(int);
    void sendFlorStatus(int);


};

#endif // glancehub_H
