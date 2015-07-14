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
#ifndef HANDOFFSETWIDGET_H
#define HANDOFFSETWIDGET_H

#include <QWidget>
#include <QSettings>
#include <QDir>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

//#include <graspWidget.h>

#define DEG2RAD 0.01745329

namespace Ui {
class handOffsetWidget;
}

class handOffsetWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit handOffsetWidget(QWidget *parent = 0);
    ~handOffsetWidget();
    
public Q_SLOTS:
    void on_roll_inc_clicked();
    void on_roll_dec_clicked();
    void on_pitch_inc_clicked();
    void on_pitch_dec_clicked();
    void on_yaw_inc_clicked();
    void on_yaw_dec_clicked();
    void on_x_inc_clicked();
    void on_x_dec_clicked();
    void on_y_inc_clicked();
    void on_y_dec_clicked();
    void on_z_inc_clicked();
    void on_z_dec_clicked();
    void on_load_offset_clicked();
    void on_save_offset_clicked();

private:
    Ui::handOffsetWidget *ui;

    std::string hand_;

    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;

    ros::NodeHandle nh_;
    ros::Publisher l_hand_template_offset_pub_;
    ros::Publisher r_hand_template_offset_pub_;
    geometry_msgs::PoseStamped hand_offset;

    QSettings saved_offsets;

    void calc_offset();
};

#endif // HANDOFFSETWIDGET_H
