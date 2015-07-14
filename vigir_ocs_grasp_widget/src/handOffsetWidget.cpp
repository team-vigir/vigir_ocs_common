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
#include "handOffsetWidget.h"
#include "ui_handOffsetWidget.h"

handOffsetWidget::handOffsetWidget(QWidget *parent) : saved_offsets(QString::fromStdString(ros::package::getPath("vigir_ocs_grasp_widget")) + "/saved_hand_offsets.ini", QSettings::IniFormat)
    , QWidget(parent)
    , ui(new Ui::handOffsetWidget)
{
    ui->setupUi(this);

    ros::NodeHandle nhp("~");

    nhp.param<std::string>("hand",hand_,"left");

    l_hand_template_offset_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/template/l_hand_template_offset", 1, false);
    r_hand_template_offset_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/template/r_hand_template_offset", 1, false);

    this->setWindowTitle(QString::fromStdString(hand_) + " Hand Template Offset Adjustment");

    //saved_offsets = new QSettings(QString("~/settings/" + QString::fromStdString(hand_) + "_offsets.ini"), QSettings::IniFormat);

    saved_offsets.beginGroup(QString::fromStdString(hand_));
    ui->current_offset->addItems(saved_offsets.childGroups());
    saved_offsets.endGroup();

    roll = 0;
    pitch = 0;
    yaw = 0;
    x = 0;
    y = 0;
    z = 0;
}

handOffsetWidget::~handOffsetWidget()
{
    //QSettings::~QSettings(saved_offsets);
    delete ui;
}

void handOffsetWidget::on_roll_inc_clicked()
{
    roll = ui->roll_current->value();
    roll += ui->roll_adj->value();
    ui->roll_current->display(roll);

    calc_offset();
}

void handOffsetWidget::on_roll_dec_clicked()
{
    roll = ui->roll_current->value();
    roll -= ui->roll_adj->value();
    ui->roll_current->display(roll);

    calc_offset();
}

void handOffsetWidget::on_pitch_inc_clicked()
{
    pitch = ui->pitch_current->value();
    pitch += ui->pitch_adj->value();
    ui->pitch_current->display(pitch);

    calc_offset();
}

void handOffsetWidget::on_pitch_dec_clicked()
{
    pitch = ui->pitch_current->value();
    pitch -= ui->pitch_adj->value();
    ui->pitch_current->display(pitch);

    calc_offset();
}

void handOffsetWidget::on_yaw_inc_clicked()
{
    yaw = ui->yaw_current->value();
    yaw += ui->yaw_adj->value();
    ui->yaw_current->display(yaw);

    calc_offset();
}

void handOffsetWidget::on_yaw_dec_clicked()
{
    yaw = ui->yaw_current->value();
    yaw -= ui->yaw_adj->value();
    ui->yaw_current->display(yaw);

    calc_offset();
}

void handOffsetWidget::on_x_inc_clicked()
{
    x = ui->x_current->value();
    x += ui->x_adj->value();
    ui->x_current->display(x);

    calc_offset();
}

void handOffsetWidget::on_x_dec_clicked()
{
    x = ui->x_current->value();
    x -= ui->x_adj->value();
    ui->x_current->display(x);

    calc_offset();
}

void handOffsetWidget::on_y_inc_clicked()
{
    y = ui->y_current->value();
    y += ui->y_adj->value();
    ui->y_current->display(y);

    calc_offset();
}

void handOffsetWidget::on_y_dec_clicked()
{
    y = ui->y_current->value();
    y -= ui->y_adj->value();
    ui->y_current->display(y);

    calc_offset();
}

void handOffsetWidget::on_z_inc_clicked()
{
    z = ui->z_current->value();
    z += ui->z_adj->value();
    ui->z_current->display(z);

    calc_offset();
}

void handOffsetWidget::on_z_dec_clicked()
{
    z = ui->z_current->value();
    z -= ui->z_adj->value();
    ui->z_current->display(z);

    calc_offset();
}

void handOffsetWidget::calc_offset()
{
    roll = ui->roll_current->value();
    pitch = ui->pitch_current->value();
    yaw = ui->yaw_current->value();

    x = ui->x_current->value();
    y = ui->y_current->value();
    z = ui->z_current->value();

    tf::Quaternion hand_quat;

    hand_quat.setEuler(yaw*DEG2RAD, pitch*DEG2RAD, roll*DEG2RAD);

    hand_offset.pose.orientation.w = hand_quat.w();
    hand_offset.pose.orientation.x = hand_quat.x();
    hand_offset.pose.orientation.y = hand_quat.y();
    hand_offset.pose.orientation.z = hand_quat.z();
    hand_offset.pose.position.x = x/100;
    hand_offset.pose.position.y = y/100;
    hand_offset.pose.position.z = z/100;

    if(hand_ == "left")
        l_hand_template_offset_pub_.publish(hand_offset);

    if(hand_ == "right")
        r_hand_template_offset_pub_.publish(hand_offset);
}

void handOffsetWidget::on_load_offset_clicked()
{
    saved_offsets.beginGroup(QString::fromStdString(hand_));
    saved_offsets.beginGroup(ui->current_offset->currentText());

    roll = saved_offsets.value("roll",0).toDouble();
    pitch = saved_offsets.value("pitch",0).toDouble();
    yaw = saved_offsets.value("yaw",0).toDouble();
    x = saved_offsets.value("x",0).toDouble();
    y = saved_offsets.value("y",0).toDouble();
    z = saved_offsets.value("z",0).toDouble();

    saved_offsets.endGroup();
    saved_offsets.endGroup();

    ui->roll_current->display(roll);
    ui->pitch_current->display(pitch);
    ui->yaw_current->display(yaw);

    ui->x_current->display(x);
    ui->y_current->display(y);
    ui->z_current->display(z);
}

void handOffsetWidget::on_save_offset_clicked()
{
    saved_offsets.beginGroup(QString::fromStdString(hand_));
    saved_offsets.beginGroup(ui->new_offset_name->text());
    saved_offsets.setValue("roll",roll);
    saved_offsets.setValue("pitch",pitch);
    saved_offsets.setValue("yaw",yaw);
    saved_offsets.setValue("x",x);
    saved_offsets.setValue("y",y);
    saved_offsets.setValue("z",z);

    saved_offsets.endGroup();
    saved_offsets.endGroup();

    saved_offsets.sync();

    if(ui->new_offset_name->text() != ui->current_offset->currentText())
    {
        ui->current_offset->addItem(ui->new_offset_name->text());
    }
}
