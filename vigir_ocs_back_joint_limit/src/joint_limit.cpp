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
#include "joint_limit.h"
#include "ui_joint_limit.h"
#include <ros/package.h>
#include <vigir_planning_msgs/PlannerConfiguration.h>

#include <vigir_ocs_msgs/WindowCodes.h>

joint_limit::joint_limit(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::joint_limit)
{
    ui->setupUi(this);

    ros::NodeHandle pnh("~");

    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
    robot_model_ = robot_model_loader_->getModel();

    constraints_pub_ = nh_.advertise<vigir_planning_msgs::PlannerConfiguration>( "/flor/planning/upper_body/configuration",1,false);

    //key_event_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &joint_limit::processNewKeyEvent, this );

    std::vector<std::string> joints;
    std::vector<std::string> joint_descriptions;

    //joints.push_back("waist_pan");
    //joints.push_back("waist_tilt");

    std::string joint_list;
    std::string joint_description_list;
    pnh.param("joint_names", joint_list, std::string(""));
    pnh.param("joint_name_descriptions", joint_description_list, std::string(""));

    boost::algorithm::split(joints, joint_list, boost::is_any_of("\t "));
    boost::algorithm::split(joint_descriptions, joint_description_list, boost::is_any_of("\t "));

    bool use_descriptions = true;

    if (joints.size() != joint_descriptions.size()){
      ROS_WARN("Size of joint names list not same as joint name descriptions list! Not using descriptions!");
      use_descriptions = false;
    }

    if ((joints.size() > 0) && (joints[0].length() > 0)){

      for (size_t i = 0; i < joints.size(); ++i)
      {
        boost::shared_ptr<JointSettingControls> tmp;
        tmp.reset(new JointSettingControls(ui->horizontalLayout,
                                           joints[i],
                                           use_descriptions ? joint_descriptions[i] : joints[i],
                                           *robot_model_));

        joint_controls_.push_back(tmp);

      }
    }

    window_control_sub = nh_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &joint_limit::processWindowControl, this );
    window_control_pub = nh_.advertise<std_msgs::Int8>("/flor/ocs/window_control", 1, false);

    timer.start(33, this);

    //Restore State
    //this->show();
    QSettings settings("OCS", "joint_limit");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    this->geometry_ = this->geometry();
    // create docks, toolbars, etc...
    //this->restoreState(settings.value("mainWindowState").toByteArray());
}

joint_limit::~joint_limit()
{
    delete ui;
}

void joint_limit::closeEvent(QCloseEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());
    std_msgs::Int8 msg;
    msg.data = -WINDOW_PLANNER_CONFIG;
    window_control_pub.publish(msg);
    event->ignore();
}

void joint_limit::resizeEvent(QResizeEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());

}

void joint_limit::moveEvent(QMoveEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());

}

void joint_limit::on_apply_clicked()
{
    vigir_planning_msgs::PlannerConfiguration msg;

    for (size_t i = 0; i < joint_controls_.size(); ++i){

      // If locked, set both limits to same value
      if (joint_controls_[i]->isLocked()){
        vigir_planning_msgs::JointPositionConstraint constraint;
        constraint.joint_index = joint_controls_[i]->getJointIndex();

        constraint.joint_max = 100.0;
        constraint.joint_min = constraint.joint_max;

        msg.joint_position_constraints.push_back(constraint);

      //Only add constraint if bounds are different from limits
      }else if(joint_controls_[i]->constraintsDifferFromModelBounds()){
        vigir_planning_msgs::JointPositionConstraint constraint;

        constraint.joint_index = joint_controls_[i]->getJointIndex();
        constraint.joint_max = joint_controls_[i]->getMax();
        constraint.joint_min = joint_controls_[i]->getMin();

        msg.joint_position_constraints.push_back(constraint);
      }
    }

    msg.disable_collision_avoidance = !ui->collision_avoidance_->isChecked();
    msg.disable_left_hand_collision_avoidance = !ui->left_hand_collision_avoidance_->isChecked();
    msg.disable_right_hand_collision_avoidance = !ui->right_hand_collision_avoidance_->isChecked();
    msg.robot_collision_padding = ui->padding_->value();
    msg.trajectory_time_factor = ui->time_factor_->value();
    msg.octomap_max_height = ui->octomap_height_->value();
    msg.goal_cube_clearance = ui->octomap_clearance_cube_dimensions_->value();

    constraints_pub_.publish(msg);
}

void joint_limit::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
        
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

/*void joint_limit::on_Presets_comboBox_currentIndexChanged(int index)
{
    switch(index){
    case 0:
        lbzMinVal = -0.610865;
        lbzMaxVal = 0.610865;
        mbyMinVal = -1.2;
        mbyMaxVal = 1.28;
        ubxMinVal = -0.790809;
        ubxMaxVal = 0.790809;
        break;
    case 1:
        lbzMinVal = 0.0;
        lbzMaxVal = 0.001;
        mbyMinVal = 0.0;
        mbyMaxVal = 0.001;
        ubxMinVal = 0.0;
        ubxMaxVal = 0.001;
        break;
    case 2:
        lbzMinVal = -0.610865;
        lbzMaxVal = 0.610865;
        mbyMinVal = 0.0;
        mbyMaxVal = 0.15;
        ubxMinVal = -0.15;
        ubxMaxVal = 0.15;
        break;
    case 3:
        lbzMinVal = -0.610865;
        lbzMaxVal = 0.610865;
        mbyMinVal = 0.0;
        mbyMaxVal = 0.001;
        ubxMinVal = -0.0;
        ubxMaxVal = 0.001;
        break;
    default: break;
    }

    ui->lbzMin->setValue(lbzMinVal*1000000.0);
    ui->lbzMinLabel->setText(QString::number(lbzMinVal,'g',6));
    ui->lbzMax->setValue(lbzMaxVal*1000000.0);
    ui->lbzMaxLabel->setText(QString::number(lbzMaxVal,'g',6));

    ui->mbyMin->setValue(mbyMinVal*100.0);
    ui->mbyMinLabel->setText(QString::number(mbyMinVal,'g',6));
    ui->mbyMax->setValue(mbyMaxVal*100.0);
    ui->mbyMaxLabel->setText(QString::number(mbyMaxVal,'g',6));

    ui->ubxMin->setValue(ubxMinVal*1000000.0);
    ui->ubxMinLabel->setText(QString::number(ubxMinVal,'g',6));
    ui->ubxMax->setValue(ubxMaxVal*1000000.0);
    ui->ubxMaxLabel->setText(QString::number(ubxMaxVal,'g',6));

}*/

//void joint_limit::processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
//{
//    // store key state
//    if(key_event->state)
//        keys_pressed_list_.push_back(key_event->keycode);
//    else
//        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

//    // process hotkeys
//    std::vector<int>::iterator key_is_pressed;

//    key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
//    /*if(key_event->keycode == 13 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+4
//    {
//        if(this->isVisible())
//        {
//            this->hide();
//        }
//        else
//        {
//            this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
//            this->show();
//        }
//    }*/
//}

void joint_limit::processWindowControl(const std_msgs::Int8::ConstPtr& msg)
{
    if(!isVisible() && msg->data == WINDOW_PLANNER_CONFIG)
    {
        this->show();
        this->setGeometry(geometry_);
    }
    else if(isVisible() && (!msg->data || msg->data == -WINDOW_PLANNER_CONFIG))
    {
        geometry_ = this->geometry();
        this->hide();
    }
}
