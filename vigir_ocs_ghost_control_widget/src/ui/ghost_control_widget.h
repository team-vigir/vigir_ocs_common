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
#ifndef GhostControlWidget_H
#define GhostControlWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include <vigir_ocs_msgs/OCSTemplateList.h>
#include <vigir_grasp_msgs/InverseReachabilityForGraspRequest.h>
#include <vigir_object_template_msgs/GetGraspInfo.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "hotkey_manager.h"

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class GhostControlWidget;
}

class GhostControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit GhostControlWidget(QWidget *parent = 0);
    ~GhostControlWidget();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

    void processTemplateList( const vigir_ocs_msgs::OCSTemplateList::ConstPtr& list);        
    int calcTargetPose(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2, geometry_msgs::Pose& pose_result);


    void useTorsoContextMenu(const std_msgs::BoolConstPtr &msg);
    void snapGhostContextMenu(const std_msgs::BoolConstPtr &msg);

protected:
    void timerEvent(QTimerEvent *event);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

private Q_SLOTS:        
    void snapClicked();
    void sendTargetPoseClicked();
    void sendTargetConfigClicked();
    void resetPelvisClicked();    
    void on_planning_torso__clicked();
    void on_position_only_ik__clicked();
    void on_lock_pelvis__clicked();
    void on_use_drake_ik__clicked();
    void on_send_left_pose_button__clicked();    
    void on_send_right_pose_button__clicked();    
    void on_send_left_ghost_hand_button__clicked();
    void on_send_right_ghost_hand_button__clicked();
    void on_send_left_configuration_button__clicked();
    void on_send_right_configuration_button__clicked();    
    void on_send_upper_body_button__clicked();
    void on_send_whole_body_button__clicked();
    void on_send_torso_button__clicked();
    void on_send_left_leg_button__clicked();
    void on_send_right_leg_button__clicked();
    void on_pushButton_clicked();
    void on_send_ghost_to_template_button_clicked();
    void on_templateBox_activated(const QString &arg1);
    void on_graspBox_activated(const QString &arg1);
    void on_send_left_cartesian_button__clicked();
    void on_send_right_cartesian_button__clicked();

private:
    ros::Subscriber window_control_sub;
    ros::Publisher window_control_pub;
    QRect geometry_;

    std::string getGroupNameForSettings(const std::vector<unsigned char>& settings);
    std::vector< std::vector<QString> > readTextDBFile(QString path);

    Ui::GhostControlWidget* ui;

    ros::Subscriber snap_ghost_sub_;
    ros::Subscriber use_torso_sub_;

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_; // need to make sure we subscribe to others in case someone else changes the configuration
    ros::Subscriber template_list_sub_; // subscriber for template list
    ros::Publisher state_pub_;
    ros::Publisher set_to_target_pose_pub_;
    ros::Publisher set_to_target_config_pub_;
    ros::Publisher reset_pelvis_pub_;
    ros::Publisher send_pelvis_pub_;
    ros::Publisher send_inverse_rechability_req_pub_;
    ros::Publisher send_ghost_to_template_pub_;
    ros::Publisher send_ghost_cartesian_pub_;
    ros::Publisher send_template_to_behavior_pub_;

    ros::ServiceClient grasp_info_client_;
    ros::ServiceClient template_info_client_;

    // variables that hold saved state of the widget
    static std::vector<unsigned char> saved_state_planning_group_;
    static unsigned char saved_state_use_torso_;
    static unsigned char saved_state_lock_pelvis_;
    static unsigned char saved_state_position_only_ik_;
    static unsigned char saved_state_use_drake_ik_;

    vigir_ocs_msgs::OCSTemplateList last_template_list_;
    int selected_template_id_;
    int selected_pose_id_;


    QBasicTimer timer;

    void addHotkeys();

    ros::Publisher state_use_torso_pub_;
    ros::Publisher state_lock_pelvis_pub_;
    ros::Publisher state_position_only_ik_pub_;
    ros::Publisher state_use_drake_ik_pub_;
    ros::Publisher state_snap_ghost_to_robot_pub_;

};

#endif // GhostControlWidget_H
