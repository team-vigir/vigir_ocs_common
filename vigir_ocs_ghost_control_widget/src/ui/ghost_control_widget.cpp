#include <ros/package.h>

#include "ghost_control_widget.h"
#include "ui_ghost_control_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>

std::vector<unsigned char> GhostControlWidget::saved_state_planning_group_;
std::vector<unsigned char> GhostControlWidget::saved_state_pose_source_;
std::vector<unsigned char> GhostControlWidget::saved_state_world_lock_;
unsigned char GhostControlWidget::saved_state_collision_avoidance_;
unsigned char GhostControlWidget::saved_state_lock_pelvis_;

GhostControlWidget::GhostControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GhostControlWidget)
{    
    ui->setupUi(this);

    saveState();

    // subscribe to the topic to load state configurations
    state_sub_ = nh_.subscribe<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 5, &GhostControlWidget::processState, this );

    // advertise the topic to publish state configurations
    state_pub_ = nh_.advertise<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 1, false );
    reset_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/reset_pelvis", 1, false );
    send_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/send_pelvis_to_footstep", 1, false );

    // advertise set pose buttons
    set_to_target_pose_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_pose_state", 1, false );
    set_to_target_config_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_joint_state", 1, false );

    timer.start(33, this);
}

GhostControlWidget::~GhostControlWidget()
{
    delete ui;
}

void GhostControlWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void GhostControlWidget::processState(const flor_ocs_msgs::OCSGhostControl::ConstPtr &msg)
{
    // apply state coming from message
    loadState(msg->planning_group,msg->pose_source,msg->world_lock,msg->collision_avoidance,msg->lock_pelvis);

    // save as the last used state
    saveState();
}

void GhostControlWidget::publishState( bool snap )
{
    flor_ocs_msgs::OCSGhostControl cmd;
    cmd.planning_group = saved_state_planning_group_;
    cmd.pose_source = saved_state_pose_source_;
    cmd.world_lock = saved_state_world_lock_;
    //cmd.collision_avoidance = saved_state_collision_avoidance_;
    cmd.lock_pelvis = saved_state_lock_pelvis_;
    cmd.snap = snap;
    state_pub_.publish(cmd);
}

void GhostControlWidget::saveState()
{
    saved_state_planning_group_.clear();
    saved_state_pose_source_.clear();
    saved_state_world_lock_.clear();

    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(ui->planning_torso_->isChecked());

    if(ui->left_template_lock->isChecked())
    {
        saved_state_pose_source_.push_back(1);
        saved_state_world_lock_.push_back(1);
    }
    else
    {
        saved_state_pose_source_.push_back(0);
        saved_state_world_lock_.push_back(ui->left_marker_lock->isChecked());
    }

    if(ui->right_template_lock->isChecked())
    {
        saved_state_pose_source_.push_back(1);
        saved_state_world_lock_.push_back(1);
    }
    else
    {
        saved_state_pose_source_.push_back(0);
        saved_state_world_lock_.push_back(ui->right_marker_lock->isChecked());
    }

    saved_state_lock_pelvis_ = ui->lock_pelvis_->isChecked();

}

// default arguments are class members for saved state
void GhostControlWidget::loadState(std::vector<unsigned char> planning_group, std::vector<unsigned char> pose_source,
                                   std::vector<unsigned char> world_lock, unsigned char collision_avoidance,
                                   unsigned char lock_pelvis)
{
    /*ui->planning_left_->setChecked(planning_group[0]);
    ui->planning_right_->setChecked(planning_group[1]);
    ui->planning_torso_->setChecked(planning_group[2]);

    ui->pose_left_->setCurrentIndex(pose_source[0]);
    ui->pose_right_->setCurrentIndex(pose_source[1]);
    //ui->pose_torso_->setCurrentIndex(pose_source[2]);

    //ui->lock_left_->setChecked(world_lock[0]);
    //ui->lock_right_->setChecked(world_lock[1]);
    //ui->lock_torso_->setChecked(world_lock[2]);

    //ui->collision_->setChecked(collision_avoidance);

    ui->lock_pelvis_->setChecked(lock_pelvis);*/
}

void GhostControlWidget::applyClicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::cancelClicked()
{
    loadState();
}

void GhostControlWidget::snapClicked()
{
    publishState(true);
}

void GhostControlWidget::sendTargetPoseClicked()
{
    std_msgs::String cmd;

    bool left = saved_state_planning_group_[0];
    bool right = saved_state_planning_group_[1];
    bool torso = saved_state_planning_group_[2];

    if(left && !right && !torso)
        cmd.data = "l_arm_group";
    else if(left && !right && torso)
        cmd.data = "l_arm_with_torso_group";
    else if(!left && right && !torso)
        cmd.data = "r_arm_group";
    else if(!left && right && torso)
        cmd.data = "r_arm_with_torso_group";
    else if(left && right && !torso)
        cmd.data = "both_arms_group";
    else if(left && right && torso)
        cmd.data = "both_arms_with_torso_group";

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::sendTargetConfigClicked()
{
    std_msgs::String cmd;

    bool left = saved_state_planning_group_[0];
    bool right = saved_state_planning_group_[1];
    bool torso = saved_state_planning_group_[2];

    if(left && !right && !torso)
        cmd.data = "l_arm_group";
    else if(left && !right && torso)
        cmd.data = "l_arm_with_torso_group";
    else if(!left && right && !torso)
        cmd.data = "r_arm_group";
    else if(!left && right && torso)
        cmd.data = "r_arm_with_torso_group";
    else if(left && right && !torso)
        cmd.data = "both_arms_group";
    else if(left && right && torso)
        cmd.data = "both_arms_with_torso_group";

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::resetPelvisClicked()
{
    ROS_ERROR("RESET PELVIS!");
    std_msgs::Bool cmd;
    cmd.data = true;
    reset_pelvis_pub_.publish(cmd);
}

void GhostControlWidget::on_planning_left__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_planning_torso__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_planning_right__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_left__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_torso__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_right__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_left__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_torso__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_right__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_pelvis__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_send_left_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "l_arm_group";

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::on_send_left_torso_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "l_arm_with_torso_group";

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::on_send_right_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "r_arm_group";

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::on_send_right_torso_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "r_arm_with_torso_group";

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::on_pushButton_clicked()
{
    std_msgs::Bool cmd;
    cmd.data = true;
    send_pelvis_pub_.publish(cmd);
}

void GhostControlWidget::on_left_no_lock_toggled(bool checked)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_left_marker_lock_toggled(bool checked)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_left_template_lock_toggled(bool checked)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_right_no_lock_toggled(bool checked)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_right_marker_lock_toggled(bool checked)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_right_template_lock_toggled(bool checked)
{
    saveState();
    publishState();
}
