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

    ui->pose_left_->setCurrentIndex(0);
    ui->pose_right_->setCurrentIndex(0);
    ui->pose_torso_->setCurrentIndex(0);

    saveState();

    // subscribe to the topic to load state configurations
    state_sub_ = nh_.subscribe<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 5, &GhostControlWidget::processState, this );

    // advertise the topic to publish state configurations
    state_pub_ = nh_.advertise<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 1, false );

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
    cmd.collision_avoidance = saved_state_collision_avoidance_;
    cmd.lock_pelvis = saved_state_lock_pelvis_;
    cmd.snap = snap;
    state_pub_.publish(cmd);
}

void GhostControlWidget::saveState()
{
    saved_state_planning_group_.clear();
    saved_state_pose_source_.clear();
    saved_state_world_lock_.clear();

    saved_state_planning_group_.push_back(ui->planning_left_->isChecked());
    saved_state_planning_group_.push_back(ui->planning_right_->isChecked());
    saved_state_planning_group_.push_back(ui->planning_torso_->isChecked());

    saved_state_pose_source_.push_back(ui->pose_left_->currentIndex());
    saved_state_pose_source_.push_back(ui->pose_right_->currentIndex());
    saved_state_pose_source_.push_back(ui->pose_torso_->currentIndex());

    saved_state_world_lock_.push_back(ui->lock_left_->isChecked());
    saved_state_world_lock_.push_back(ui->lock_right_->isChecked());
    saved_state_world_lock_.push_back(ui->lock_torso_->isChecked());

    saved_state_collision_avoidance_ = ui->collision_->isChecked();

    saved_state_lock_pelvis_ = ui->lock_pelvis_->isChecked();

}

// default arguments are class members for saved state
void GhostControlWidget::loadState(std::vector<unsigned char> planning_group, std::vector<unsigned char> pose_source,
                                   std::vector<unsigned char> world_lock, unsigned char collision_avoidance,
                                   unsigned char lock_pelvis)
{
    ui->planning_left_->setChecked(planning_group[0]);
    ui->planning_right_->setChecked(planning_group[1]);
    ui->planning_torso_->setChecked(planning_group[2]);

    ui->pose_left_->setCurrentIndex(pose_source[0]);
    ui->pose_right_->setCurrentIndex(pose_source[1]);
    ui->pose_torso_->setCurrentIndex(pose_source[2]);

    ui->lock_left_->setChecked(world_lock[0]);
    ui->lock_right_->setChecked(world_lock[1]);
    ui->lock_torso_->setChecked(world_lock[2]);

    ui->collision_->setChecked(collision_avoidance);

    ui->lock_pelvis_->setChecked(lock_pelvis);
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
