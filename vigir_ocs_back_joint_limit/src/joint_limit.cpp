#include "joint_limit.h"
#include "ui_joint_limit.h"
#include <ros/package.h>
#include <flor_planning_msgs/JointPositionConstraints.h>

joint_limit::joint_limit(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::joint_limit)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    constraints_pub_ = nh.advertise<flor_planning_msgs::JointPositionConstraints>( "/flor/planning/torso_joint_limits",1,false);
    timer.start(33, this);
}

joint_limit::~joint_limit()
{
    delete ui;
}
void joint_limit::on_apply_clicked()
{
    flor_planning_msgs::JointPositionConstraints msg;

    msg.back_lbz_max.data = (float)ui->lbzMax->value();
    msg.back_lbz_min.data = (float)ui->lbzMin->value();

    msg.back_mby_max.data = (float)ui->mbyMax->value();
    msg.back_mby_max.data = (float)ui->mbyMin->value();

    msg.back_ubx_max.data = (float)ui->ubxMax->value();
    msg.back_ubx_max.data = (float)ui->ubxMin->value();
    std::cout << "The following values were set:" <<std::endl;
    std::cout << "lbz: max = " << ui->lbzMax->value() << " min = " << ui->lbzMin->value() << std::endl;
    std::cout << "mby: max = " << ui->mbyMax->value() << " min = " << ui->mbyMin->value() << std::endl;
    std::cout << "ubx: max = " << ui->ubxMax->value() << " min = " << ui->ubxMin->value() << std::endl << std::endl;
    constraints_pub_.publish(msg);
}

void joint_limit::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}
