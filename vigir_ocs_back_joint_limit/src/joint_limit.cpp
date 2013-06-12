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
    lbzMinVal = -10.6109;
    lbzMaxVal = 10.6109;

    mbyMinVal = -11.2;
    mbyMaxVal = 11.28;

    ubxMinVal = -10.7908;
    ubxMaxVal = 10.7908;
}

joint_limit::~joint_limit()
{
    delete ui;
}
void joint_limit::on_lbzMin_sliderReleased()
{
    if(ui->lbzMin->value() >= lbzMaxVal*10000.0)
        ui->lbzMin->setValue(lbzMinVal*10000.0);
    else
    {
        lbzMinVal = (float)ui->lbzMin->value()/10000.0;
        ui->lbzMinLabel->setText(QString::number(lbzMinVal,'g',6));
    }
}

void joint_limit::on_lbzMax_sliderReleased()
{
    if(ui->lbzMax->value() <= lbzMinVal*10000.0)
        ui->lbzMax->setValue(lbzMaxVal*10000.0);
    else
    {
        lbzMaxVal = (float)ui->lbzMax->value()/10000.0;
        ui->lbzMaxLabel->setText(QString::number(lbzMaxVal,'g',6));
    }
}

void joint_limit::on_mbyMin_sliderReleased()
{
    if(ui->mbyMin->value() >= mbyMaxVal*100.0)
        ui->mbyMin->setValue(mbyMinVal*100.0);
    else
    {
        mbyMinVal = (float)ui->mbyMin->value()/100.0;
        ui->mbyMinLabel->setText(QString::number(mbyMinVal,'g',6));
    }
}

void joint_limit::on_mbyMax_sliderReleased()
{
    if(ui->mbyMax->value() <= mbyMinVal*100.0)
        ui->mbyMax->setValue(mbyMaxVal*100.0);
    else
    {
        mbyMaxVal = (float)ui->mbyMax->value()/100.0;
        ui->mbyMaxLabel->setText(QString::number(mbyMaxVal,'g',6));
    }
}

void joint_limit::on_ubxMin_sliderReleased()
{
    if(ui->ubxMin->value() >= ubxMaxVal*10000.0)
        ui->ubxMin->setValue(ubxMinVal*10000.0);
    else
    {
        ubxMinVal = (float)ui->ubxMin->value()/10000.0;
        ui->ubxMinLabel->setText(QString::number(ubxMinVal,'g',6));
    }
}

void joint_limit::on_ubxMax_sliderReleased()
{
    if(ui->ubxMax->value() <= ubxMinVal*10000.0)
        ui->ubxMax->setValue(ubxMaxVal*10000.0);
    else
    {
        ubxMaxVal = (float)ui->ubxMax->value()/10000.0;
        ui->ubxMaxLabel->setText(QString::number(ubxMaxVal,'g',6));
    }
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
