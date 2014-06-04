#include "glancehubSbar.h"
#include <ros/ros.h>
#include "ui_glancehubSbar.h"
#include<QFile>
#include<QTextStream>
#include<QDebug>
#include <ros/package.h>
#include "flor_ocs_msgs/RobotStatusCodes.h"

glancehubSbar::glancehubSbar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glancehubSbar)
{
    ui->setupUi(this);
    ghub = new glancehub();

    connect(ghub,SIGNAL(sendMoveitStatus(bool)),this,SLOT(receiveMoveitStatus(bool)));
    connect(ghub,SIGNAL(sendFoostepStatus(int)),this,SLOT(receiveFootstepStatus(int)));
    connect(ghub,SIGNAL(sendFlorStatus(int)),this,SLOT(receiveFlorStatus(int)));
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(receiveModeChange(int)));

    ui->modelabel->setText("Flor_Off");// default setting is off

    // self.mode_pub = rospy.Publisher('/flor/controller/mode_command', FlorControlModeCommand)
    mode_pub = nh.advertise<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command", 1, false);

    ui->plannerLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
    ui->footstepLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
}

void glancehubSbar::receiveModeChange(int mode)
{

    QString modeBefore = ui->modelabel->text();
    int i = modeBefore.indexOf("->");
    if(i != -1) //arrow found?
        modeBefore = modeBefore.left(i-1); // only want last state

    flor_control_msgs::FlorControlModeCommand msg;
    msg.header.stamp = ros::Time::now();

    switch(mode)
    {
    case 0:
        ui->modelabel->setText(modeBefore+" -> Stand");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::STAND;
        break;
    case 1:
        ui->modelabel->setText(modeBefore+" -> Walk");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::WALK;
        break;
    case 2:
        ui->modelabel->setText(modeBefore+" -> Step");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::STEP;
        break;
    case 3:
        ui->modelabel->setText(modeBefore+" -> Manipulate");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::MANIPULATE;
        break;
    case 4:
        ui->modelabel->setText(modeBefore+" -> Flor_Stand");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_STAND;
        break;
    case 5:
        ui->modelabel->setText(modeBefore+" -> Flor_Walk_Mani");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_WALK_MANI;
        break;
    case 6:
        ui->modelabel->setText(modeBefore+" -> Flor_Step_Mani");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_STEP_MANI;
        break;
    case 7:
        ui->modelabel->setText(modeBefore+" -> Flor_Dance");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_DANCE;
        break;
    case 8:
        ui->modelabel->setText(modeBefore+" -> Flor_WBC");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_WBC;
        break;                        
    }
    mode_pub.publish(msg);
}
void glancehubSbar::receiveMoveitStatus(bool status)
{
    if(status)
        ui->plannerLight->setStyleSheet("QLabel { background-color: red;border:2px solid grey; }");
    else
        ui->plannerLight->setStyleSheet("QLabel { background-color: green; border:2px solid grey; }");
}

void glancehubSbar::receiveFootstepStatus(int status)
{    
    switch(status)
    {
    case RobotStatusCodes::FOOTSTEP_PLANNER_ACTIVE:
        ui->footstepLight->setStyleSheet("QLabel { background-color: yellow; border:2px solid grey;}");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_FAILED:
        ui->footstepLight->setStyleSheet("QLabel { background-color: red; border:2px solid grey; }");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_SUCCESS:
        ui->footstepLight->setStyleSheet("QLabel { background-color: green; border:2px solid grey;}");
        break;
    }
}

void glancehubSbar::receiveFlorStatus(int status)
{    
    QString newText;
    switch(status)
    {
    case flor_control_msgs::FlorControlModeCommand::FLOR_DANCE:
        newText = QString::fromStdString("Flor Dance");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_MANIPULATE:
        newText = QString::fromStdString("Flor Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_OFF:
        newText = QString::fromStdString("Flor Off");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STAND:
        newText = QString::fromStdString("Flor Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP:
        newText = QString::fromStdString("Flor Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP_MANI:
        newText = QString::fromStdString("Flor Step Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STOP:
        newText = QString::fromStdString("Flor Stop");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK:
        newText = QString::fromStdString("Flor Walk");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK_MANI:
        newText = QString::fromStdString("Flor Walk Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WBC:
        newText = QString::fromStdString("Flor WBC");
        break;
    case flor_control_msgs::FlorControlModeCommand::FREEZE:
        newText = QString::fromStdString("Freeze");
        break;
    case flor_control_msgs::FlorControlModeCommand::MANIPULATE:
        newText = QString::fromStdString("Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND:
        newText = QString::fromStdString("Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND_PREP:
        newText = QString::fromStdString("Stand Prep");
        break;
    case flor_control_msgs::FlorControlModeCommand::STEP:
        newText = QString::fromStdString("Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::USER:
        newText = QString::fromStdString("User");
        break;
    case flor_control_msgs::FlorControlModeCommand::WALK:
        newText = QString::fromStdString("Walk");
        break;
    }
    ROS_ERROR("receiving status : %s", qPrintable(newText));
    ui->modelabel->setText(newText);
}

glancehubSbar::~glancehubSbar()
{
    delete(ghub);
    delete ui;    
}
