#include "glancehubsbar.h"
#include <ros/ros.h>
#include "ui_glancehubsbar.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"

glancehubsbar::glancehubsbar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glancehubsbar)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    controlMode_sub = nh.subscribe<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command",5,&glancehubsbar::controlModeMsgRcv, this);
    robotStatusMoveit_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/planning/upper_body/status",2,&glancehubsbar::robotStatusMoveit,this);
    robotStatusFootstep_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/footstep_planner/status",2,&glancehubsbar::robotStatusFootstep,this);
    timer.start(33, this);
}

void glancehubsbar::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
}

void glancehubsbar::robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    if(msg->status != RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE)
        ui->plannerLight->setStyleSheet("QLabel { background-color: red; }");
    else
        ui->plannerLight->setStyleSheet("QLabel { background-color: green; }");
}

void glancehubsbar::robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    switch(msg->status)
    {
    case RobotStatusCodes::FOOTSTEP_PLANNER_ACTIVE:
        ui->footLight->setStyleSheet("QLabel { background-color: yellow; }");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_FAILED:
        ui->footLight->setStyleSheet("QLabel { background-color: red; }");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_SUCCESS:
        ui->footLight->setStyleSheet("QLabel { background-color: green; }");
        break;
    }

}

void glancehubsbar::controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg)
{
    QString newText;
    switch(msg->behavior)
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
        newText = QString::fromStdString("Flor Freeze");
        break;
    case flor_control_msgs::FlorControlModeCommand::MANIPULATE:
        newText = QString::fromStdString("Flor Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND:
        newText = QString::fromStdString("Flor Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND_PREP:
        newText = QString::fromStdString("Flor Stand Prep");
        break;
    case flor_control_msgs::FlorControlModeCommand::STEP:
        newText = QString::fromStdString("Flor Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::USER:
        newText = QString::fromStdString("Flor User");
        break;
    case flor_control_msgs::FlorControlModeCommand::WALK:
        newText = QString::fromStdString("Flor Walk");
        break;
    }
    ui->controlModeLabel->setText(newText);
    std::cout << "Changing to "<< newText.toStdString() << " Mode" << std::endl;
}

glancehubsbar::~glancehubsbar()
{
    delete ui;
}
