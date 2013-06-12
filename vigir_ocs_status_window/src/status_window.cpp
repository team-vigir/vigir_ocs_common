#include "status_window.h"
#include "ui_status_window.h"
#include "jointList.h"
#include <QtGui>
#include <QPainter>
#include <ros/ros.h>
#include <QLabel>
#include <flor_control_msgs/FlorControlModeCommand.h>

status_window::status_window(QWidget *parent) :
     QWidget(parent),
    ui(new Ui::status_window)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    mode_subscriber = nh.subscribe<flor_control_msgs::FlorControlMode>("/flor/controller/mode",1,&status_window::controlModeMsgRecieved, this);
    stability_subscriber = nh.subscribe<flor_ocs_msgs::OCSRobotStability>("/flor/controller/stability",1,&status_window::stabilityMsgRecieved, this);
    jntList = new jointList(NULL);
    jntList->hide();
    rbtStatus = new robotStatus(NULL);
    rbtStatus->hide();
    timer.start(33, this);
    timerColor.start(100,this);
    oldJointStyleSheet = ui->showJointButton->styleSheet();
    oldRobotStyleSheet = ui->showRobotStatus->styleSheet();
}

void status_window::controlModeMsgRecieved(const flor_control_msgs::FlorControlMode::ConstPtr& modeMsg)
{rbtStatus->show();
    //flor_control_msgs::FlorControlMode::LEFT_SIDE_DOWN
    switch(modeMsg->posture & 0x80)
    {
    case flor_control_msgs::FlorControlMode::LEFT_SIDE_DOWN:
        ui->postureLabel->setText(QString::fromStdString("Left Side Down"));
        break;
    case flor_control_msgs::FlorControlMode::RIGHT_SIDE_DOWN:
        ui->postureLabel->setText(QString::fromStdString("Right Side Down"));
        break;
    case flor_control_msgs::FlorControlMode::FRONT_SIDE_DOWN:
        ui->postureLabel->setText(QString::fromStdString("Front Side Down"));
        break;
    case flor_control_msgs::FlorControlMode::BACK_SIDE_DOWN:
        ui->postureLabel->setText(QString::fromStdString("Back Side Down"));
        break;
    case flor_control_msgs::FlorControlMode::UPRIGHT:
        ui->postureLabel->setText(QString::fromStdString("Upright"));
        break;
    case flor_control_msgs::FlorControlMode::VERTICAL:
        ui->postureLabel->setText(QString::fromStdString("Verticle"));
        break;
    case flor_control_msgs::FlorControlMode::HEAD_STAND:
        ui->postureLabel->setText(QString::fromStdString("Head Stand"));
        break;
    case flor_control_msgs::FlorControlMode::STABLE:
        ui->postureLabel->setText(QString::fromStdString("Stable"));
        break;
    default:
        ui->postureLabel->setText(QString::fromStdString("Unknown Posture"));
    }
    if(modeMsg->posture && 0x80)
        ui->postureLabel->setStyleSheet("QLabel { color: green; }");
    ui->l_armLabel->setText(getControllerStatus(modeMsg->left_arm));
    ui->l_legLabel->setText(getControllerStatus(modeMsg->left_leg));
    ui->r_armLabel->setText(getControllerStatus(modeMsg->right_arm));
    ui->r_legLabel->setText(getControllerStatus(modeMsg->right_leg));
    switch(modeMsg->behavior)
    {
    case flor_control_msgs::FlorControlModeCommand::STAND:
        ui->behaviorLabel->setText(QString::fromStdString("Stand"));
        break;
    case flor_control_msgs::FlorControlModeCommand::USER:
        ui->behaviorLabel->setText(QString::fromStdString("User"));
        break;
    case flor_control_msgs::FlorControlModeCommand::FREEZE:
        ui->behaviorLabel->setText(QString::fromStdString("Freeze"));
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND_PREP:
        ui->behaviorLabel->setText(QString::fromStdString("Stand Prep"));
        break;
    case flor_control_msgs::FlorControlModeCommand::WALK:
        ui->behaviorLabel->setText(QString::fromStdString("Walk"));
        break;
    case flor_control_msgs::FlorControlModeCommand::STEP:
        ui->behaviorLabel->setText(QString::fromStdString("Step"));
        break;
    case flor_control_msgs::FlorControlModeCommand::MANIPULATE:
        ui->behaviorLabel->setText(QString::fromStdString("Manipulate"));
        break;
    default:
        ui->behaviorLabel->setText(QString::fromStdString("Unknown"));
        break;
    }
}

QString status_window::getControllerStatus(uint8_t flag)
{
    switch(flag)
    {
    case flor_control_msgs::FlorControlMode::IDLE:
        return QString::fromStdString("Idle");
    case flor_control_msgs::FlorControlMode::TRAJECTORY_ACTIVE:
        return QString::fromStdString("Trajectory Active");
    case flor_control_msgs::FlorControlMode::TRAJECTORY_HOLD:
        return QString::fromStdString("Trajectory Hold");
    case flor_control_msgs::FlorControlMode::TRAJECTORY_WAIT:
        return QString::fromStdString("Trajectory Wait");
    case flor_control_msgs::FlorControlMode::TRAJECTORY_INVALID:
        return QString::fromStdString("Trajectory Invalid");
    case flor_control_msgs::FlorControlMode::TRAJECTORY_EMPTY:
        return QString::fromStdString("Trajectory Empty");
    case flor_control_msgs::FlorControlMode::POSITION:
        return QString::fromStdString("Position");
    case flor_control_msgs::FlorControlMode::TRACK:
        return QString::fromStdString("Track");
    case flor_control_msgs::FlorControlMode::BDI:
        return QString::fromStdString("BDI");
    case flor_control_msgs::FlorControlMode::COMPLIANT:
        return QString::fromStdString("Compliant");
    }
}

void status_window::stabilityMsgRecieved(const flor_ocs_msgs::OCSRobotStability::ConstPtr& stabilityMsg)
{
    ui->stabilityLabel->setText(QString::number(stabilityMsg->stability));
    if(stabilityMsg->stability >= 193)
        ui->stabilityLabel->setStyleSheet("QLabel { color: green; }");
    else if (stabilityMsg->stability < 92)
        ui->stabilityLabel->setStyleSheet("QLabel { color: red; }");
    else
        ui->stabilityLabel->setStyleSheet("QLabel { color: yellow; }");
    switch(stabilityMsg->stance)
    {
    case 0x01:
        ui->stanceLabel->setText(QString::fromStdString("Right Leg Stance"));
        break;
    case 0x02:
        ui->stanceLabel->setText(QString::fromStdString("Right Leg Swing"));
        break;
    case 0x10:
        ui->stanceLabel->setText(QString::fromStdString("Left Leg Stance"));
        break;rbtStatus->show();
    case 0x20:
        ui->stanceLabel->setText(QString::fromStdString("Left Leg Swing"));
        break;
    case 0x11:
        ui->stanceLabel->setText(QString::fromStdString("Double Support"));
        break;
    case 0x22:
        ui->stanceLabel->setText(QString::fromStdString("Flight"));
        break;
    default:
        ui->stanceLabel->setText(QString::fromStdString("Stance Unknown"));
    }
}

void status_window::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
    else
        updateButtonColor();
}

void status_window::updateButtonColor()
{
    ui->showJointButton->setStyleSheet(oldJointStyleSheet);
    if(jntList->isVisible())
        ui->showJointButton->setText(QString::fromStdString("Hide Joint List"));
    else
        ui->showJointButton->setText(QString::fromStdString("Show Joint List"));
    if(jntList->getNumWarn() > 0)
    {
        ui->showJointButton->setStyleSheet("QPushButton { background-color : yellow; }");
        ui->showJointButton->setText(ui->showJointButton->text() + QString::fromStdString(" Wrn:") + QString::number(jntList->getNumWarn()));
    }
    if(jntList->getNumError() > 0)
    {
        ui->showJointButton->setStyleSheet("QPushButton { background-color : red; }");
        ui->showJointButton->setText(ui->showJointButton->text() + QString::fromStdString(" Err:") + QString::number(jntList->getNumError()));
    }

    ui->showRobotStatus->setStyleSheet(oldRobotStyleSheet);
    if(rbtStatus->isVisible())
        ui->showRobotStatus->setText(QString::fromStdString("Hide Robot Status"));
    else
        ui->showRobotStatus->setText(QString::fromStdString("Show Robot Status"));
    if(rbtStatus->getNumWarn() > 0)
    {
        ui->showRobotStatus->setStyleSheet("QPushButton { background-color : yellow; }");
        ui->showRobotStatus->setText(ui->showRobotStatus->text() + QString::fromStdString(" Wrn:") + QString::number(rbtStatus->getNumWarn()));
    }
    if(rbtStatus->getNumError() > 0)
    {
        ui->showRobotStatus->setStyleSheet("QPushButton { background-color : red; }");
        ui->showRobotStatus->setText(ui->showRobotStatus->text() + QString::fromStdString(" Err:") + QString::number(rbtStatus->getNumError()));
    }
}

status_window::~status_window()
{
    delete ui;
}


void status_window::on_showJointButton_clicked()
{
    if(jntList->isVisible())
    {
        ui->showJointButton->setText(QString::fromStdString("Show Joint List"));
        jntList->hide();
    }
    else
    {
        jntList->show();
        ui->showJointButton->setText(QString::fromStdString("Hide Joint List"));
    }
}

void status_window::on_showRobotStatus_clicked()
{
    if(rbtStatus->isVisible())
    {
        rbtStatus->hide();
        ui->showRobotStatus->setText(QString::fromStdString("Show Robot Status"));
    }
    else
    {
        rbtStatus->show();
        ui->showRobotStatus->setText(QString::fromStdString("Hide Robot Status"));
    }
}
