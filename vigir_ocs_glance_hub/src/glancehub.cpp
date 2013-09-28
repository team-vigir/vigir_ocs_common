#include "glancehub.h"
#include <ros/ros.h>
#include "ui_glancehub.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"

glanceHub::glanceHub(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glanceHub)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    controlMode_sub = nh.subscribe<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command",5,&glanceHub::controlModeMsgRcv, this);
    robotStatusMoveit_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/planning/upper_body/status",2,&glanceHub::robotStatusMoveit,this);
    robotStatusFootstep_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/footstep_planner/status",2,&glanceHub::robotStatusFootstep,this);
    key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &glanceHub::processNewKeyEvent, this );
    timer.start(33, this);
}

glanceHub::~glanceHub()
{
    delete ui;
}

void glanceHub::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
}

void glanceHub::robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    if(msg->status != RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE)
        ui->plannerLight->setStyleSheet("QLabel { background-color: red; }");
    else
        ui->plannerLight->setStyleSheet("QLabel { background-color: green; }");
}

void glanceHub::robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
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

void glanceHub::controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg)
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

void glanceHub::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    std::vector<int>::iterator key_is_pressed;

    key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
    if(key_event->key == 10 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+1
    {
        if(this->isVisible())
        {
            this->hide();
        }
        else
        {
            //this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
            this->show();
        }
    }
}
