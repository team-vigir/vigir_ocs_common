#include "glancehub.h"
#include <ros/ros.h>
#include "ui_glancehub.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include<QFile>
#include<QTextStream>
#include<QDebug>
#include <ros/package.h>


glancehub::glancehub(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glancehub)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    controlMode_sub = nh.subscribe<flor_control_msgs::FlorControlMode>("/flor/controller/mode",5,&glancehub::controlModeMsgRcv, this);
    robotStatusMoveit_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/planning/upper_body/status",2,&glancehub::robotStatusMoveit,this);
    robotStatusFootstep_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/footstep_planner/status",2,&glancehub::robotStatusFootstep,this);

    // load control modes into dropdown box from parameters
    nh.getParam("/atlas_controller/allowed_control_modes", allowed_control_modes_);

    timer.start(33, this);
    std::string fileName;
    if(nh.getParam("robotErrorFileLocation",fileName))
        messagesPath = fileName;
    else
        messagesPath = (ros::package::getPath("flor_ocs_msgs"))+"/include/flor_ocs_msgs/messages.csv";
    std::cout << "Reading messages from <" << messagesPath << ">" << std::endl;
    loadFile();

}

void glancehub::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
}

QString glancehub::getMoveitStat()
{
    return ui->moveitstat->text();
}
QString glancehub::getFootstepStat()
{
    return ui->footstepstat->text();
}

void glancehub::robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    int count_row;
    int unreadMsgs;
    int numError;
    int numWarn;
    int maxRows;
    QFont bold;
    QFont normal;

    if(msg->status != RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE)
        ui->plannerLight->setStyleSheet("QLabel { background-color: red; }");
    else
        ui->plannerLight->setStyleSheet("QLabel { background-color: green; }");
    //update status bar
    Q_EMIT sendMoveitStatus(msg->status != RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE);

    uint8_t  level;
    uint16_t code;
    RobotStatusCodes::codes(msg->status, code,level); //const uint8_t& error, uint8_t& code, uint8_t& severity)
    std::cout << "Received message. level = " << (int)level << " code = " << (int)code << std::endl;

    QString text ;
    QString msgType ;
    switch(level){
    case 0:

        msgType="Ok";
        break;
    case 1:
        msgType="Debug";
        break;
    case 2:
        msgType="Warn";
        numWarn++;
        break;
    case 3:
        msgType="Error";
        numError++;
        break;
    }

    if(code >= errors.size() && errors.size() != 0)
    {
        std::cout << "Received message (Default Message). level = " << (int)level << " code = " << (int)code << std::endl;
        QString tempMessage = QString::fromStdString("Default Message");
        tempMessage+=QString::number(code);
        text=(tempMessage);
        numError++;
    }
    else if(errors.size() > 0)
    {
        text=QString::fromStdString(errors[code]);
    }
    else
    {
        std::cout << "Cannot find data file but recieved msg level = " << (int)level << " code = " << (int)code << std::endl;

        QString tempMessage = "Cannot find data file but recieved msg  num";
        tempMessage+= QString::number(code);
        text=tempMessage;
        numError++;
    }
    ui->moveitstat->setText(msgType+"("+text+")");
    count_row++;
    qDebug() << count_row;
    if(count_row>maxRows)
    {
        if(msgType=="Warn")
            numWarn--;
        if(msgType=="Error")
            numError--;

    }
    unreadMsgs++;
}

void glancehub::loadFile()
{
    errors.resize(RobotStatusCodes::MAX_ERROR_MESSAGES,"Default Error Message");
    QFile file(QString::fromStdString(messagesPath));
    std::cout << "Trying to open file at " << messagesPath << std::endl;
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        std::cout << "File opened successfully... now parsing. Will print valid messages" << std::endl;
        QTextStream in(&file);
        while(!in.atEnd())
        {
            QString line = in.readLine();
            if(line[0] != '#')
            {
                QStringList strings;
                strings = line.split(',');
                if(strings.size() > 1)
                {
                    errors[strings[0].toInt()] = strings[1].toStdString();
                    std::cout << "Msg # " << strings[0].toStdString() << ":" << strings[1].toStdString() <<std::endl;
                }
            }
        }
    }
}

void glancehub::robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    int count_row;
    int unreadMsgs;
    int numError;
    int numWarn;
    int maxRows;
    QFont bold;
    QFont normal;

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
    //notify status bar
    Q_EMIT sendFootstepStatus(msg->status);

    uint8_t  level;
    uint16_t code;
    RobotStatusCodes::codes(msg->status, code,level); //const uint8_t& error, uint8_t& code, uint8_t& severity)
    std::cout << "Recieved message. level = " << (int)level << " code = " << (int)code << std::endl;

    QString text ;
    QString msgType ;
    switch(level){
    case 0:

        msgType="Ok";
        break;
    case 1:
        msgType="Debug";
        break;
    case 2:
        msgType="Warn";
        numWarn++;
        break;
    case 3:
        msgType="Error";
        numError++;
        break;
    }

    if(code >= errors.size() && errors.size() != 0)
    {
        std::cout << "Received message (Default Message). level = " << (int)level << " code = " << (int)code << std::endl;
        QString tempMessage = QString::fromStdString("Default Message");
        tempMessage+=QString::number(code);
        text=(tempMessage);
        numError++;
    }
    else if(errors.size() > 0)
    {
        text=QString::fromStdString(errors[code]);
    }
    else
    {
        std::cout << "Cannot find data file but recieved msg level = " << (int)level << " code = " << (int)code << std::endl;

        QString tempMessage = "Cannot find data file but recieved msg  num";
        tempMessage+= QString::number(code);
        text=tempMessage;
        numError++;
    }
    ui->footstepstat->setText(msgType+"("+text+")");
    count_row++;
    qDebug() << count_row;
    if(count_row>maxRows)
    {
        if(msgType=="Warn")
            numWarn--;
        if(msgType=="Error")
            numError--;

    }
    unreadMsgs++;


}

void glancehub::controlModeMsgRcv(const flor_control_msgs::FlorControlMode::ConstPtr& msg)
{
    QString newText;
    if (msg->control_mode >= 0 && msg->control_mode <  allowed_control_modes_.size())
        newText = QString::fromStdString(allowed_control_modes_[msg->control_mode]);
    else
        newText = QString::fromStdString("Unknown");

    //notify status bar
    Q_EMIT sendFlorStatus(msg->control_mode);

    ui->controlModeLabel->setText(newText);
    std::cout << "Changing to "<< newText.toStdString() << " Mode" << std::endl;
}
QString glancehub::timeFromMsg(ros::Time stamp)
{

    double dSec = stamp.toSec();
    int sec = dSec;
    std::stringstream stream;

    stream.str("");
    int day = sec/86400;
    sec -= day * 86400;

    int hour = sec / 3600;
    sec -= hour * 3600;

    int min = sec / 60;
    sec -= min * 60;

    int iSec = dSec;
    dSec -= iSec;
    int ms = (dSec*1000.0);

    stream << std::setw(2) << std::setfill('0') << day << " ";
    stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
    stream << std::setw(3) << std::setfill('0') << ms ;
    return QString::fromStdString(stream.str());
}

glancehub::~glancehub()
{
    delete ui;
}
