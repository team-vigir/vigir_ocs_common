#include "widget.h"
#include "ui_widget.h"
#include<string.h>
# include <QAbstractButton>
#include<QLabel>
#include<ros/ros.h>
#include <ros/package.h>
#include<QHBoxLayout>
#include<QGridLayout>
#include<QTableWidgetItem>
#include<QFile>
#include<QTextStream>
#include<QDebug>
#include <ros/package.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    bold.setBold(true);
    normal.setBold(false);
    ui->stat->setColumnCount(3);
    ui->stat->setRowCount(5);
    count_row=0;// keep track of table rows
    ui->stat->setColumnWidth(0,145);
    ui->stat->setColumnWidth(1,50);
    ui->stat->setColumnWidth(2,300);
    std::cout << "Set width" << std::endl;
    labels.push_back("Sim Time");
    labels.push_back("Type");
    labels.push_back("Message Contents");
    ui->stat->setHorizontalHeaderLabels(labels);

    std::string fileName;
    if(nh.getParam("robotErrorFileLocation",fileName))
        messagesPath = fileName;
    else
        messagesPath = (ros::package::getPath("flor_ocs_msgs"))+"/include/flor_ocs_msgs/messages.csv";
    std::cout << "Reading messages from <" << messagesPath << ">" << std::endl;

    loadFile();

    maxRows = 100;
    unreadMsgs=0;
    numError = 0;
    numWarn = 0;
    avg_inlet_pr = -1;
    avg_air_sump_pressure= -1;
    avg_pump_rpm=-1;
    avg_pump_return_pressure=-1;
    avg_pump_supply_pressure=-1;
    filter_rate_ = 0.5; // little filtering with newer less frequent messages

    //How many data points until starts averaging
    averaging_delay = 5;

    ros::NodeHandle pnh("~");
    pnh.getParam("filter_rate",filter_rate_);
    ROS_INFO("Using Filter rate = %f for pump parameters", filter_rate_);

    ui->cs->setEnabled(false);
    ui->cs_list->setEnabled(false);
//    ui->cur_st->setEnabled(false);
    //ui->getlog->setEnabled(false);
    ui->high->setEnabled(false);
//    ui->last_stat->setEnabled(false);
//    ui->pr->setEnabled(false);
//    ui->curst->setEnabled(false);
//    ui->stat->setEnabled(false);
//    ui->robo_st->setEnabled(false);
    ui->send_mode->setEnabled(false);
    ui->start->setEnabled(false);
    ui->off->setEnabled(false);
    ui->low->setEnabled(false);
    ui->connect->setStyleSheet("background-color: grelabel_2en; color: black");
//    ui->d_state->setEnabled(false);
//    ui->d_label->setEnabled(false);
//    ui->r_state->setEnabled(false);
//    ui->pinlet->setEnabled(false);
//    ui->psump->setEnabled(false);
//    ui->psupply->setEnabled(false);
//    ui->preturn->setEnabled(false);
//    ui->return_2->setEnabled(false);
//    ui->sump->setEnabled(false);
//    ui->supply->setEnabled(false);
//    ui->pressure->setEnabled(false);
//    ui->temperature->setEnabled(false);
//    ui->inlet->setEnabled(false);
//    ui->rpm->setEnabled(false);
//    ui->prpm->setEnabled(false);
//    ui->timemeter->setEnabled(false);
//    ui->ptimemeter->setEnabled(false);
//    ui->rfault->setEnabled(false);
//    ui->fault->setEnabled(false);
//    ui->pst->setEnabled(false);
//    ui->ppst->setEnabled(false);
//    ui->mt->setEnabled(false);
//    ui->mdt->setEnabled(false);
//    ui->pmdt->setEnabled(false);
//    ui->pmt->setEnabled(false);
    ui->last_stat->setEnabled(true);
    ui->pr->setEnabled(true);
    ui->curst->setEnabled(true);
    ui->stat->setEnabled(true);
    ui->robo_st->setEnabled(true);
    ui->d_state->setEnabled(true);
    ui->d_label->setEnabled(true);
    ui->r_state->setEnabled(true);
    ui->pinlet->setEnabled(true);
    ui->psump->setEnabled(true);
    ui->psupply->setEnabled(true);
    ui->preturn->setEnabled(true);
    ui->return_2->setEnabled(true);
    ui->sump->setEnabled(true);
    ui->supply->setEnabled(true);
    ui->pressure->setEnabled(true);
    ui->temperature->setEnabled(true);
    ui->inlet->setEnabled(true);
    ui->rpm->setEnabled(true);
    ui->prpm->setEnabled(true);
    ui->timemeter->setEnabled(true);
    ui->ptimemeter->setEnabled(true);
    ui->rfault->setEnabled(true);
    ui->fault->setEnabled(true);
    ui->pst->setEnabled(true);
    ui->ppst->setEnabled(true);
    ui->mt->setEnabled(true);
    ui->mdt->setEnabled(true);
    ui->pmdt->setEnabled(true);
    ui->pmt->setEnabled(true);

    ui->right->setEnabled(true);
    ui->left->setEnabled(true);
    ui->both->setEnabled(true);
    ui->offhand->setEnabled(true);
    ui->onhand->setEnabled(true);
    ui->applyhand->setEnabled(false);
    ui->enableapplyhand->setEnabled(true);

    //sub_control = nh.subscribe<flor_control_msgs::FlorRobotStateCommand>("/flor/controller/robot_state_command", 5, &Widget::controlstate, this);
    pub = nh.advertise<flor_control_msgs::FlorRobotStateCommand> ("/flor/controller/robot_state_command",5,false);
    sub_state = nh.subscribe<flor_control_msgs::FlorRobotStatus>("/flor/controller/robot_status", 5, &Widget::robotstate, this);
    sub_behav = nh.subscribe<flor_control_msgs::FlorControlMode>("/flor/controller/mode",    5, &Widget::behavstate, this);
    sub_fault = nh.subscribe<flor_control_msgs::FlorRobotFault >("/flor/controller/robot_fault",  5, &Widget::robotfault, this);
    status_msg_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>( "/flor/controller/status",    100, &Widget::receivedMessage, this );

    pub_hand_power = nh.advertise<flor_atlas_msgs::AtlasHandPower> ("/flor/controller/hand_power",5,false);

    timer.start(33, this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}


void Widget::on_connect_clicked()
{
    if (ui->connect->text()=="CONNECT")
    {
        ui->connect->setText("CONNECT");
        ui->connect->setStyleSheet("background-color: green; color: black");
        ui->start->setStyleSheet("background-color: gray; color: black");
        ui->start->setEnabled(false);

        //publishing command "CONNECT"
        flor_control_msgs::FlorRobotStateCommand connect ;
        connect.state_command=flor_control_msgs::FlorRobotStateCommand::CONNECT;
        pub.publish(connect);
    }
    else
    {
        /*ui->connect->setText("CONNECT");
        ui->connect->setStyleSheet("background-color: green; color: black");
        ui->start->setStyleSheet("background-color: gray; color: black");
        ui->start->setEnabled(false);*/
        //publishing command "DISCONNECT"last_run_state
        flor_control_msgs::FlorRobotStateCommand disconnect ;
        disconnect.state_command=flor_control_msgs::FlorRobotStateCommand::DISCONNECT;
        pub.publish(disconnect);
        averaging_delay = 5;
    }
}

void Widget::receivedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg)
{
//    ui->stat->setColumnCount(3);
    //ui->stat->setRowCount(5);
    //count_row=0;// keep track of table rows
//    ui->stat->setColumnWidth(0,145);
//    ui->stat->setColumnWidth(1,50);
//    ui->stat->setColumnWidth(2,300);
//    //std::cout << "Set width" << std::endl;
//    labels.push_back("Sim Time");
//    labels.push_back("Type");
//    labels.push_back("Message Contents");
//    ui->stat->setHorizontalHeaderLabels(labels);
    uint8_t  level;
    uint16_t code;
    RobotStatusCodes::codes(msg->status, code,level); //const uint8_t& error, uint8_t& code, uint8_t& severity)
    //std::cout << "Recieved message. level = " << (int)level << " code = " << (int)code << std::endl;
    QTableWidgetItem* text = new QTableWidgetItem();
    QTableWidgetItem* msgType = new QTableWidgetItem();
    QTableWidgetItem* time = new QTableWidgetItem();
    time->setText(timeFromMsg(msg->stamp));
    text->setFlags(text->flags() ^ Qt::ItemIsEditable);
    time->setFlags(time->flags() ^ Qt::ItemIsEditable);
    msgType->setFlags(msgType->flags() ^ Qt::ItemIsEditable);
    switch(level){
    case 0:
        msgType->setText("Ok");
        break;
    case 1:
        msgType->setText("Debug");
        break;
    case 2:
        msgType->setText("Warn");
        text->setBackgroundColor(Qt::yellow);
        time->setBackgroundColor(Qt::yellow);
        msgType->setBackgroundColor(Qt::yellow);
        numWarn++;
        break;
    case 3:
        msgType->setText("Error");
        text->setBackgroundColor(Qt::red);
        time->setBackgroundColor(Qt::red);
        msgType->setBackgroundColor(Qt::red);
        numError++;
        break;
    }

    if(code >= errors.size() && errors.size() != 0)
    {
        std::cout << "Received message (Default Message). level = " << (int)level << " code = " << (int)code << std::endl;
        QString tempMessage = QString::fromStdString("Default Message");
        tempMessage+=QString::number(code);
        text->setText(tempMessage);
        text->setBackgroundColor(Qt::red);
        time->setBackgroundColor(Qt::red);
        msgType->setBackgroundColor(Qt::red);
        numError++;
    }
    else if(errors.size() > 0)
        text->setText(QString::fromStdString(errors[code]));
    else
    {
        std::cout << "Cannot find data file but recieved msg level = " << (int)level << " code = " << (int)code << std::endl;

        QString tempMessage = "Cannot find data file but recieved msg  num";
        tempMessage+= QString::number(code);
        text->setText(tempMessage);
        text->setBackground(Qt::red);
        msgType->setBackgroundColor(Qt::red);
        time->setBackgroundColor(Qt::red);
        numError++;
    }

    msgType->setFont(bold);
    time->setFont(bold);
    text->setFont(bold);
    //std::vector<completeRow*>::iterator it;

    //it = messages.begin();

    //messages.insert(it,new completeRow());
    // messages[0]->time = time;
    //messages[0]->priority = msgType;
    // messages[0]->text out>


    ui->stat->insertRow(0);

    //std::cout << "Adding item to table... " << messages.size() <<  " " << messages[0]->text << std::endl;
    ui->stat->setItem(0,0,time);
    ui->stat->setItem(0,1,msgType);
    ui->stat->setItem(0,2,text);
    count_row++;
    //qDebug() << count_row;
    /*for(int i=4;i>=0;i--)
           ui->stat->showRow(i);
      if(messages[0]->priority->text() == "Ok" && showOk->isChecked())
       {
           msgTable->showRow(0);
       }
       else if(messages[0]->priority->text() == "Debug" && showDebug->isChecked())
       {
           msgTable->showRow(0);
       }
       else if(messages[0]->priority->text() == "Warn" && showWarn->isChecked())
       {
           msgTable->showRow(0);
       }
       else if(messages[0]->priority->text() == "Error" && showError->isChecked())
       {
           msgTable->showRow(0);
       }
       else
           msgTable->hideRow(0);

       //std::cout << "Item added sucessfuly..." << std::endl;

       if(messages.size() > maxRows)
       {
           if(messages[messages.size()-1]->priority->text() == "Warn")
               numWarn--;
           else if(messages[messages.size()-1]->priority->text() == "Error")
               numError--;
           messages.pop_back();
           //ui->stat->removeRow(maxRows);
       }*/
    if(count_row>maxRows)
    {
        if(msgType->text()=="Warn")
            numWarn--;
        if(msgType->text()=="Error")
            numError--;
        ui->stat->removeRow(maxRows);
    }
    unreadMsgs++;

}
void Widget::loadFile()
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
QString Widget::timeFromMsg(ros::Time stamp)
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
void Widget:: robotstate( const flor_control_msgs::FlorRobotStatus::ConstPtr& msg )
{
    // save the last status message
    last_run_state = msg->robot_run_state;
    switch(msg->robot_run_state)
    {
    case 0:ui->r_state->setText("IDLE");break;
    case 1:ui->r_state->setText("START");break;
    case 3:ui->r_state->setText("CONTROL");break;
    case 5:ui->r_state->setText("STOP");break;
    }

    // Initialize the averages on first pass with a delay
    if(avg_inlet_pr==-1 || averaging_delay >= 0)
    {
        avg_inlet_pr                = msg->pump_inlet_pressure;
        avg_air_sump_pressure       = msg->air_sump_pressure;
        avg_pump_rpm                = msg->current_pump_rpm;
        avg_pump_return_pressure    = msg->pump_return_pressure;
        avg_pump_supply_pressure    = msg->pump_supply_pressure;
        avg_pump_supply_temperature = msg->pump_supply_temperature;
        avg_motor_temperature       = msg->motor_temperature;
        avg_motor_driver_temp       = msg->motor_driver_temperature;
        averaging_delay--;
    }
    else
    {
        // Average the noisy signals
        double one_minus = 1.0 - filter_rate_;
        avg_inlet_pr               = filter_rate_*msg->pump_inlet_pressure      + one_minus*avg_inlet_pr            ;
        avg_air_sump_pressure      = filter_rate_*msg->air_sump_pressure        + one_minus*avg_air_sump_pressure   ;
        avg_pump_rpm               = filter_rate_*msg->current_pump_rpm         + one_minus*avg_pump_rpm            ;
        avg_pump_return_pressure   = filter_rate_*msg->pump_return_pressure     + one_minus*avg_pump_return_pressure;
        avg_pump_supply_pressure   = filter_rate_*msg->pump_supply_pressure     + one_minus*avg_pump_supply_pressure;
        avg_pump_supply_temperature= filter_rate_*msg->pump_supply_temperature  + one_minus*avg_pump_supply_temperature;
        avg_motor_temperature      = filter_rate_*msg->motor_temperature        + one_minus*avg_motor_temperature      ;
        avg_motor_driver_temp      = filter_rate_*msg->motor_driver_temperature + one_minus*avg_motor_driver_temp      ;
    }



    // Update the text
    ui->inlet->setText(QString::number(avg_inlet_pr,'f',2));
    ui->sump->setText(QString::number(avg_air_sump_pressure,'f',2));
    ui->rpm->setText(QString::number(avg_pump_rpm,'f',2));
    ui->return_2->setText(QString::number(avg_pump_return_pressure,'f',2));
    ui->supply->setText(QString::number(avg_pump_supply_pressure,'f',2));
    ui->pst->setText(QString::number(avg_pump_supply_temperature,'f',2));
    ui->mt->setText(QString::number(avg_motor_temperature,'f',2));
    ui->mdt->setText(QString::number(avg_motor_driver_temp,'f',2));

    // Just set the time meter (no averaging)
    ui->timemeter->setText(QString::number(msg->pump_time_meter,'f',2));


    // Set the alarm colors on raw values
    if (msg->pump_supply_temperature>94.0)
        ui->pst->setStyleSheet("background-color: red");
    else if ((89.0 < msg->pump_supply_temperature) && (msg->pump_supply_temperature <= 94.0))
        ui->pst->setStyleSheet("background-color: yellow");
    else
        ui->pst->setStyleSheet("background-color: white");

    if (msg->motor_temperature>149.0)
        ui->mt->setStyleSheet("background-color: red");
    else if((124.0<msg->motor_temperature) && (msg->motor_temperature <=149.0))
        ui->mt->setStyleSheet("background-color: yellow");
    else
        ui->mt->setStyleSheet("background-color: white");

    if (msg->motor_driver_temperature>59.0)
        ui->mdt->setStyleSheet("background-color: red");
    else if ((54.0 < msg->motor_driver_temperature) && (msg->motor_driver_temperature<59.0))
        ui->mdt->setStyleSheet("background-color: yellow");
    else
        ui->mdt->setStyleSheet("background-color: white");

    // code to detect fault in inlet pressure
    if (msg->pump_inlet_pressure<50.0)
        ui->inlet->setStyleSheet("background-color: red");
    else if((50.0<msg->pump_inlet_pressure) && (msg->pump_inlet_pressure <69.0))
        ui->inlet->setStyleSheet("background-color: yellow");
    else
        ui->inlet->setStyleSheet("background-color: white");

    if (avg_air_sump_pressure<50.0)
        ui->sump->setStyleSheet("background-color: red");
    else if((50.0 <avg_air_sump_pressure) && (avg_air_sump_pressure<70.0))
        ui->sump->setStyleSheet("background-color: yellow");
    else
        ui->sump->setStyleSheet("background-color: white");

    if (msg->pump_return_pressure<50)
        ui->return_2->setStyleSheet("background-color: red");
    else if((50.0 <msg->pump_return_pressure) && (msg->pump_return_pressure<70.0))
        ui->return_2->setStyleSheet("background-color: yellow");
    else
        ui->return_2->setStyleSheet("background-color: white");


    if (msg->pump_supply_pressure<1500.0)
        ui->supply->setStyleSheet("background-color: red");
    else if ((1500.0 <msg->pump_supply_pressure) && (msg->pump_supply_pressure<2700.0))
        ui->supply->setStyleSheet("background-color: yellow");
    else
        ui->supply->setStyleSheet("background-color: white");

    // check if we are connected to the robotfalse
    if(msg->robot_connected==1)
    {
        ui->connect->setText("DISCONNECT");
        ui->connect->setStyleSheet("background-color: red; color: black");

        ui->pr->setEnabled(true);
        ui->high->setEnabled(true);
        ui->off->setEnabled(true);
        ui->low->setEnabled(true);
    }
    if(msg->robot_connected ==0)
    {
        ui->connect->setText("CONNECT");
        ui->connect->setStyleSheet("background-color: green; color: black");
        ui->start->setStyleSheet("background-color: gray; color: black");
        ui->start->setEnabled(false);
        averaging_delay = 5;
    }
    // check if we need to enable start
    if(msg->robot_run_state==0)
        enableStart();

    // check if run_state is different than idle to enable stop and all the other options in the UI
    if(msg->robot_run_state!=0)
    {
        ui->start->setEnabled(true);
        ui->start->setText("STOP");
        ui->start->setStyleSheet("background-color: red; color: black");
        ui->cs->setEnabled(true);
        ui->cs_list->setEnabled(true);
        ui->cur_st->setEnabled(true);
//        ui->last_stat->setEnabled(true);
//        ui->curst->setEnabled(true);
//        ui->stat->setEnabled(true);
//        ui->robo_st->setEnabled(true);
//        ui->d_label->setEnabled(true);
//        ui->d_state->setEnabled(true);
//        ui->r_state->setEnabled(true);
        ui->send_mode->setEnabled(true);
//        ui->pinlet->setEnabled(true);
//        ui->inlet->setEnabled(true);
//        ui->psump->setEnabled(true);
//        ui->psupply->setEnabled(true);
//        ui->preturn->setEnabled(true);
//        ui->return_2->setEnabled(true);
//        ui->pressure->setEnabled(true);
//        ui->temperature->setEnabled(true);
//        ui->sump->setEnabled(true);
//        ui->supply->setEnabled(true);
//        ui->inlet->setEnabled(true);
//        ui->pr->setEnabled(false);
//        ui->off->setEnabled(false);
//        ui->low->setEnabled(false);
//        ui->high->setEnabled(false);
//        ui->rpm->setEnabled(true);
//        ui->prpm->setEnabled(true);
//        ui->timemeter->setEnabled(true);
//        ui->ptimemeter->setEnabled(true);
//        ui->rfault->setEnabled(true);
//        ui->fault->setEnabled(true);
//        ui->pst->setEnabled(true);
//        ui->ppst->setEnabled(true);
//        ui->mt->setEnabled(true);
//        ui->mdt->setEnabled(true);
//        ui->pmdt->setEnabled(true);
//        ui->pmt->setEnabled(true);
        ui->right->setEnabled(true);
        ui->left->setEnabled(true);
        ui->both->setEnabled(true);
        ui->offhand->setEnabled(true);
        ui->onhand->setEnabled(true);
        ui->applyhand->setEnabled(false);
        ui->enableapplyhand->setEnabled(true);
    }

    if(msg->robot_critical_fault==1)
    {

        QLabel *fault_label = new QLabel();
        fault_label->setStyleSheet("background-color:yellow");
        QHBoxLayout *h = new QHBoxLayout();
        h->addWidget(fault_label);
        h->addWidget(ui->connect);
        ui->widget->setLayout(h);


    }

}
void Widget::robotfault(const flor_control_msgs::FlorRobotFault::ConstPtr& msg)
{

    ui->fault->setText(msg->message.c_str());

}

void Widget:: behavstate( const flor_control_msgs::FlorControlMode::ConstPtr& msg )
{
    switch(msg->bdi_current_behavior)
    { // defined in AtlasInterface or SimInterfaceCommand
    case atlas_msgs::AtlasSimInterfaceCommand::NONE:        ui->cur_st->setText("NONE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::FREEZE:      ui->cur_st->setText("FREEZE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP:  ui->cur_st->setText("STAND_PREP"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND:       ui->cur_st->setText("STAND"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::WALK:        ui->cur_st->setText("WALK"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STEP:        ui->cur_st->setText("STEP"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:  ui->cur_st->setText("MANIPULATE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::USER:        ui->cur_st->setText("USER"); break;
    case 8:                                                 ui->cur_st->setText("CALIBRATE"); break;  // not in simulation, but in AtlasInterface
    default: ROS_ERROR("Unknown current behavior = %d", msg->bdi_current_behavior); ui->cur_st->setText("Unknown!"); break;
    }
    switch(msg->bdi_desired_behavior)
    {
    case atlas_msgs::AtlasSimInterfaceCommand::NONE:        ui->d_state->setText("NONE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::FREEZE:      ui->d_state->setText("FREEZE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND_PREP:  ui->d_state->setText("STAND_PREP"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STAND:       ui->d_state->setText("STAND"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::WALK:        ui->d_state->setText("WALK"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::STEP:        ui->d_state->setText("STEP"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:  ui->d_state->setText("MANIPULATE"); break;
    case atlas_msgs::AtlasSimInterfaceCommand::USER:        ui->d_state->setText("USER"); break;
    case 8:                                                 ui->d_state->setText("CALIBRATE"); break;  // not in simulation, but in AtlasInterface
    default: ROS_ERROR("Unknown desired behavior = %d", msg->bdi_desired_behavior); ui->d_state->setText("Unknown!"); break;
    }

}
void Widget:: controlstate(const flor_control_msgs::FlorRobotStateCommand::ConstPtr& msg)
{
    if(msg->state_command==flor_control_msgs::FlorRobotStateCommand::FREEZE)
        ui->cs_list->setCurrentItem(ui->cs_list->findItems("FREEZE",Qt::MatchExactly)[0]);
    else if(msg->state_command==flor_control_msgs::FlorRobotStateCommand::STAND)
        ui->cs_list->setCurrentItem(ui->cs_list->findItems("STAND",Qt::MatchExactly)[0]);
    else if(msg->state_command==flor_control_msgs::FlorRobotStateCommand::STAND_PREP)
        ui->cs_list->setCurrentItem(ui->cs_list->findItems("STAND PREP",Qt::MatchExactly)[0]);
    else if(msg->state_command==flor_control_msgs::FlorRobotStateCommand::CALIBRATE)
        ui->cs_list->setCurrentItem(ui->cs_list->findItems("CALIBRATE",Qt::MatchExactly)[0]);

}
void Widget::on_start_clicked()
{
    if(ui->off->isChecked())
    {
        flor_control_msgs::FlorRobotStateCommand off ;
        off.state_command = flor_control_msgs::FlorRobotStateCommand::START_HYDRAULIC_PRESSURE_OFF;
        pub.publish(off);
    }
    else if(ui->low->isChecked())
    {
        flor_control_msgs::FlorRobotStateCommand low ;
        low.state_command = flor_control_msgs::FlorRobotStateCommand::START_HYDRAULIC_PRESSURE_LOW;
        pub.publish(low);
    }
    else if(ui->high->isChecked())
    {
        flor_control_msgs::FlorRobotStateCommand high ;
        high.state_command = flor_control_msgs::FlorRobotStateCommand::START_HYDRAULIC_PRESSURE_HIGH;
        pub.publish(high);
    }

    if (ui->start->text()=="STOP")
    {
        ui->start->setText("START");
        ui->start->setEnabled(false);
        ui->start->setStyleSheet("background-color: gray; color: black");
        ui->cs->setEnabled(false);
        ui->cs_list->setEnabled(false);
//        ui->cur_st->setEnabled(false);
//        ui->last_stat->setEnabled(false);
//        ui->curst->setEnabled(false);
//        ui->stat->setEnabled(false);
//        ui->robo_st->setEnabled(false);
//        ui->d_label->setEnabled(false);
//        ui->d_state->setEnabled(false);
//        ui->r_state->setEnabled(false);
        ui->send_mode->setEnabled(false);
        ui->start->setEnabled(true);
//        ui->pinlet->setEnabled(false);
//        ui->psump->setEnabled(false);
//        ui->psupply->setEnabled(false);
//        ui->preturn->setEnabled(false);
//        ui->pr->setEnabled(false);
//        ui->off->setEnabled(false);
//        ui->low->setEnabled(false);
//        ui->high->setEnabled(false);
//        ui->pr->setEnabled(false);
//        ui->high->setEnabled(false);
//        ui->off->setEnabled(false);
//        ui->low->setEnabled(false);
//        ui->rpm->setEnabled(false);
//        ui->prpm->setEnabled(false);
//        ui->timemeter->setEnabled(false);
//        ui->ptimemeter->setEnabled(false);
//        ui->rfault->setEnabled(false);
//        ui->fault->setEnabled(false);
         ui->right->setEnabled(false);
         ui->left->setEnabled(false);
         ui->both->setEnabled(false);
         ui->offhand->setEnabled(false);
         ui->onhand->setEnabled(false);
         ui->applyhand->setEnabled(false);
        // NEED TO SEND STOP MESSAGE HERE
        flor_control_msgs::FlorRobotStateCommand stop ;
        stop.state_command = flor_control_msgs::FlorRobotStateCommand::STOP;
        pub.publish(stop);
    }
}

// Checks if start should be enabled
void Widget::enableStart()
{
    if(last_run_state==0 && (ui->low->isChecked() || ui->off->isChecked() || ui->high->isChecked()))
    {
        ui->start->setEnabled(true);
        ui->start->setText("START");
        ui->start->setStyleSheet("background-color: green; color: black");
        //ui->pr->setEnabled(false);
        //ui->high->setEnabled(false);
        //ui->off->setEnabled(false);
        //ui->low->setEnabled(false);
    }
}

void Widget::on_off_clicked()
{
    enableStart();
}

void Widget::on_low_clicked()
{
    enableStart();
}

void Widget::on_high_clicked()
{
    enableStart();
}

void Widget::on_send_mode_clicked()
{
    if (ui->cs_list->currentItem()->text()=="FREEZE")
    {
        flor_control_msgs::FlorRobotStateCommand freeze ;
        freeze.state_command  = flor_control_msgs::FlorRobotStateCommand::FREEZE;
        pub.publish(freeze);
    }

    if (ui->cs_list->currentItem()->text()=="STAND")
    {
        flor_control_msgs::FlorRobotStateCommand stand ;
        stand.state_command  = flor_control_msgs::FlorRobotStateCommand::STAND;
        pub.publish(stand);
    }

    if (ui->cs_list->currentItem()->text()=="STAND PREP")
    {
        flor_control_msgs::FlorRobotStateCommand stand_prep ;
        stand_prep.state_command= flor_control_msgs::FlorRobotStateCommand::STAND_PREP;
        pub.publish(stand_prep);
    }

    if (ui->cs_list->currentItem()->text()=="CALIBRATE")
    {
        flor_control_msgs::FlorRobotStateCommand calibrate ;
        calibrate.state_command= flor_control_msgs::FlorRobotStateCommand::CALIBRATE;
        pub.publish(calibrate);
    }
}



void Widget::on_enableapplyhand_stateChanged(int arg1)
{
    qDebug()<<"enable"<<arg1;
    if (arg1==0 )
        ui->applyhand->setEnabled(false);
    if(arg1>0)
    {
        if((ui->right->isChecked()||ui->left->isChecked()||ui->both->isChecked()) && (ui->onhand->isChecked()||ui->offhand->isChecked()))
        ui->applyhand->setEnabled(true);
     }
}

void Widget::on_applyhand_clicked()
{
    flor_atlas_msgs::AtlasHandPower handpower;
    if(ui->right->isChecked())
        handpower.hand=flor_atlas_msgs::AtlasHandPower::HAND_RIGHT;
    if(ui->left->isChecked())
        handpower.hand=flor_atlas_msgs::AtlasHandPower::HAND_LEFT;
    if(ui->both->isChecked())
        handpower.hand= -1;
    if(ui->onhand->isChecked())
        handpower.power=flor_atlas_msgs::AtlasHandPower::HAND_POWER_ON;
    if(ui->offhand->isChecked())
        handpower.power = flor_atlas_msgs::AtlasHandPower::HAND_POWER_OFF;


    pub_hand_power.publish (handpower);
}
