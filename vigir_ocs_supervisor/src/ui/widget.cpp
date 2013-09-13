#include "widget.h"
#include "ui_widget.h"
#include<string.h>
# include <QAbstractButton>
#include<QLabel>
#include<ros/ros.h>
#include <ros/package.h>
#include<QHBoxLayout>
#include<QGridLayout>


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    /*QGridLayout * grid = new QGridLayout();
    grid->addWidget(ui->pinlet,0,0);
    grid->addWidget(ui->inlet,0,1);
    grid->addWidget(ui->preturn,0,2);
    grid->addWidget(ui->return_2,0,3);
    grid->addWidget(ui->psump,1,0);
    grid->addWidget(ui->sump,1,1);
    grid->addWidget(ui->psupply,1,2);
    grid->addWidget(ui->supply,1,3);
    grid->addWidget(ui->ptimemeter,2,0);
    grid->addWidget(ui->timemeter,2,1);
    grid->addWidget(ui->prpm,2,2);
    grid->addWidget(ui->rpm,2,3);
    grid->addWidget(ui->ppst,3,0);
    grid->addWidget(ui->pst,3,1);
    grid->addWidget(ui->pmt,3,2);
    grid->addWidget(ui->mt,3,3);
    grid->addWidget(ui->pmdt,4,0);
    grid->addWidget(ui->mdt,4,1);
    ui->widget_6->setLayout(grid);
    */
    last_inlet_pr = -1;
    last_air_sump_pressure= -1;
    last_pump_rpm=-1;
    last_pump_return_pressure=-1;
    last_pump_supply_pressure=-1;
    last_pump_time_meter=-1;
    last_pt = -1;
    last_mt = -1;
    last_mdt = -1;
    ui->setupUi(this);
    ui->cs->setEnabled(false);
    ui->cs_list->setEnabled(false);
    ui->cur_st->setEnabled(false);
    //ui->getlog->setEnabled(false);
    ui->high->setEnabled(false);
    ui->last_stat->setEnabled(false);
    ui->pr->setEnabled(false);
    ui->curst->setEnabled(false);
    ui->stat->setEnabled(false);
    ui->robo_st->setEnabled(false);
    ui->send_mode->setEnabled(false);
    ui->start->setEnabled(false);
    ui->off->setEnabled(false);
    ui->low->setEnabled(false);
    ui->connect->setStyleSheet("background-color: grelabel_2en; color: black");
    ui->d_state->setEnabled(false);
    ui->d_label->setEnabled(false);
    ui->r_state->setEnabled(false);
    ui->pinlet->setEnabled(false);
    ui->psump->setEnabled(false);
    ui->psupply->setEnabled(false);
    ui->preturn->setEnabled(false);
    ui->return_2->setEnabled(false);
    ui->sump->setEnabled(false);
    ui->supply->setEnabled(false);
    ui->inlet->setEnabled(false);
    ui->rpm->setEnabled(false);
    ui->prpm->setEnabled(false);
    ui->timemeter->setEnabled(false);
    ui->ptimemeter->setEnabled(false);
    ui->rfault->setEnabled(false);
    ui->fault->setEnabled(false);
    ui->pst->setEnabled(false);
    ui->ppst->setEnabled(false);
    ui->mt->setEnabled(false);
    ui->mdt->setEnabled(false);
    ui->pmdt->setEnabled(false);
    ui->pmt->setEnabled(false);
    //sub_control = nh.subscribe<flor_control_msgs::FlorRobotStateCommand>("/flor/controller/robot_state_command", 5, &Widget::controlstate, this);
    pub = nh.advertise<flor_control_msgs::FlorRobotStateCommand> ("/flor/controller/robot_state_command",5,false);
    sub_state = nh.subscribe<flor_control_msgs::FlorRobotStatus>("/flor/controller/robot_status", 5, &Widget::robotstate, this);
    sub_behav = nh.subscribe<atlas_msgs::AtlasSimInterfaceState>("/atlas/atlas_sim_interface_state", 5, &Widget::behavstate, this);
    sub_fault = nh.subscribe<flor_control_msgs::FlorRobotFault >("/flor/controller/robot_fault", 5, &Widget::robotfault, this);
    //status_msg_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>( "/flor_robot_sinttatus", 100, &robotStatus::recievedMessage, this );
    timer.start(1, this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}


void Widget::on_connect_clicked()
{
    if (ui->connect->text()=="CONNECT")
    {ui->connect->setText("CONNECT");
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
    }
}

/*void Widget::recievedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg)
{

    uint8_t  level;
    uint16_t code;
    RobotStatusCodes::codes(msg->status, code,level);























}*/
 void Widget:: robotstate( const flor_control_msgs::FlorRobotStatus::ConstPtr& msg )
 {
     // save the last status message
     last_run_state = msg->robot_run_state;
     if(last_inlet_pr==-1)
         last_inlet_pr=msg->pump_inlet_pressure;
     if(last_air_sump_pressure==-1)
    last_air_sump_pressure = msg->air_sump_pressure;
     if(last_pump_rpm==-1)
     last_pump_rpm=msg->current_pump_rpm;
     if(last_pump_return_pressure==-1)
     last_pump_return_pressure=msg->pump_return_pressure;
     if(last_pump_supply_pressure==-1)float last_pt;
     float last_mt;
     float last_mdt;
     last_pump_supply_pressure=msg->pump_supply_pressure;
     if(last_pump_time_meter==-1)
     last_pump_time_meter=msg->pump_time_meter;
     if(last_pt==-1)
         last_pt = msg->pump_supply_temperature;
     if(last_mt==-1)
         last_mt = msg->motor_temperature;
     if (last_mdt==-1)
         last_mdt= msg->motor_driver_temperature;
     switch(msg->robot_run_state)
     {
     case 0:ui->r_state->setText("IDLE");break;
     case 1:ui->r_state->setText("START");break;
     case 3:ui->r_state->setText("CONTROL");break;
     case 5:ui->r_state->setText("STOP");break;
     }
 float pumpinlet = 0.1*msg->pump_inlet_pressure+0.9*last_inlet_pr;
 float pst = 0.1*msg->pump_supply_temperature+0.9*last_pt;
 float mt = 0.1*msg->motor_temperature+0.9*last_mt;
 float mdt = 0.1*msg->motor_driver_temperature+0.9*last_mdt;
 ui->pst->setText(QString::number(pst,'f',2));
 ui->mt->setText(QString::number(mt,'f',2));
 ui->mdt->setText(QString::number(mdt,'f',2));
 if (pst>94)
     ui->ppst->setStyleSheet("background-color: red");
 else if(89<pst<94)
     ui->ppst->setStyleSheet("background-color: yellow");
 else
     if (pst <89)
         ui->ppst->setStyleSheet("background-color: grey");

 if (mt>149)
     ui->pmt->setStyleSheet("background-color: red");
 else if(124<mt<149)
     ui->pmt->setStyleSheet("background-color: yellow");
 else
     if (mt <124)
         ui->pmt->setStyleSheet("background-color: grey");

 if (mdt>59)
     ui->pmdt->setStyleSheet("background-color: red");
 else if(54<mdt<59)
     ui->pmdt->setStyleSheet("background-color: yellow");
 else
     if (mdt <54)
         ui->pmdt->setStyleSheet("background-color: grey");




 // code to detect fault in inlet pressure

 if (pumpinlet<50)
     ui->pinlet->setStyleSheet("background-color: red");
 else if(50<pumpinlet<70)
     ui->pinlet->setStyleSheet("background-color: yellow");
 else
     if (pumpinlet >70)
         ui->pinlet->setStyleSheet("background-color: grey");

 ui->sump->setText(QString::number(0.1*msg->air_sump_pressure+0.9*last_air_sump_pressure,'f',2));
 float airsump= 0.1*msg->air_sump_pressure+0.9*last_air_sump_pressure;
 if (airsump<50)
     ui->psump->setStyleSheet("background-color: red");
 else if(50<airsump<70)
     ui->psump->setStyleSheet("background-color: yellow");
 else
     if (airsump >70)
         ui->psump->setStyleSheet("background-color: grey");
 ui->inlet->setText(QString::number(pumpinlet,'f',2));
 ui->timemeter->setText(QString::number(0.1*msg->pump_time_meter+0.9*last_pump_time_meter,'f',2));
 ui->rpm->setText(QString::number(0.1*msg->current_pump_rpm+0.9*last_pump_rpm,'f',2));
 ui->return_2->setText(QString::number(0.1*msg->pump_return_pressure+0.9*last_pump_return_pressure,'f',2));
 float pumpreturn = 0.1*msg->pump_return_pressure+0.9*last_pump_return_pressure;
 if (pumpreturn<50)
     ui->preturn->setStyleSheet("background-color: red");
 else if(50<pumpreturn<70)
     ui->preturn->setStyleSheet("background-color: yellow");
 else
     if (pumpreturn >70)
         ui->preturn->setStyleSheet("background-color: grey");


 ui->supply->setText(QString::number(0.1*msg->pump_supply_pressure+0.9*last_pump_supply_pressure,'f',2));
 float pumpsupply= 0.1*msg->pump_supply_pressure+0.9*last_pump_supply_pressure;
 if (pumpsupply<1500)
     ui->psupply->setStyleSheet("background-color: red");
 else if(1500<pumpsupply<2700)
     ui->psupply->setStyleSheet("background-color: yellow");
 else
     if (pumpsupply >2700)
         ui->psupply->setStyleSheet("background-color: grey");

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
         ui->last_stat->setEnabled(true);
         ui->curst->setEnabled(true);
         ui->stat->setEnabled(true);
         ui->robo_st->setEnabled(true);
         ui->d_label->setEnabled(true);
         ui->d_state->setEnabled(true);
         ui->r_state->setEnabled(true);
         ui->send_mode->setEnabled(true);
         ui->pinlet->setEnabled(true);
         ui->psump->setEnabled(true);
         ui->psupply->setEnabled(true);
         ui->preturn->setEnabled(true);
         ui->pr->setEnabled(false);
         ui->off->setEnabled(false);
         ui->low->setEnabled(false);
         ui->high->setEnabled(false);
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
     last_inlet_pr = msg->pump_inlet_pressure;
     last_air_sump_pressure = msg->air_sump_pressure;
     last_pump_rpm=msg->current_pump_rpm;
     last_pump_return_pressure=msg->pump_return_pressure;
     last_pump_supply_pressure=msg->pump_supply_pressure;
     last_pump_time_meter=msg->pump_time_meter;

 }
 void Widget::robotfault(const flor_control_msgs::FlorRobotFault::ConstPtr& msg)
 {

     ui->fault->setText(msg->message.c_str());

 }

void Widget:: behavstate( const atlas_msgs::AtlasSimInterfaceState::ConstPtr& msg )
{
    switch(msg->current_behavior)
    {
    case 0: ui->cur_st->setText("STAND"); break;
    case 1: ui->cur_st->setText("USER"); break;
    case 2: ui->cur_st->setText("FREEZE"); break;
    case 3: ui->cur_st->setText("STAND_PREP"); break;
    case 4: ui->cur_st->setText("WALK"); break;
    case 5: ui->cur_st->setText("STEP"); break;
    case 6: ui->cur_st->setText("MANIPULATE"); break;
    }
    switch(msg->desired_behavior)
    {
    case 0: ui->d_state->setText("STAND"); break;
    case 1: ui->d_state->setText("USER"); break;
    case 2: ui->d_state->setText("FREEZE"); break;
    case 3: ui->d_state->setText("STAND_PREP"); break;
    case 4: ui->d_state->setText("WALK"); break;
    case 5: ui->d_state->setText("STEP"); break;
    case 6: ui->d_state->setText("MANIPULATE"); break;
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
        ui->cur_st->setEnabled(false);
        ui->last_stat->setEnabled(false);
        ui->curst->setEnabled(false);
        ui->stat->setEnabled(false);
        ui->robo_st->setEnabled(false);
        ui->d_label->setEnabled(false);
        ui->d_state->setEnabled(false);
        ui->r_state->setEnabled(false);
        ui->send_mode->setEnabled(false);
        //ui->start->setEnabled(true);
        ui->pinlet->setEnabled(false);
        ui->psump->setEnabled(false);
        ui->psupply->setEnabled(false);
        ui->preturn->setEnabled(false);
        ui->pr->setEnabled(false);
        ui->off->setEnabled(false);
        ui->low->setEnabled(false);
        ui->high->setEnabled(false);
        ui->pr->setEnabled(false);
        ui->high->setEnabled(false);
        ui->off->setEnabled(false);
        ui->low->setEnabled(false);
        ui->rpm->setEnabled(false);
        ui->prpm->setEnabled(false);
        ui->timemeter->setEnabled(false);
        ui->ptimemeter->setEnabled(false);
        ui->rfault->setEnabled(false);
        ui->fault->setEnabled(false);
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
}
