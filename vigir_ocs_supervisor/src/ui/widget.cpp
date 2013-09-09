#include "widget.h"
#include "ui_widget.h"
#include<string.h>
# include <QAbstractButton>
#include<QLabel>
#include<ros/ros.h>
#include <ros/package.h>



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
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
    ui->connect->setStyleSheet("background-color: green; color: black");
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
    
    //sub_control = nh.subscribe<flor_control_msgs::FlorRobotStateCommand>("/flor/controller/robot_state_command", 5, &Widget::controlstate, this);
    pub = nh.advertise<flor_control_msgs::FlorRobotStateCommand> ("/flor/controller/robot_state_command",5,false);
    sub_state = nh.subscribe<flor_control_msgs::FlorRobotStatus>("/flor/controller/robot_status", 5, &Widget::robotstate, this);
    sub_behav = nh.subscribe<atlas_msgs::AtlasSimInterfaceState>("/atlas/atlas_sim_interface_state", 5, &Widget::behavstate, this);

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
    {

        //publishing command "CONNECT"
        flor_control_msgs::FlorRobotStateCommand connect ;
        connect.state_command=flor_control_msgs::FlorRobotStateCommand::CONNECT;
        pub.publish(connect);
    }
    else
    {
        ui->connect->setText("CONNECT");
        ui->connect->setStyleSheet("background-color: green; color: black");
        ui->start->setStyleSheet("background-color: gray; color: black");
        ui->start->setEnabled(false);
        //publishing command "DISCONNECT"
        flor_control_msgs::FlorRobotStateCommand disconnect ;
        disconnect.state_command=flor_control_msgs::FlorRobotStateCommand::DISCONNECT;
        pub.publish(disconnect);
    }


}
 void Widget:: robotstate( const flor_control_msgs::FlorRobotStatus::ConstPtr& msg )
 {
     // save the last status message
     last_run_state = msg->robot_run_state;

     ui->r_state->setText(QString::number(msg->robot_run_state));
     ui->inlet->setText(QString::number(msg->pump_inlet_pressure));
     ui->sump->setText(QString::number(msg->air_sump_pressure));
     ui->return_2->setText(QString::number(msg->pump_return_pressure));
     ui->supply->setText(QString::number(msg->pump_supply_pressure));
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
     }

 }
void Widget:: behavstate( const atlas_msgs::AtlasSimInterfaceState::ConstPtr& msg )
{
    ui->cur_st->setText(QString::number(msg->current_behavior));
    ui->d_state->setText(QString::number(msg->desired_behavior));

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
