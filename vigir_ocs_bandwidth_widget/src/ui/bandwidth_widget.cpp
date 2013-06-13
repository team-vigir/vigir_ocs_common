#include <ros/package.h>

#include "bandwidth_widget.h"
#include "ui_bandwidth_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>



BandwidthWidget::BandwidthWidget(QWidget *parent) :
    QWidget(parent)
    , ui(new Ui::BandwidthWidget)
{
    ui->setupUi(this);
    bytes_remaining_initialized = false;


    // subscribe to the topic to monitor bandwidth usage
    ocs_bandwidth_sub_ = nh_.subscribe<flor_ocs_msgs::OCSBandwidth>( "/flor_ocs_bandwidth", 5, &BandwidthWidget::processBandwidthMessage, this );
    // subscribe to the topic to load all waypoints
    vrc_data_sub_ = nh_.subscribe<flor_ocs_msgs::VRCdata>( "/vrc_data", 5, &BandwidthWidget::processVRCData, this );
    timer.start(33, this);
    topic_heartbeat_sub_ = nh_.subscribe<std_msgs::String>("/flor_ocs_bandwidth/heartbeat",1, &BandwidthWidget::heartbeatRecieved, this);
    ui->tableWidget->setColumnWidth(0,  155);
    ui->tableWidget->setColumnWidth(1,  170);
    ui->tableWidget->setColumnWidth(2,  170);
    ui->tableWidget->setColumnWidth(3,  80);
}

BandwidthWidget::~BandwidthWidget()
{
    delete ui;
}

void BandwidthWidget::heartbeatRecieved(const std_msgs::String::ConstPtr& msg)
{
    int timerNum = 0;
    for(;timerNum < topicObjectsList.size();timerNum++)
    {
        if(ui->tableWidget->item(timerNum,0)->text().toStdString() == msg->data)
        {
            topicObjectsList[timerNum]->alarmsSinceReturn = 0;
            QTableWidgetItem* block = new QTableWidgetItem();
            block->setBackgroundColor(Qt::green);
            ui->tableWidget->setItem(timerNum,3,block);
            topicObjectsList[timerNum]->timer.start(1000,this);
        }
    }
}

void BandwidthWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
    else
    {
        int timerNum = 0;
        for(; timerNum< topicObjectsList.size();timerNum++)
        {
            if(topicObjectsList[timerNum]->timer.timerId() == event->timerId())
            {
                topicObjectsList[timerNum]->timer.start(1000,this);
                break;
            }
        }
        topicObjectsList[timerNum]->alarmsSinceReturn = topicObjectsList[timerNum]->alarmsSinceReturn + 1;
        if(topicObjectsList[timerNum]->alarmsSinceReturn > 4)
        {
            QTableWidgetItem* block = new QTableWidgetItem();
            block->setBackgroundColor(Qt::red);
            ui->tableWidget->setItem(timerNum,3,block);
        }
        else if (topicObjectsList[timerNum]->alarmsSinceReturn >2)
        {
            QTableWidgetItem* block = new QTableWidgetItem();
            block->setBackgroundColor(Qt::yellow);
            ui->tableWidget->setItem(timerNum,3,block);
        }
    }
}



void BandwidthWidget::processBandwidthMessage(const flor_ocs_msgs::OCSBandwidth::ConstPtr& msg)
{
    int starting_row_count = ui->tableWidget->rowCount();
    QTableWidgetItem* item;

    // get info for the node in the message
    int rcv_index = 0;
    for(; rcv_index < node_bandwidth_info_.size(); rcv_index++)
        if(node_bandwidth_info_[rcv_index].node_name.compare(msg->node_name) == 0)
            break;

    // in case we need a new entry
    if(rcv_index == node_bandwidth_info_.size())
    {
        BandwidthStruct new_item;
        new_item.node_name = msg->node_name;
        new_item.total_bytes_read = msg->bytes_read;
        new_item.total_bytes_sent = msg->bytes_sent;
        node_bandwidth_info_.push_back(new_item);

        //std::cout << "Adding new item.... rcv_index = " <<rcv_index<<" node bandwith info " << node_bandwidth_info_.size() << " name="<< msg->node_name <<std::endl;

        ui->tableWidget->setRowCount( node_bandwidth_info_.size());
        // create new items
        item = new QTableWidgetItem(QString(node_bandwidth_info_[rcv_index].node_name.c_str()));
        ui->tableWidget->setItem(rcv_index,0,item);
        item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_read));
        ui->tableWidget->setItem(rcv_index,1,item);
        item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent));
        ui->tableWidget->setItem(rcv_index,2,item);
        item = new QTableWidgetItem();
        item->setBackgroundColor(Qt::green);
        ui->tableWidget->setItem(rcv_index,3,item);

        topicObject* objTopic = new topicObject;
        objTopic->timer.start(1000,this);
        objTopic->alarmsSinceReturn = 0;

        topicObjectsList.push_back(objTopic);

    }
    else // if it already exists
    {
        node_bandwidth_info_[rcv_index].total_bytes_read = msg->bytes_read;
        node_bandwidth_info_[rcv_index].total_bytes_sent = msg->bytes_sent;

        // simply update the existing ones
        item = ui->tableWidget->item(rcv_index,1);
        item->setText(QString::number(node_bandwidth_info_[rcv_index].total_bytes_read));
        item = ui->tableWidget->item(rcv_index,2);
        item->setText(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent));
    }

    unsigned long total_read = 0, total_sent = 0;
    for(int i = 0; i < node_bandwidth_info_.size(); i++)
    {
        total_read += node_bandwidth_info_[i].total_bytes_read;
        total_sent += node_bandwidth_info_[i].total_bytes_sent;
    }
    ui->downTotalLabel->setText(QString::number(total_read));
    ui->upTotalLabel->setText(QString::number(total_sent));
//    total_bytes_sent_item = new QTableWidgetItem(QString::number(total_sent));
//    total_bytes_read_item = new QTableWidgetItem(QString::number(total_read));
//    QTableWidgetItem* name = new QTableWidgetItem(QString::fromStdString("TOTAL"));
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),0,name);
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),1,total_bytes_read_item);
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),2,total_bytes_sent_item);

    //double temp = (double)(node_bandwidth_info_[rcv_index].total_bytes_read)/(double)total_read;
    //item = new QTableWidgetItem(QString::number(temp));
    //ui->tableWidget->setItem(rcv_index,4,item);
    //item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent/total_sent));
    //ui->tableWidget->setItem(rcv_index,3,item);

    if(starting_row_count < 2)
    {
        ui->tableWidget->setColumnWidth(0,  155);
        ui->tableWidget->setColumnWidth(1,  170);
        ui->tableWidget->setColumnWidth(2,  170);
        ui->tableWidget->setColumnWidth(3,  80);
    }
}

QString timeFromMsg(const ros::Time& stamp)
{
    int sec = stamp.toSec();
    std::stringstream stream;

    stream.str("");
    int day = sec/86400;
    sec -= day * 86400;

    int hour = sec / 3600;
    sec -= hour * 3600;

    int min = sec / 60;
    sec -= min * 60;
    uint32_t nano = (stamp.toSec() - (int)stamp.toSec())*1000;
    stream << std::setw(2) << std::setfill('0') << day << " ";
    stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
     stream << std::setw(3) << std::setfill('0') << nano;
//    stream << std::setw(3) << std::setfill('0') << (stamp.toNSec()*(0.00001));
    //std::cout << "Nano = " << stamp.toNSec() <<  " full = " << stamp.toSec() << std::endl;
    return QString::fromStdString(stream.str());
}

void BandwidthWidget::processVRCData(const flor_ocs_msgs::VRCdata::ConstPtr& msg)
{
    ui->competition_score->setText(QString::number(msg->competition_score));
    ui->falls->setText(QString::number(msg->falls));
    ui->message->setText(QString(msg->message.c_str()));
    if(!bytes_remaining_initialized && msg->downlink_bytes_remaining < UINT_MAX)
    {
        down_max = msg->downlink_bytes_remaining;
        up_max = msg->downlink_bytes_remaining;
        bytes_remaining_initialized = true;
    }
    else if (bytes_remaining_initialized)
    {
        ui->down_remaining_bar->setValue((msg->downlink_bytes_remaining/down_max)*1000);
        ui->up_remaining_bar->setValue((msg->uplink_bytes_remaining/up_max)*1000);
    }
    //ui->remaining_download->setText(QString::number(msg->downlink_bytes_remaining));
    //ui->remaining_upload->setText(QString::number(msg->uplink_bytes_remaining));
}

