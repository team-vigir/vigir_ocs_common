#include <ros/package.h>

#include "bandwidth_widget.h"
#include "ui_bandwidth_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>

#define SECONDS_BETWEEN_UPDATES 5

BandwidthWidget::BandwidthWidget(QWidget *parent) :
    QWidget(parent)
    , ui(new Ui::BandwidthWidget)
{
    ui->setupUi(this);
    avgLatency = 0;
    bytes_remaining_initialized = false;
    numLatencyEntries = 0;
    // subscribe to the topic to monitor bandwidth usage
    drc_data_pub_ = nh_.advertise<flor_ocs_msgs::DRCdata>("/drc_data_sumary",false);
    ocs_bandwidth_sub_ = nh_.subscribe<flor_ocs_msgs::OCSBandwidth>( "/flor_ocs_bandwidth", 5, &BandwidthWidget::processBandwidthMessage, this );
    // subscribe to the topic to load all waypoints
    vrc_data_sub_ = nh_.subscribe<flor_ocs_msgs::DRCdata>( "/vrc_data", 5, &BandwidthWidget::processDRCData, this );
    timer.start(33, this);
    updateTimer.start(SECONDS_BETWEEN_UPDATES*1000,this);
    lowBWMode = false;
    last_max_bytes_up = last_max_bytes_down = 0;
    topic_heartbeat_sub_ = nh_.subscribe<std_msgs::String>("/flor_ocs_bandwidth/heartbeat",1, &BandwidthWidget::heartbeatRecieved, this);
    ui->tableWidget->setColumnWidth(0,  10);
    ui->tableWidget->setColumnWidth(1,  155);
    ui->tableWidget->setColumnWidth(2,  170);
    ui->tableWidget->setColumnWidth(3,  170);
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
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
    else if(event->timerId() == updateTimer.timerId())
        updateRateValues();
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
            block->setBackgroundColor(Qt::green);
            ui->tableWidget->setItem(timerNum,0,block);
        }
        else if (topicObjectsList[timerNum]->alarmsSinceReturn >2)
        {
            QTableWidgetItem* block = new QTableWidgetItem();
            block->setBackgroundColor(Qt::yellow);
            ui->tableWidget->setItem(timerNum,0,block);
        }
    }
}

void BandwidthWidget::updateRateValues()
{
    for(int index=0;index<node_bandwidth_info_.size();index++)
    {
        double average_down = (node_bandwidth_info_[index].total_bytes_read - node_bandwidth_info_[index].last_calculated_bytes_down)/SECONDS_BETWEEN_UPDATES;
        double average_up = (node_bandwidth_info_[index].total_bytes_sent - node_bandwidth_info_[index].last_calculated_bytes_up)/SECONDS_BETWEEN_UPDATES;
        node_bandwidth_info_[index].last_calculated_bytes_down = node_bandwidth_info_[index].total_bytes_read;
        node_bandwidth_info_[index].last_calculated_bytes_up = node_bandwidth_info_[index].total_bytes_sent;
    }
    double total_down = (last_max_bytes_down - ui->downTotalLabel->text().toDouble())/SECONDS_BETWEEN_UPDATES;
    double total_up = (last_max_bytes_up - ui->upTotalLabel->text().toDouble())/SECONDS_BETWEEN_UPDATES;
    ui->downTotalLabel->setText(QString::number(total_down));
    ui->upTotalLabel->setText(QString::number(total_up));
    flor_ocs_msgs::DRCdata msg;
    if(numLatencyEntries > 0)
    	msg.latency = avgLatency/numLatencyEntries;
    else
        msg.latency = 0;
    msg.downlink_bytes_average = total_down;
    msg.uplink_bytes_average = total_up;
    msg.message = ui->message->text().toStdString();
    drc_data_pub_.publish(msg);
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
        new_item.last_calculated_bytes_down = 0;
        new_item.last_calculated_bytes_up = 0;
        node_bandwidth_info_.push_back(new_item);

        //std::cout << "Adding new item.... rcv_index = " <<rcv_index<<" node bandwith info " << node_bandwidth_info_.size() << " name="<< msg->node_name <<std::endl;

        ui->tableWidget->setRowCount( node_bandwidth_info_.size());
        // create new items
        item = new QTableWidgetItem(QString(node_bandwidth_info_[rcv_index].node_name.c_str()));
        ui->tableWidget->setItem(rcv_index,1,item);
        item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_read));
        ui->tableWidget->setItem(rcv_index,2,item);
        item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent));
        ui->tableWidget->setItem(rcv_index,3,item);
        item = new QTableWidgetItem();
        item->setBackgroundColor(Qt::red);
        ui->tableWidget->setItem(rcv_index,0,item);

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
        /*item = ui->tableWidget->item(rcv_index,2);
        item->setText(QString::number(node_bandwidth_info_[rcv_index].total_bytes_read));
        item = ui->tableWidget->item(rcv_index,3);
        item->setText(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent));*/
    }

    unsigned long total_read = 0, total_sent = 0;
    for(int i = 0; i < node_bandwidth_info_.size(); i++)
    {
        total_read += node_bandwidth_info_[i].total_bytes_read;
        total_sent += node_bandwidth_info_[i].total_bytes_sent;
    }
    last_max_bytes_down = total_read;
    ui->upTotalLabel->setText(QString::number(total_sent));

//    total_bytes_sent_item = new QTableWidgetItem(QString::number(total_sent));
//    total_bytes_read_item = new QTableWidgetItem(QString::number(total_read));
//    QTableWidgetItem* name = new QTableWidgetItem(QString::fromStdString("TOTAL"));
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),0,name);
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),1,total_bytes_read_item);
//    ui->tableWidget->setItem(node_bandwidth_info_.size(),2,total_bytes_sent_item);

//calculating the pcercentage of the download/upload for each topic.
    if(total_read > 0 )
    {
        double temp = ((double)(node_bandwidth_info_[rcv_index].total_bytes_read)/(double)total_read)*100;
        QString percent = QString::number(node_bandwidth_info_[rcv_index].total_bytes_read);
        percent.append(QString::fromStdString(" (")).append(QString::number(temp,'g',3)).append(QString::fromStdString(" %)"));
        item = new QTableWidgetItem(percent);
        ui->tableWidget->setItem(rcv_index,2,item);
    }
    if(total_sent > 0 )
    {
        double temp = ((double)(node_bandwidth_info_[rcv_index].total_bytes_sent)/(double)total_sent)*100 ;
        QString percent = QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent);
        percent.append(QString::fromStdString(" (")).append(QString::number(temp,'g',3)).append(QString::fromStdString(" %)"));
        item = new QTableWidgetItem(percent);
        ui->tableWidget->setItem(rcv_index,3,item);
    }
    //ui->tableWidget->setItem(rcv_index,4,item);
    //item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent/total_sent));
    //ui->tableWidget->setItem(rcv_index,3,item);
    if(starting_row_count < 2)
    {
        ui->tableWidget->setColumnWidth(0,  10);
        ui->tableWidget->setColumnWidth(1,  155);
        ui->tableWidget->setColumnWidth(2,  170);
        ui->tableWidget->setColumnWidth(3,  170);
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

void BandwidthWidget::processDRCData(const flor_ocs_msgs::DRCdata::ConstPtr& msg)
{
    //ui->competition_score->setText(QString::number(msg->competition_score));
    if(msg->latency > 0)
    {
        ui->latency->setText(QString::number(msg->latency));
        numLatencyEntries++;
        avgLatency += msg->latency;
        if(msg->latency >= 500 && lowBWMode)
        {
            modeStartTime = ros::Time::now();
            lowBWMode = false;
            avgLatency = msg->latency;
            numLatencyEntries = 1;
        }
        if(msg->latency < 500 && !lowBWMode)
        {
            modeStartTime = ros::Time::now();
            lowBWMode = true;
            avgLatency = msg->latency;
	    numLatencyEntries = 1;
        }
    }
    ui->message->setText(QString(msg->message.c_str()));
    if(!bytes_remaining_initialized && msg->downlink_bytes_average < UINT_MAX)
    {
        std::cout << "got max upload = " << msg->uplink_bytes_average << " max download = " <<msg->downlink_bytes_average <<std::endl;
        down_max = msg->downlink_bytes_average;
        up_max = msg->uplink_bytes_average;
        bytes_remaining_initialized = true;
    }
    else if (bytes_remaining_initialized)
    {
        float down = (msg->downlink_bytes_average / (double)down_max)*1000.0;
        float up = (msg->uplink_bytes_average/(double)up_max)*1000.0;
        std::cout << "new usage message recieved up = " << up << " down = " <<down <<  " dwn_raw = " << msg->downlink_bytes_average << std::endl;
        ui->down_remaining_bar->setValue((msg->downlink_bytes_average/(double)down_max)*1000.0);
        ui->up_remaining_bar->setValue((msg->uplink_bytes_average/(double)up_max)*1000.0);
        ui->upLabel->setText(QString::number(msg->uplink_bytes_average));
        ui->downLabel->setText(QString::number(msg->downlink_bytes_average));
    }
    //ui->remaining_download->setText(QString::number(msg->downlink_bytes_remaining));
    //ui->remaining_upload->setText(QString::number(msg->uplink_bytes_remaining));
}

