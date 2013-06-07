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

    // subscribe to the topic to monitor bandwidth usage
    ocs_bandwidth_sub_ = nh_.subscribe<flor_ocs_msgs::OCSBandwidth>( "/flor_ocs_bandwidth", 5, &BandwidthWidget::processBandwidthMessage, this );
    
    // subscribe to the topic to load all waypoints
    vrc_data_sub_ = nh_.subscribe<flor_ocs_msgs::VRCdata>( "/vrc_data", 5, &BandwidthWidget::processVRCData, this );
    
    timer.start(33, this);
    
    ui->tableWidget->setColumnWidth(0,  155);
    ui->tableWidget->setColumnWidth(1,  170);
    ui->tableWidget->setColumnWidth(2,  170);
}

BandwidthWidget::~BandwidthWidget()
{
    delete ui;
}

void BandwidthWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void BandwidthWidget::processBandwidthMessage(const flor_ocs_msgs::OCSBandwidth::ConstPtr& msg)
{    
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
    	
    	ui->tableWidget->setRowCount( node_bandwidth_info_.size() );
	
		// create new items
		item = new QTableWidgetItem(QString(node_bandwidth_info_[rcv_index].node_name.c_str()));
		ui->tableWidget->setItem(rcv_index,0,item);
		item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_read));
		ui->tableWidget->setItem(rcv_index,1,item);
		item = new QTableWidgetItem(QString::number(node_bandwidth_info_[rcv_index].total_bytes_sent));
		ui->tableWidget->setItem(rcv_index,2,item);
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
	
	ui->total_download->setText(QString::number(total_read));
	ui->total_upload->setText(QString::number(total_sent));
    
    ui->tableWidget->setColumnWidth(0,  155);
    ui->tableWidget->setColumnWidth(1,  170);
    ui->tableWidget->setColumnWidth(2,  170);
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
    ui->sim_time_elapsed->setText(timeFromMsg(msg->sim_time_elapsed));	
	ui->competition_score->setText(QString::number(msg->competition_score));
	ui->falls->setText(QString::number(msg->falls));
	ui->message->setText(QString(msg->message.c_str()));
	ui->task_type->setText(QString::number(msg->task_type));
	ui->remaining_download->setText(QString::number(msg->downlink_bytes_remaining));
	ui->remaining_upload->setText(QString::number(msg->uplink_bytes_remaining));    
}

