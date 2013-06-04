#include <ros/package.h>

#include "waypoint_manager_widget.h"
#include "ui_waypoint_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>



WaypointManagerWidget::WaypointManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WaypointManagerWidget)
{    
    ui->setupUi(this);

    // subscribe to the topic to load all waypoints
    waypoint_list_sub_ = nh_.subscribe<nav_msgs::Path>( "/waypoint/list", 5, &WaypointManagerWidget::processWaypointList, this );

    // and advertise the waypoint update to update the manipulator
    waypoint_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSWaypointRemove>( "/waypoint/remove", 1, false );

    // add a confirmation publisher for the button callback
    confirm_navigation_pub_ = nh_.advertise<flor_ocs_msgs::OCSWaypointUpdate>( "/waypoint/confirm", 1, false );

    timer.start(33, this);
}

WaypointManagerWidget::~WaypointManagerWidget()
{
    delete ui;
}

void WaypointManagerWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void WaypointManagerWidget::processWaypointList(const nav_msgs::Path::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();
    ui->tableWidget->setRowCount(msg->poses.size());

    // add info about waypoints to the table
    QSignalMapper* signalMapper = new QSignalMapper(this);
    QTableWidgetItem * item;

    for(int i = 0; i < msg->poses.size(); i++)
    {
        //std::cout << "waypoint: " << msg->waypoint_list[i] << std::endl;

        float px,py,pz;
        px = msg->poses[i].pose.position.x;
        py = msg->poses[i].pose.position.y;
        pz = msg->poses[i].pose.position.z;
        float qx,qy,qz,qw;
        qw= msg->poses[i].pose.orientation.w;
        qx= msg->poses[i].pose.orientation.x;
        qy= msg->poses[i].pose.orientation.y;
        qz= msg->poses[i].pose.orientation.z;

        // remove   position   orientation
        item = new QTableWidgetItem(QString("REMOVE"));
        item->setBackground(QBrush(QColor(200,200,200)));
        item->setForeground(QBrush(QColor(20,20,20)));
        item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
        ui->tableWidget->setItem(i,0,item);
        item = new QTableWidgetItem(QString::number(px)+", "+QString::number(py)+", "+QString::number(pz));
        ui->tableWidget->setItem(i,1,item);
        item = new QTableWidgetItem(QString::number(qx)+", "+QString::number(qy)+", "+QString::number(qz)+", "+QString::number(qw));
        ui->tableWidget->setItem(i,2,item);
    }
}

void WaypointManagerWidget::removeWaypoint(int id)
{
    flor_ocs_msgs::OCSWaypointRemove cmd;

    cmd.waypoint_id = id;

    // publish waypoint to be removed
    waypoint_remove_pub_.publish( cmd );
}

void WaypointManagerWidget::editSlot(int row, int col)
{
    if(col == 0)
    {
        removeWaypoint(row);
    }
}

void WaypointManagerWidget::confirmNavigation()
{
    flor_ocs_msgs::OCSWaypointUpdate cmd;

    confirm_navigation_pub_.publish( cmd );
}
