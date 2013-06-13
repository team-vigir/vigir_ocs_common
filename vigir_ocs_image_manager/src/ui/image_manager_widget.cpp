#include <ros/package.h>

#include "image_manager_widget.h"
#include "ui_image_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>



ImageManagerWidget::ImageManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageManagerWidget)
{    
    ui->setupUi(this);

    // initialize publishers for communication with
    image_list_request_pub_ = nh_.advertise<std_msgs::Bool>(   "/flor/ocs/image_history/list_request", 1, true );
    image_selected_pub_     = nh_.advertise<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 1, false );

    image_added_sub_ = nh_.subscribe<flor_ocs_msgs::OCSImageAdd>(  "/flor/ocs/image_history/add",  5, &ImageManagerWidget::processImageAdd,  this );;
    image_list_sub_  = nh_.subscribe<flor_ocs_msgs::OCSImageList>( "/flor/ocs/image_history/list", 5, &ImageManagerWidget::processImageList, this );

    //connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(editSlot(int, int)));
    std_msgs::Bool list_request;
    list_request.data = true;
    image_list_request_pub_.publish(list_request);

    timer.start(33, this);
}

ImageManagerWidget::~ImageManagerWidget()
{
    delete ui;
}

void ImageManagerWidget::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void ImageManagerWidget::processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr &msg)
{
    int row = ui->tableWidget->rowCount()+1;
    ui->tableWidget->setRowCount(row);

    // add info about images to the table
    QSignalMapper* signalMapper = new QSignalMapper(this);
    QTableWidgetItem * item;

    //std::cout << "Image: " << msg->image_list[i] << std::endl;

    /*float px,py,pz;
    px = msg->pose[i].pose.position.x;
    py = msg->pose[i].pose.position.y;
    pz = msg->pose[i].pose.position.z;
    float qx,qy,qz,qw;
    qw= msg->pose[i].pose.orientation.w;
    qx= msg->pose[i].pose.orientation.x;
    qy= msg->pose[i].pose.orientation.y;
    qz= msg->pose[i].pose.orientation.z;

    // remove   id   name   position   orientation   parentframe   grasp   confirmgrasp
    item = new QTableWidgetItem(QString("REMOVE"));
    item->setBackground(QBrush(QColor(200,200,200)));
    item->setForeground(QBrush(QColor(20,20,20)));
    item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
    ui->tableWidget->setItem(i,0,item);
    item = new QTableWidgetItem(QString::number(msg->image_id_list[i]));
    ui->tableWidget->setItem(i,1,item);
    item = new QTableWidgetItem(QString::fromUtf8(msg->image_list[i].substr(0,msg->image_list[i].size()-5).c_str()));
    ui->tableWidget->setItem(i,2,item);
    item = new QTableWidgetItem(QString::number(px)+", "+QString::number(py)+", "+QString::number(pz));
    ui->tableWidget->setItem(i,3,item);
    item = new QTableWidgetItem(QString::number(qx)+", "+QString::number(qy)+", "+QString::number(qz)+", "+QString::number(qw));
    ui->tableWidget->setItem(i,4,item);
    item = new QTableWidgetItem(QString::fromUtf8(msg->pose[i].header.frame_id.c_str()));
    ui->tableWidget->setItem(i,5,item);
    QComboBox *combo = new QComboBox();
    configureGrasps(msg->image_list[i].substr(0,msg->image_list[i].size()-5), combo);
    // TODO: need to create changed function to keep track of what's being visualized
    //connect(combo, SIGNAL(currentIndexChanged(int)), signalMapper, SLOT(map()));
    //signalMapper->setMapping(combo, QString("%1").arg(i));
    ui->tableWidget->setCellWidget(i, 6, combo);
    //ui->tableWidget->setItem(i,6,item);
    item = new QTableWidgetItem(QString("MATCH"));
    item->setBackground(QBrush(QColor(200,200,200)));
    item->setForeground(QBrush(QColor(20,20,20)));
    item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
    ui->tableWidget->setItem(i,7,item);
    item = new QTableWidgetItem(QString("GRASP"));
    item->setBackground(QBrush(QColor(200,200,200)));
    item->setForeground(QBrush(QColor(20,20,20)));
    item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEditable);
    ui->tableWidget->setItem(i,8,item);*/
}

void ImageManagerWidget::processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();

    for(int i = 0; i < msg->image.size(); i++)
    {
        flor_ocs_msgs::OCSImageAdd add_image;
        add_image.id = msg->id[i];
        add_image.topic = msg->topic[i];
        add_image.camera_info = msg->camera_info[i];
        add_image.image = msg->image[i];
        //processImageAdd(add_image);
    }

    image_list_sub_.shutdown();
}
/*
void ImageManagerWidget::configureGrasps(std::string image_name, QComboBox* combo_box)
{
    //std::cout << "looking for grasps for image " << image_name << std::endl;
    // add all grasp ids to the combo box
    for(int i = 0; i < grasp_db_.size(); i++)
    {
        if(image_name.compare(grasp_db_[i].image_name) == 0)
        {
            //std::cout << "  found " << (unsigned int)grasp_db_[i].grasp_id << std::endl;
            combo_box->addItem(QString::number((unsigned int)grasp_db_[i].grasp_id));
        }
    }
}

void ImageManagerWidget::removeImage(int id)
{
    flor_ocs_msgs::OCSImageRemove cmd;

    cmd.image_id = id;

    // publish image to be removed
    image_remove_pub_.publish( cmd );
}

void ImageManagerWidget::editSlot(int row, int col)
{
    if(col == 0)
    {
        removeImage(ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff);
    }
    else if(col == 7)
    {
        flor_grasp_msgs::ImageSelection cmd;

        cmd.image_id.data = ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff;
        std::string image_name = ui->tableWidget->item(row,2)->text().toUtf8().constData();

        for(int i = 0; i < grasp_db_.size(); i++)
        {
            if(grasp_db_[i].image_name.compare(image_name) == 0)
            {
                cmd.image_type.data = grasp_db_[i].image_type;
                break;
            }
        }
        cmd.pose.header.frame_id = "/world";

        // publish image to be matched
        image_match_request_pub_.publish( cmd );
    }
    else if(col == 8)
    {
        QComboBox* combo = (QComboBox*)ui->tableWidget->cellWidget(row, 6);
        unsigned short grasp_id = combo->itemText(combo->currentIndex()).toUInt() & 0x0000ffff;

        for(int i = 0; i < grasp_db_.size(); i++)
        {
            if(grasp_id == grasp_db_[i].grasp_id)
            {
                flor_grasp_msgs::GraspSelection cmd;

                cmd.grasp_id.data = grasp_id;
                cmd.image_id.data = ui->tableWidget->item(row,1)->text().toUInt() & 0x000000ff;
                cmd.image_type.data = grasp_db_[i].image_type;
                cmd.header.frame_id = "/world";

                // publish image to be matched
                grasp_request_pub_.publish( cmd );

                break;
            }
        }
    }
}*/
