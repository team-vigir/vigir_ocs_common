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

void ImageManagerWidget::addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info)
{
    unsigned char image_data[1000*1000*3];
    int row = ui->tableWidget->rowCount();
    ui->tableWidget->setRowCount(row+1);
    ui->tableWidget->setRowHeight(row,100);

    // add info about images to the table
    QSignalMapper* signalMapper = new QSignalMapper(this);
    QTableWidgetItem * item;

    // image
    item = new QTableWidgetItem();

    ROS_ERROR("Encoding: %s", image.encoding.c_str());

    double aspect_ratio = (double)image.width/(double)image.height;
    ROS_ERROR("Size: %dx%d aspect %f", image.width, image.height, aspect_ratio);

    if(image.encoding == "rgb8")
    {
        ROS_ERROR("rgb");

        // hack for the vrc
        for(int i = 0; i < image.data.size(); i++)
            image_data[i] = image.data[i];
        QImage tmp(&image_data[0],image.width,image.height,QImage::Format_RGB888);
        QPixmap pixmap = QPixmap::fromImage(tmp).scaled((unsigned int)100, (unsigned int)100, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        item->setData(Qt::DecorationRole, pixmap);
    }
    else if(image.encoding == "mono8")
    {
        ROS_ERROR("mono");

        for(int i = 0, j = 0; i < image.data.size(); i++)
        {
            image_data[j++] = image.data[i];
            image_data[j++] = image.data[i];
            image_data[j++] = image.data[i];
        }
        QImage tmp(&image_data[0],image.width,image.height,QImage::Format_RGB888);
        QPixmap pixmap = QPixmap::fromImage(tmp).scaled((unsigned int)100, (unsigned int)100, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        item->setData(Qt::DecorationRole, pixmap);
    }

    item->setToolTip(QString::number(id));
    ui->tableWidget->setItem(row,0,item);

    ROS_ERROR("Added %ld to the table",id);

    // source
    item = new QTableWidgetItem(QString(topic.c_str()));
    ui->tableWidget->setItem(row,1,item);
    // width
    item = new QTableWidgetItem(QString::number(image.width));
    ui->tableWidget->setItem(row,2,item);
    // height
    item = new QTableWidgetItem(QString::number(image.height));
    ui->tableWidget->setItem(row,3,item);
}

void ImageManagerWidget::processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr &msg)
{
    addImage(msg->id,msg->topic,msg->image,msg->camera_info);
}

void ImageManagerWidget::processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr& msg)
{
    // reset table
    ui->tableWidget->clearContents();

    for(int i = 0; i < msg->image.size(); i++)
        addImage(msg->id[i],msg->topic[i],msg->image[i],msg->camera_info[i]);

    image_list_sub_.shutdown();
}

void ImageManagerWidget::imageClicked(int row, int column)
{
    std_msgs::UInt64 request;
    request.data = ui->tableWidget->item(row,0)->toolTip().toULong();
    ROS_ERROR("Clicked at %d %d    %s  %ld",row,column,ui->tableWidget->item(row,0)->toolTip().toStdString().c_str(),request.data);

    image_selected_pub_.publish(request);
}
