#include <ros/package.h>

#include "image_manager_widget.h"
#include "ui_image_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


ImageManagerWidget::ImageManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageManagerWidget)
{    
    ui->setupUi(this);

    // initialize publishers for communication with nodelet
    image_list_request_pub_ = nh_.advertise<std_msgs::Bool>(   "/flor/ocs/image_history/list_request", 1, true );
    image_selected_pub_     = nh_.advertise<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 1, false );

    image_added_sub_    = nh_.subscribe<flor_ocs_msgs::OCSImageAdd>(  "/flor/ocs/image_history/add",  5, &ImageManagerWidget::processImageAdd,  this );
    image_list_sub_     = nh_.subscribe<flor_ocs_msgs::OCSImageList>( "/flor/ocs/image_history/list", 100, &ImageManagerWidget::processImageList, this );
    image_selected_sub_ = nh_.subscribe<sensor_msgs::Image>( "/flor/ocs/history/image_raw", 5, &ImageManagerWidget::processSelectedImage, this );

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

QImage Mat2QImage(const cv::Mat3b &src)
{
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    for (int y = 0; y < src.rows; ++y)
    {
        const cv::Vec3b *srcrow = src[y];
        QRgb *destrow = (QRgb*)dest.scanLine(y);
        for (int x = 0; x < src.cols; ++x)
        {
            destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
        }
    }
    return dest;
}

void ImageManagerWidget::addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info)
{
    int row = 0;
    ui->tableWidget->insertRow(row);
    ui->tableWidget->setRowHeight(row,100);

    // add info about images to the table
    QTableWidgetItem * item;

    // image
    item = new QTableWidgetItem();

    ROS_ERROR("Encoding: %s", image.encoding.c_str());

    double aspect_ratio = (double)image.width/(double)image.height;
    ROS_ERROR("Size: %dx%d aspect %f", image.width, image.height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Size2i img_size;
    if(aspect_ratio > 1)
    {
        img_size.width = 100.0f;
        img_size.height = 100.0f/aspect_ratio;
    }
    else
    {
        img_size.width = 100.0f/aspect_ratio;
        img_size.height = 100.0f;
    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
    QImage tmp = Mat2QImage(cv_ptr->image);
    QPixmap pixmap = QPixmap::fromImage(tmp);
    item->setData(Qt::DecorationRole, pixmap);

    item->setToolTip(QString::number(id));
    ui->tableWidget->setItem(row,0,item);

    ROS_ERROR("Added %ld to the table",id);

    // stamp
    item = new QTableWidgetItem(timeFromMsg(image.header.stamp));
    ui->tableWidget->setItem(row,1,item);
    // source
    item = new QTableWidgetItem(QString(topic.c_str()));
    ui->tableWidget->setItem(row,2,item);
    // width
    item = new QTableWidgetItem(QString::number(image.width));
    ui->tableWidget->setItem(row,3,item);
    // height
    item = new QTableWidgetItem(QString::number(image.height));
    ui->tableWidget->setItem(row,4,item);
}

void ImageManagerWidget::processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr &msg)
{
    ROS_ERROR("process image add");
    addImage(msg->id,msg->topic,msg->image,msg->camera_info);
}

void ImageManagerWidget::processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr& msg)
{
    // reset table
    ROS_ERROR("process image list");
    ui->tableWidget->clearContents();

    for(int i = 0; i < msg->image.size(); i++)
        addImage(msg->id[i],msg->topic[i],msg->image[i],msg->camera_info[i]);

    image_list_sub_.shutdown();
}

void ImageManagerWidget::processSelectedImage(const sensor_msgs::Image::ConstPtr &msg)
{
    // image
    ROS_ERROR("Encoding: %s", msg->encoding.c_str());

    double aspect_ratio = (double)msg->width/(double)msg->height;
    ROS_ERROR("Size: %dx%d aspect %f", msg->width, msg->height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Size2i img_size;
    if(aspect_ratio > 1)
    {
        img_size.width = ui->image_view->width();
        img_size.height = ((float)ui->image_view->width())/aspect_ratio;
    }
    else
    {
        img_size.width = ((float)ui->image_view->height())/aspect_ratio;
        img_size.height = ui->image_view->height();
    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
    QImage tmp = Mat2QImage(cv_ptr->image);
    QPixmap pixmap = QPixmap::fromImage(tmp);
    ui->image_view->setPixmap(pixmap);
}

void ImageManagerWidget::imageClicked(int row, int column)
{
    std_msgs::UInt64 request;
    request.data = ui->tableWidget->item(row,0)->toolTip().toULong();
    //ROS_ERROR("Clicked at %d %d    %s  %ld",row,column,ui->tableWidget->item(row,0)->toolTip().toStdString().c_str(),request.data);

    image_selected_pub_.publish(request);
}

QString ImageManagerWidget::timeFromMsg(const ros::Time& stamp)
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
    return QString::fromStdString(stream.str());
}
