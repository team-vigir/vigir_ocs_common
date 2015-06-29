#include <ros/package.h>

#include "video_editor_widget.h"
#include "ui_video_editor_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<QTreeWidget>
#include <QTreeWidgetItem>



VideoEditorWidget::VideoEditorWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VideoEditorWidget)
{    
    ui->setupUi(this);
    feed_rate= -1;
    feed_rate_prev = -1;
    ui->timeslider->setMinimum(0);
    ui->timeslider->setMaximum(0);
    imagecount=0;
    videocount=0;
    interval_count=0;
    video_start_time=0;
    subseq_video_time=0;
    video_start_time_nano=0;
    subseq_video_time_nano=0;

    // initialize publishers for communication with nodelet
    image_list_request_pub_ = nh_.advertise<std_msgs::Bool>(   "/flor/ocs/image_history/list_request", 1, true );
    image_selected_pub_     = nh_.advertise<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 1, false );

    image_added_sub_    = nh_.subscribe(  "/flor/ocs/image_history/add",  5, &VideoEditorWidget::processImageAdd,  this );
    image_list_sub_     = nh_.subscribe( "/flor/ocs/image_history/list", 100, &VideoEditorWidget::processImageList, this );

    // uncomment later
     image_selected_sub_ = nh_.subscribe( "/flor/ocs/history/image_raw", 5, &VideoEditorWidget::processSelectedImage, this );

    // sunscribers for video stream

    img_req_sub_full_l_ = nh_.subscribe( "/l_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_full_r_ = nh_.subscribe( "/r_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_full_lhl_ = nh_.subscribe( "/lhl_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_full_lhr_ = nh_.subscribe( "/lhr_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_full_rhl_ = nh_.subscribe( "/rhl_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_full_rhr_ = nh_.subscribe( "/rhr_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );

    img_req_sub_crop_l_ = nh_.subscribe( "/l_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_crop_r_ = nh_.subscribe( "/r_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_crop_lhl_ = nh_.subscribe( "/lhl_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_crop_lhr_ = nh_.subscribe( "/lhr_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_crop_rhl_ = nh_.subscribe( "/rhl_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    img_req_sub_crop_rhr_ = nh_.subscribe( "/rhr_image_full/image_request", 1, &VideoEditorWidget::processvideoimage, this );
    //connect(ui->treeWidget, SIGNAL(cellClicked(int,int)), this, SLOT(editSlot(int, int)));
    std_msgs::Bool list_request;
    list_request.data = true;
    image_list_request_pub_.publish(list_request);

    timer.start(33, this);

}

VideoEditorWidget::~VideoEditorWidget()
{
    delete ui;
}

void VideoEditorWidget::timerEvent(QTimerEvent *event)
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

void VideoEditorWidget::addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info)
{
    /*int row = 0;
    ui->treeWidget->add;
    ui->treeWidget->setRowHeight(row,100);*/

    // add info about images to the table

    QTreeWidgetItem * item;

    // image
    item = new QTreeWidgetItem();

    //ROS_ERROR("Encoding: %s", image.encoding.c_str());

    double aspect_ratio = (double)image.width/(double)image.height;
    //ROS_ERROR("Size: %dx%d aspect %f", image.width, image.height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
       // ROS_ERROR("cv_bridge exception: %s", e.what());
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
    item->setData(0,Qt::DecorationRole, pixmap);

    item->setToolTip(0,QString::number(id));
    //ui->treeWidget->setItem(row,0,item);

   // ROS_ERROR("Added %ld to the table",id);

    // stamp
    //item = new QTreeWidgetItem(timeFromMsg(image.header.stamp));
    item->setText(1,timeFromMsg(image.header.stamp));
    // source
    //item = new QTreeWidgeexttItem(QString(topic.c_str()));
    item->setText(2,QString(topic.c_str()));
    // width
   // item = new QTreeWidgetItem(QString::number(image.width));
    item->setText(3,QString::number(image.width));
    // height
   // item = new QTreeWidgetItem(QString::number(image.height));
    item->setText(4,QString::number(image.height));
    ui->treeWidget->addTopLevelItem(item);
}

void VideoEditorWidget::addImageChild(QTreeWidgetItem *parent, const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info)
{
    /*int row = 0;
    ui->treeWidget->add;
    ui->treeWidget->setRowHeight(row,100);*/

    // add info about images to the table

    QTreeWidgetItem * item;

    // image
    item = new QTreeWidgetItem();

    //ROS_ERROR("Encoding: %s", image.encoding.c_str());

    double aspect_ratio = (double)image.width/(double)image.height;
   // ROS_ERROR("Size: %dx%d aspect %f", image.width, image.height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
       // ROS_ERROR("cv_bridge exception: %s", e.what());
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
    item->setData(0,Qt::DecorationRole, pixmap);

    item->setToolTip(0,QString::number(id));
    //ui->treeWidget->setItem(row,0,item);

    //ROS_ERROR("Added %ld to the table",id);

    // stamp
    //item = new QTreeWidgetItem(timeFromMsg(image.header.stamp));
    item->setText(1,timeFromMsg(image.header.stamp));
    item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    // source
    //item = new QTreeWidgeexttItem(QString(topic.c_str()));
    item->setText(2,QString(topic.c_str()));
    // width
   // item = new QTreeWidgetItem(QString::number(image.width));
    item->setText(3,QString::number(image.width));
    // height
   // item = new QTreeWidgetItem(QString::number(image.height));
    item->setText(4,QString::number(image.height));
    parent->setText(1,timeFromMsg(image.header.stamp));
    parent->setText(2,QString(topic.c_str()));
    //parent->parent()->setText(1,timeFromMsg(image.header.stamp));
    parent->addChild(item);
}

QTreeWidgetItem* VideoEditorWidget::addvideoitem(int videocount,const sensor_msgs::Image& image)

{
    QTreeWidgetItem * item;
    item = new QTreeWidgetItem();
    item->setText(0,"Video"+QString::number(videocount));
    item->setText(1,timeFromMsg(image.header.stamp));
    item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    ui->treeWidget->insertTopLevelItem(0,item);
    return item;
}
QTreeWidgetItem* VideoEditorWidget::add_time_child(QTreeWidgetItem *pitem)

{
    QTreeWidgetItem * iteml;
    iteml = new QTreeWidgetItem();
    iteml->setText(0,QString::number(10*interval_count));
   // item->setText(1,timeFromMsg(image.header.stamp));
   // item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    pitem->addChild(iteml);
    return iteml;
}
QTreeWidgetItem* VideoEditorWidget::addimageitem(int imagecount,const sensor_msgs::Image& image)

{
    QTreeWidgetItem * item;
    item = new QTreeWidgetItem();
    item->setText(0,"Image"+QString::number(imagecount));
    item->setText(1,timeFromMsg(image.header.stamp));
    item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    ui->treeWidget->insertTopLevelItem(0,item);
    return item;
}
void VideoEditorWidget::processImageAdd(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{


    //ROS_ERROR("In Process Image Add%f",feed_rate);
    if(feed_rate==0.0f)
      {
        imagecount++;
        item = addimageitem(imagecount,msg->image);
        addImageChild(item,msg->id,msg->topic,msg->image,msg->camera_info);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate>0.0f)
    {
        subseq_video_time=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        qDebug()<<subseq_video_time;
        if(feed_rate!=feed_rate_prev)
        {
            videocount++;
            parentitem=addvideoitem(videocount,msg->image);
            interval_count=1;
            video_start_time=(int)msg->image.header.stamp.toSec();
            timeitem=add_time_child(parentitem);
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;

        }

        if((subseq_video_time-video_start_time)>10)
     {
         interval_count++;
         timeitem=add_time_child(parentitem);
         video_start_time=(int)msg->image.header.stamp.toSec();
     }
     addImageChild(timeitem,msg->id,msg->topic,msg->image,msg->camera_info);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev = feed_rate;

}
void VideoEditorWidget::processImageList(const vigir_ocs_msgs::OCSImageList::ConstPtr msg)
{
    // reset table
    //ui->treeWidget->clear();
    //ROS_ERROR("in process image list");




}

void VideoEditorWidget::processSelectedImage(const sensor_msgs::Image::ConstPtr msg)
{
    // image
    //ROS_ERROR("Encoding: %s", msg->encoding.c_str());

    double aspect_ratio = (double)msg->width/(double)msg->height;
   // ROS_ERROR("Size: %dx%d aspect %f", msg->width, msg->height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //ROS_ERROR("cv_bridge exception: %s", e.what());
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

QString VideoEditorWidget::timeFromMsg(const ros::Time& stamp)
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
void VideoEditorWidget::processvideoimage (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate = msg->publish_frequency;
    //ROS_ERROR("in process video image.feed rate = %f", feed_rate);

}






void VideoEditorWidget::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    if(item->parent()!=0)  // check if item clicked is parent or child
    {
        std_msgs::UInt64 request;
        request.data = item->toolTip(0).toULong();
       // ui->image_view->setText(item->toolTip(0));
        image_selected_pub_.publish(request);


    }
    else
        ui->image_view->setText("No Image Selected!!");

}




void VideoEditorWidget::on_timeslider_valueChanged(int value)
{
    //ui->timelabel->setText("Time(s):"+QString::number(value));
    ui->timelabel->setNum(value);

}
bool VideoEditorWidget::check_item_time(QTreeWidgetItem *item, int time)
{
    qDebug()<<"item time"<<item->toolTip(1);
    qDebug()<<"time label:"<<QString::number(time);
    if(item->toolTip(1).toInt()<=time)
       return true;
    else
       return false;


}
void VideoEditorWidget:: settree_show()
{
    for(int j=0;j<videocount+imagecount;j++)
    {
   QTreeWidgetItem *item = ui->treeWidget->topLevelItem(j);

   for (int i =0;i<item->childCount();i++)
        {
            QTreeWidgetItem *item_child = item->child(i);
            item_child->setHidden(false);
        }
    }

}
void VideoEditorWidget::on_pushButton_clicked()
{
    bool flag=0,ok;
    settree_show();
    int time = ui->timelabel->text().toInt(&ok,10);
    for(int j=0;j<videocount+imagecount;j++)
    {
   QTreeWidgetItem *item = ui->treeWidget->topLevelItem(j);

   for (int i =0;i<item->childCount();i++)
        {

            QTreeWidgetItem *item_child = item->child(i);
            QString list ;
            bool check_time =check_item_time(item_child,time);
            if (check_time==true)
            {
                item->setHidden(false);
                flag = 1;
                item_child->setHidden(false);
                list = "in if";
                qDebug()<<list<<"\n";

            }
            else
            {
                item_child->setHidden(true);
                if (flag==0)
                    item->setHidden(true);
                list = "in else";
                qDebug()<<list<<"\n";
            }


          //  list = item_child->toolTip(1)+item_child->toolTip(0);
          //  qDebug()<<list<<"\n";


        }
   flag = 0;
    }

   /* while (item->child(i)) {
               //if ((*it)->toolTip(1).toInt(&ok,10) <= time)

                   ui->image_view->setText(ui->image_view->text()+","+(*it)->toolTip(1));
                   //ui->image_view->setText(ui->image_view->text()+","+(*it)->toolTip(0));
                 //(*it)->setHidden(false);
              // else
                   //(*it)->setHidden(true);
                 //  ROS_ERROR("more");
           it++;

        }*/

}
