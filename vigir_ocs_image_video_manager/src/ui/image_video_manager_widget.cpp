#include <ros/package.h>

#include "image_video_manager_widget.h"
#include "ui_image_video_manager_widget.h"
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



ImageVideoManagerWidget::ImageVideoManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageVideoManagerWidget)
{
    ui->setupUi(this);
    flag_first_image=0;
    feed_rate_l=feed_rate_r=feed_rate_lhl=feed_rate_lhr=feed_rate_rhl=feed_rate_rhr= -1;
    feed_rate_prev_l=feed_rate_prev_r=feed_rate_prev_lhl=feed_rate_prev_lhr=feed_rate_prev_rhl=feed_rate_prev_rhr = -1;
    ui->timeslider->setMinimum(0);
    ui->timeslider->setMaximum(0);
    imagecount_l=imagecount_r=imagecount_lhl=imagecount_lhr=imagecount_rhl=imagecount_rhr=0;
    videocount_l=videocount_lhl=videocount_lhr=videocount_r=videocount_rhr=videocount_rhl=0;
    interval_count_l=interval_count_lhl=interval_count_lhr=interval_count_r=interval_count_rhr=interval_count_rhl=0;
    video_start_time_l=video_start_time_r=video_start_time_lhr=video_start_time_lhl=video_start_time_rhr=video_start_time_rhl=0;
    subseq_video_time_l=subseq_video_time_r=subseq_video_time_lhr=subseq_video_time_lhl=subseq_video_time_rhr=subseq_video_time_rhl=0;


    // initialize publishers for communication with nodelet
    image_list_request_pub_ = nh_.advertise<std_msgs::Bool>(   "/flor/ocs/image_history/list_request", 1, true );
    image_selected_pub_     = nh_.advertise<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 1, false );

    image_added_sub_    = nh_.subscribe(  "/flor/ocs/image_history/add",  5, &ImageVideoManagerWidget::processImageAdd,  this );
    image_list_sub_     = nh_.subscribe( "/flor/ocs/image_history/list", 100, &ImageVideoManagerWidget::processImageList, this );

    // uncomment later
     image_selected_sub_ = nh_.subscribe( "/flor/ocs/history/image_raw", 5, &ImageVideoManagerWidget::processSelectedImage, this );

    // sunscribers for video stream

    img_req_sub_full_l_ = nh_.subscribe( "/l_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_l, this );
    img_req_sub_full_r_ = nh_.subscribe( "/r_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_r, this );
    img_req_sub_full_lhl_ = nh_.subscribe( "/lhl_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_lhl, this );
    img_req_sub_full_lhr_ = nh_.subscribe( "/lhr_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_lhr, this );
    img_req_sub_full_rhl_ = nh_.subscribe( "/rhl_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_rhl, this );
    img_req_sub_full_rhr_ = nh_.subscribe( "/rhr_image_full/image_request", 1, &ImageVideoManagerWidget::processvideoimage_rhr, this );

    img_req_sub_crop_l_ = nh_.subscribe( "/l_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_l, this );
    img_req_sub_crop_r_ = nh_.subscribe( "/r_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_r, this );
    img_req_sub_crop_lhl_ = nh_.subscribe( "/lhl_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_lhl, this );
    img_req_sub_crop_lhr_ = nh_.subscribe( "/lhr_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_lhr, this );
    img_req_sub_crop_rhl_ = nh_.subscribe( "/rhl_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_rhl, this );
    img_req_sub_crop_rhr_ = nh_.subscribe( "/rhr_image_cropped/image_request", 1, &ImageVideoManagerWidget::processvideoimage_rhr, this );

    std_msgs::Bool list_request;
    list_request.data = true;
    image_list_request_pub_.publish(list_request);
    temptree = new QTreeWidget();
    treeWidget = new QTreeWidget();
    ui->stackedWidget->addWidget(treeWidget);
    ui->stackedWidget->addWidget(temptree);
    ui->stackedWidget->setCurrentWidget(treeWidget);
    treeWidget->setMaximumSize(520,550);
    treeWidget->setMinimumSize(520,550);
    treeWidget->setIndentation(10);
    treeWidget->setColumnCount(3);
    treeWidget->setColumnWidth(0,170);
    treeWidget->setColumnWidth(1,190);
    treeWidget->setColumnWidth(2,180);
    QStringList list;
    list<<"Image"<<"TimeStamp"<<"Source";
    treeWidget->setHeaderLabels(list);
    ui->timeslider->setEnabled(false);
    timer.start(33, this);
    connect(treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*,int)),this,SLOT(on_treeWidget_itemClicked(QTreeWidgetItem*,int)));
    connect (temptree,SIGNAL(itemClicked(QTreeWidgetItem*,int)),this,SLOT(on_treeWidget_itemClicked(QTreeWidgetItem*,int)));
   // timer1=timer2=0;
    //timerflag=0;




}

ImageVideoManagerWidget::~ImageVideoManagerWidget()
{
    delete ui;
}

void ImageVideoManagerWidget::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

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

void ImageVideoManagerWidget::addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info)
{
    /*int row = 0;
    treeWidget->add;
    treeWidget->setRowHeight(row,100);*/

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
        //img_size.width = 100.0f;
        //img_size.height = 100.0f/aspect_ratio;
        img_size.width = 50.0f;
        img_size.height = 50.0f;
       // img_size.height = 50.0f/aspect_ratio; uncomment on adding open cv code
    }
    else
    {
        img_size.width = 50.0f;
        //img_size.width = 50.0f/aspect_ratio; uncomment on adding opencv code
        img_size.height = 50.0f;
    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
    QImage tmp = Mat2QImage(cv_ptr->image);
    QPixmap pixmap = QPixmap::fromImage(tmp);
    item->setData(0,Qt::DecorationRole, pixmap);

    item->setToolTip(0,QString::number(id));
    //treeWidget->setItem(row,0,item);

   // ROS_ERROR("Added %ld to the table",id);

    // stamp
    //item = new QTreeWidgetItem(timeFromMsg(image.header.stamp));
    item->setText(1,timeFromMsg(image.header.stamp));
    // source image_selected_pub_
    //item = new QTreeWidgeexttItem(QString(topic.c_str()));
    item->setText(2,QString(topic.c_str()));
    // width
   // item = new QTreeWidgetItem(QString::number(image.width));
    item->setText(3,QString::number(image.width));
    // height
   // item = new QTreeWidgetItem(QString::number(image.height));
    item->setText(4,QString::number(image.height));
    treeWidget->addTopLevelItem(item);
}
void ImageVideoManagerWidget::thumbnail(const sensor_msgs::Image& image,QTreeWidgetItem *item)
{


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
        img_size.width = 50.0f;
        img_size.height = 50.0f/aspect_ratio;
    }
    else
    {
        img_size.width = 50.0f/aspect_ratio;
        img_size.height = 50.0f;
    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
    QImage tmp = Mat2QImage(cv_ptr->image);
    QPixmap pixmap = QPixmap::fromImage(tmp);
    item->setData(0,Qt::DecorationRole, pixmap);


}
void ImageVideoManagerWidget::addImageChild(QTreeWidgetItem *parent, const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info,int flag)
// flag to distinguish between image and video
{
    /*int row = 0;
    treeWidget->add;
    treeWidget->setRowHeight(row,100);*/

    // add info about images to the table
    QString cam_name;
    QString t2;
    if(QString(topic.c_str())=="/l_image_full" || QString(topic.c_str())=="/l_image_cropped")
        cam_name = "Head Camera Left";
   if(QString(topic.c_str())=="/r_image_full" || QString(topic.c_str())=="/r_image_cropped")
        cam_name = "Head Camera Right";
    if(QString(topic.c_str())=="/lhl_image_full" || QString(topic.c_str())=="/lhl_image_cropped")
        cam_name = "Left Hand Camera Left";
    if(QString(topic.c_str())=="/lhr_image_full" || QString(topic.c_str())=="/lhr_image_cropped")
        cam_name = "Left Hand Camera Right";
    if(QString(topic.c_str())=="/rhl_image_full" || QString(topic.c_str())=="/rhl_image_cropped")
       cam_name = "Right Hand Camera Left";
    if(QString(topic.c_str())=="/rhr_image_full" || QString(topic.c_str())=="/rhr_image_cropped")
        cam_name = "Right Hand Camera Right";
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
        img_size.width = 50.0f;
        img_size.height = 50.0f/aspect_ratio;
    }
    else
    {
        img_size.width = 50.0f/aspect_ratio;
        img_size.height = 50.0f;

    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
    QImage tmp = Mat2QImage(cv_ptr->image);
    QPixmap pixmap = QPixmap::fromImage(tmp);



    item->setData(0,Qt::DecorationRole, pixmap);

    item->setToolTip(0,QString::number(id));
    // stamp

    item->setText(1,timeFromMsg(image.header.stamp));
    item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    /*if(flag_first_image==0)
    {
        ui->timeslider->setMinimum((int)image.header.stamp.toSec());
        flag_first_image=1;}
    // source
*/
    item->setText(2,cam_name);
    item->setToolTip(2,QString(topic.c_str()));

    item->setText(3,QString::number(image.width));

    item->setText(4,QString::number(image.height));
    if(flag==0)
    {
       parent=item;
       treeWidget->insertTopLevelItem(0,item);
    }
    else
    {

    parent->addChild(item);
    while(parent)
      {

        t2=timeFromMsg(image.header.stamp);
        parent->setText(1,(parent->text(1)).left((parent->text(1)).indexOf("-",0))+"-"+t2);
        parent->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
        parent->setText(2,cam_name);
        parent->setToolTip(2,QString(topic.c_str()));
        thumbnail(image,parent);
        parent= parent->parent();
      }
    }

}
QTreeWidgetItem* ImageVideoManagerWidget::addvideoitem(int videocount,const sensor_msgs::Image& image)

{
    QTreeWidgetItem * item;
    item = new QTreeWidgetItem();
    //item->setText(0,"Video"+QString::number(videocount));
    item->setText(1,timeFromMsg(image.header.stamp));
    item->setToolTip(1,QString::number((int)image.header.stamp.toSec()));
    treeWidget->insertTopLevelItem(0,item);
    return item;
}
QTreeWidgetItem* ImageVideoManagerWidget::add_time_child(QTreeWidgetItem *pitem,int count_n)

{
    QTreeWidgetItem * iteml;
    iteml = new QTreeWidgetItem();
    //iteml->setText(0,QString::number(count_n));
   // item->setText(1,timeFromMsg(image.header.stamp));
   // item->setToolTip(1,QString::number((int)image.header.stamif(timerflag==0)

    pitem->addChild(iteml);
    return iteml;
}

void ImageVideoManagerWidget::processImageAdd(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{

    //ROS_ERROR(msg->topic.c_str());

   if(QString(msg->topic.c_str())=="/l_image_full" || QString(msg->topic.c_str())=="/l_image_cropped")
        imageaddfunction_l(msg);if ((subseq_video_time_l-childtimeitem_start_time_l)>60)

   if(QString(msg->topic.c_str())=="/r_image_full" || QString(msg->topic.c_str())=="/r_image_cropped")
        imageaddfunction_r(msg);
    if(QString(msg->topic.c_str())=="/lhl_image_full" || QString(msg->topic.c_str())=="/lhl_image_cropped")
        imageaddfunction_lhl(msg);
    if(QString(msg->topic.c_str())=="/lhr_image_full" || QString(msg->topic.c_str())=="/lhr_image_cropped")
        imageaddfunction_lhr(msg);
    if(QString(msg->topic.c_str())=="/rhl_image_full" || QString(msg->topic.c_str())=="/rhl_image_cropped")
        imageaddfunction_rhl(msg);
    if(QString(msg->topic.c_str())=="/rhr_image_full" || QString(msg->topic.c_str())=="/rhr_image_cropped")
        imageaddfunction_rhr(msg);


 }
void ImageVideoManagerWidget:: imageaddfunction_l(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{


    if(feed_rate_l==0.0f)
      {
        imagecount_l++;
      //  item_l = hiitem(imagecount_l,msg->image);
        addImageChild(item_l,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_l>0.0f)
    {   //ROS_ERROR("in l image");
        subseq_video_time_l=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        //qDebug()<<subseq_video_time_l;
        if(feed_rate_l!=feed_rate_prev_l)
        {

            videocount_l++;
            parentitem_l=addvideoitem(videocount_l,msg->image);
            interval_count_l=1;
            video_start_time_l=(int)msg->image.header.stamp.toSec();
            timeitem_l=add_time_child(parentitem_l,5);
            childtimeitem_l = add_time_child(timeitem_l,1);
            childtimeitem_start_time_l = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            parentitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
        }

        if((subseq_video_time_l-video_start_time_l)>300)
     {
         interval_count_l++;
         timeitem_l=add_time_child(parentitem_l,5);
         video_start_time_l=(int)msg->image.header.stamp.toSec();
         childtimeitem_l = add_time_child(timeitem_l,1);
         childtimeitem_start_time_l = (int)msg->image.header.stamp.toSec();
         timeitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
        if((subseq_video_time_l-childtimeitem_start_time_l)>60)
     {

            childtimeitem_l = add_time_child(timeitem_l,1);
            childtimeitem_start_time_l = (int)msg->image.header.stamp.toSec();
            childtimeitem_l->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }

     addImageChild(childtimeitem_l,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev_l = feed_rate_l;

}

void ImageVideoManagerWidget:: imageaddfunction_r(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{

    if(feed_rate_r==0.0f)
      {
        ROS_ERROR("in r image");
        imagecount_r++;
        //item_r = addimageitem(imagecount_r,msg->image);
        addImageChild(item_r,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_r>0.0f)
    {
        subseq_video_time_r=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        //qDebug()<<subseq_video_time_r;
        if(feed_rate_r!=feed_rate_prev_r)
        {
            videocount_r++;
            parentitem_r=addvideoitem(videocount_r,msg->image);
            interval_count_r=1;
            video_start_time_r=(int)msg->image.header.stamp.toSec();
            parentitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_r=add_time_child(parentitem_r,5);
            childtimeitem_r = add_time_child(timeitem_r,1);
            childtimeitem_start_time_r = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            timeitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
        }

        if((subseq_video_time_r-video_start_time_r)>300)
     {
         interval_count_r++;
         timeitem_r=add_time_child(parentitem_r,5);
         video_start_time_r=(int)msg->image.header.stamp.toSec();
         childtimeitem_r = add_time_child(timeitem_r,1);
         childtimeitem_start_time_r = (int)msg->image.header.stamp.toSec();
         timeitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
        if((subseq_video_time_r-childtimeitem_start_time_r)>60)
     {

            childtimeitem_r = add_time_child(timeitem_r,1);
            childtimeitem_start_time_r = (int)msg->image.header.stamp.toSec();
            childtimeitem_r->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
     addImageChild(childtimeitem_r,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev_r = feed_rate_r;

}
void ImageVideoManagerWidget:: imageaddfunction_lhl(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{
    ROS_ERROR("in lhl image");
    if(feed_rate_lhl==0.0f)
      {
        imagecount_lhl++;
        //item_lhl = addimageitem(imagecount_lhl,msg->image);
        addImageChild(item_lhl,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_lhl>0.0f)
    {
        subseq_video_time_lhl=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        qDebug()<<subseq_video_time_lhl;
        if(feed_rate_lhl!=feed_rate_prev_lhl)
        {
            videocount_lhl++;
            parentitem_lhl=addvideoitem(videocount_lhl,msg->image);
            interval_count_lhl=1;
            video_start_time_lhl=(int)msg->image.header.stamp.toSec();
            timeitem_lhl=add_time_child(parentitem_lhl,5);
            childtimeitem_lhl = add_time_child(timeitem_lhl,1);
            childtimeitem_start_time_lhl = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            parentitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
        }

        if((subseq_video_time_lhl-video_start_time_lhl)>300)
     {
         interval_count_lhl++;
         timeitem_lhl=add_time_child(parentitem_lhl,5);
         video_start_time_lhl=(int)msg->image.header.stamp.toSec();
         childtimeitem_lhl = add_time_child(timeitem_lhl,1);
         childtimeitem_start_time_lhl = (int)msg->image.header.stamp.toSec();
         timeitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
     }
        if((subseq_video_time_lhl-childtimeitem_start_time_lhl)>60)
     {

            childtimeitem_lhl = add_time_child(timeitem_lhl,1);
            childtimeitem_start_time_lhl = (int)msg->image.header.stamp.toSec();
            childtimeitem_lhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
     }
     addImageChild(childtimeitem_lhl,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev_lhl = feed_rate_lhl;

}
void ImageVideoManagerWidget:: imageaddfunction_lhr(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{
    ROS_ERROR("in lhr image");
    if(feed_rate_lhr==0.0f)
      {
        imagecount_lhr++;
       // item_lhr = addimageitem(imagecount_lhr,msg->image);
        addImageChild(item_lhr,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_lhr>0.0f)
    {
        subseq_video_time_lhr=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        qDebug()<<subseq_video_time_lhr;
        if(feed_rate_lhr!=feed_rate_prev_lhr)
        {
            videocount_lhr++;
            parentitem_lhr=addvideoitem(videocount_lhr,msg->image);
            interval_count_lhr=1;
            video_start_time_lhr=(int)msg->image.header.stamp.toSec();
            timeitem_lhr=add_time_child(parentitem_lhr,5);
            childtimeitem_lhr = add_time_child(timeitem_lhr,1);
            childtimeitem_start_time_lhr = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            parentitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

        }

        if((subseq_video_time_lhr-video_start_time_lhr)>300)
     {
         interval_count_lhr++;
         timeitem_lhr=add_time_child(parentitem_lhr,5);
         video_start_time_lhr=(int)msg->image.header.stamp.toSec();
         childtimeitem_lhr = add_time_child(timeitem_lhr,1);
         childtimeitem_start_time_lhr = (int)msg->image.header.stamp.toSec();
         timeitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
        if((subseq_video_time_lhr-childtimeitem_start_time_lhr)>60)
     {

            childtimeitem_lhr = add_time_child(timeitem_lhr,1);
            childtimeitem_start_time_lhr = (int)msg->image.header.stamp.toSec();
            childtimeitem_lhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
     addImageChild(childtimeitem_lhr,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev_lhr = feed_rate_lhr;

}
void ImageVideoManagerWidget:: imageaddfunction_rhr(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{
    ROS_ERROR("in rhr image");
    if(feed_rate_rhr==0.0f)
      {
        imagecount_rhr++;
        //item_rhr = addimageitem(imagecount_rhr,msg->image);
        addImageChild(item_rhr,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_rhr>0.0f)
    {
        subseq_video_time_rhr=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        qDebug()<<subseq_video_time_rhr;
        if(feed_rate_rhr!=feed_rate_prev_rhr)
        {
            videocount_rhr++;
            parentitem_rhr=addvideoitem(videocount_rhr,msg->image);
            interval_count_rhr=1;
            video_start_time_rhr=(int)msg->image.header.stamp.toSec();
            timeitem_rhr=add_time_child(parentitem_rhr,5);
            childtimeitem_rhr = add_time_child(timeitem_rhr,1);
            childtimeitem_start_time_rhr = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            parentitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

        }

        if((subseq_video_time_rhr-video_start_time_rhr)>300)
     {
         interval_count_rhr++;
         timeitem_rhr=add_time_child(parentitem_rhr,5);
         video_start_time_rhr=(int)msg->image.header.stamp.toSec();
         childtimeitem_rhr = add_time_child(timeitem_rhr,1);
         childtimeitem_start_time_rhr = (int)msg->image.header.stamp.toSec();
         timeitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
        if((subseq_video_time_rhr-childtimeitem_start_time_rhr)>60)
     {

            childtimeitem_rhr = add_time_child(timeitem_rhr,1);
            childtimeitem_start_time_rhr = (int)msg->image.header.stamp.toSec();
            childtimeitem_rhr->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
     addImageChild(childtimeitem_rhr,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((msg->image).header.stamp.toSec());

     }
       feed_rate_prev_rhr = feed_rate_rhr;

}
void ImageVideoManagerWidget:: imageaddfunction_rhl(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg)
{
   ROS_ERROR("in rhl image");
    if(feed_rate_rhl==0.0f)
      {
        imagecount_rhl++;
        //item_rhl = addimageitem(imagecount_rhl,msg->image);
        addImageChild(item_rhl,msg->id,msg->topic,msg->image,msg->camera_info,0);
        ui->timeslider->setMaximum((msg->image).header.stamp.toSec());// in seconds. nano seconds not counted
      }

    else
        if(feed_rate_rhl>0.0f)
    {
        subseq_video_time_rhl=(int)msg->image.header.stamp.toSec();
        //subseq_video_time_nano= ((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
        qDebug()<<subseq_video_time_rhl;
        if(feed_rate_rhl!=feed_rate_prev_rhl)
        {
            videocount_rhl++;
            parentitem_rhl=addvideoitem(videocount_rhl,msg->image);
            interval_count_rhl=1;
            video_start_time_rhl=(int)msg->image.header.stamp.toSec();
            timeitem_rhl=add_time_child(parentitem_rhl,5);
            childtimeitem_rhl = add_time_child(timeitem_rhl,1);
            childtimeitem_start_time_rhl = (int)msg->image.header.stamp.toSec();
            //video_start_time_nano=((msg->image).header.stamp.toSec() - (int)(msg->image).header.stamp.toSec())*1000;
            parentitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            timeitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
            childtimeitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

        }

        if((subseq_video_time_rhl-video_start_time_rhl)>300)
     {
         interval_count_rhl++;
         timeitem_rhl=add_time_child(parentitem_rhl,5);
         video_start_time_rhl=(int)msg->image.header.stamp.toSec();
         childtimeitem_rhl = add_time_child(timeitem_rhl,1);
         childtimeitem_start_time_rhl = (int)msg->image.header.stamp.toSec();
         timeitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");
         childtimeitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
        if((subseq_video_time_rhl-childtimeitem_start_time_rhl)>60)
     {

            childtimeitem_rhl = add_time_child(timeitem_rhl,1);
            childtimeitem_start_time_rhl = (int)msg->image.header.stamp.toSec();
            childtimeitem_rhl->setText(1,timeFromMsg(msg->image.header.stamp)+"-");

     }
     addImageChild(childtimeitem_rhl,msg->id,msg->topic,msg->image,msg->camera_info,1);
     ui->timeslider->setMaximum((int)(msg->image).header.stamp.toSec());

     }
       feed_rate_prev_rhl = feed_rate_rhl;

}

void ImageVideoManagerWidget::processImageList(const vigir_ocs_msgs::OCSImageList::ConstPtr msg)
{
    // reset table
    //treeWidget->clear();
    ROS_ERROR("in process image list");




}

void ImageVideoManagerWidget::processSelectedImage(const sensor_msgs::Image::ConstPtr msg)
{
    // image
    ROS_ERROR("Encoding: %s", msg->encoding.c_str());

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

QString ImageVideoManagerWidget::timeFromMsg(const ros::Time& stamp)
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
    //stream << std::setw(2) << std::setfill('0') << day << " ";
    //stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
    stream << std::setw(3) << std::setfill('0') << nano;
    return QString::fromStdString(stream.str());
}
void ImageVideoManagerWidget::processvideoimage_l (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_l = msg->publish_frequency;
    ROS_ERROR("in process video imagel.feed rate = %f", feed_rate_l);

}
void ImageVideoManagerWidget::processvideoimage_r (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_r = msg->publish_frequency;
    ROS_ERROR("in process video imager.feed rate = %f", feed_rate_r);

}
void ImageVideoManagerWidget::processvideoimage_lhl (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_lhl = msg->publish_frequency;
    ROS_ERROR("in process video imagelhl.feed rate = %f", feed_rate_lhl);

}
void ImageVideoManagerWidget::processvideoimage_lhr (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_lhr = msg->publish_frequency;
    ROS_ERROR("in process video imagelhr.feed rate = %f", feed_rate_lhr);

}
void ImageVideoManagerWidget::processvideoimage_rhl (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_rhl = msg->publish_frequency;
    ROS_ERROR("in process video imagerhl.feed rate = %f", feed_rate_rhl);

}
void ImageVideoManagerWidget::processvideoimage_rhr (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg)
{
    //if(msinsertTopLevelItemg->mode==vigir_perception_msgs::DownSampledImageRequest::PUBLISH_FREQ)
        feed_rate_rhr = msg->publish_frequency;
    ROS_ERROR("in process video imagerhr.feed rate = %f", feed_rate_rhr);

}





void ImageVideoManagerWidget::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    //if(item->parent()!=0)  // check if item clicked is parent or child
    {
        std_msgs::UInt64 request;
        request.data = item->toolTip(0).toULong();
       // ui->image_view->setText(item->toolTip(0));
        image_selected_pub_.publish(request);


    }
    //else
    //    ui->image_view->setText("No Image Selected!!");

}




void ImageVideoManagerWidget::on_timeslider_valueChanged(int value)
{
    //ui->timelabel->setText("Time(s):"+QString::number(value));
    int sec = value;
    std::stringstream stream;

    stream.str("");
    int day = sec/86400;
    sec -= day * 86400;

    int hour = sec / 3600;
    sec -= hour * 3600;

    int min = sec / 60;
    sec -= min * 60;
    //uint32_t nano = (stamp.toSec() - (int)stamp.toSec())*1000;
    //stream << std::setw(2) << std::setfill('0') << day << " ";
    //stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
    //stream << std::setw(3) << std::setfill('0') << nano;

    ui->timelabel->setText(QString::fromStdString(stream.str()));


    ui->stackedWidget->setCurrentWidget(temptree);
    temptree->setMaximumSize(520,550);
    temptree->setMinimumSize(520,550);
    temptree->setIndentation(10);
    temptree->setColumnCount(3);
    temptree->setColumnWidth(0,170);
    temptree->setColumnWidth(1,170);
    temptree->setColumnWidth(2,180);

    QStringList list;
    list<<"Image"<<"TimeStamp"<<"Source";
    temptree->setHeaderLabels(list);
    temptree->clear();

    //ui->slidertree->show();
   // ui->slidertree->clear();
    int time = value;
    qDebug()<<time;
    QTreeWidgetItem *item;

    int j=0;
    int flag=0,ok;
{
    //settree_show();->child(i)
    //settree_hide();
       // int time = ui->timelabel->text().toInt(&ok,10);
    for( j=0;j<treeWidget->topLevelItemCount();j++)
    {
        item = treeWidget->topLevelItem(j);
        qDebug()<<"here in loop 1";
        if(time<=item->toolTip(1).toInt())
        {
            flag = 1;
            search_child(time,flag,item);
            qDebug()<<"here in loop 1"<<item->toolTip(1)<<flag;
            //break;
        }
    }
    qDebug()<<"here out loop 1";
}
}
/*
    ui->stackedWidget->setCurrentWidget(temptree);
    temptree->setMaximumSize(520,550);
    temptree->setMinimumSize(520,550);
    temptree->setIndentation(10);
    temptree->setColumnCount(3);
    temptree->setColumnWidth(0,170);
    temptree->setColumnWidth(1,170);
    temptree->setColumnWidth(2,180);

    QStringList list;
    list<<"Image"<<"TimeStamp"<<"Source";
    temptree->setHeaderLabels(list);
    temptree->clear();

    //ui->slidertree->show();
   // ui->slidertree->clear();
    int time = ui->timeslider->value();
    qDebug()<<time;
    QTreeWidgetItem *item;

    int j=0;
    int flag=0,ok;
{
    //settree_show();->child(i)
    //settree_hide();
       // int time = ui->timelabel->text().toInt(&ok,10);
    for( j=0;j<treeWidget->topLevelItemCount();j++)
    {
        item = treeWidget->topLevelItem(j);
        qDebug()<<"here in loop 1";
        if(time<=item->toolTip(1).toInt())
        {
            flag = 1;
            search_child(time,flag,item);
            qDebug()<<"here in loop 1"<<item->toolTip(1)<<flag;
            //break;
        }
    }
    qDebug()<<"here out loop 1";
}


}*/
bool ImageVideoManagerWidget::check_item_time(QTreeWidgetItem *item, int time)
{
    qDebug()<<"item time"<<item->toolTip(1);
    qDebug()<<"time label:"<<QString::number(time);
    if(item->toolTip(1).toInt()==time)
       return true;
    else
       return false;


}
void ImageVideoManagerWidget:: settree_hide()
{
    /*for(int j=0;j<treeWidget->topLevelItemCount();j++)
    {
   treeWidget->topLevelItem(j)->setHidden(true);


    }*/
    treeWidget->hide();

}
void ImageVideoManagerWidget:: settree_show()
{
    for(int j=0;j<treeWidget->topLevelItemCount();j++)
    {
   QTreeWidgetItem *item = treeWidget->topLevelItem(j);

   for (int i =0;i<item->childCount();i++)
        {
            QTreeWidgetItem *item_child = item->child(i);
            item_child->setHidden(false);
        }
    }

}

void ImageVideoManagerWidget::on_cameralist_currentIndexChanged(int index)
{
    int toplevelitemcount;
    QString camera;
    QTreeWidgetItem * item;
    QTreeWidget *tree;
    if(ui->checkBox->checkState()==0)
        tree=treeWidget;
    else if (ui->checkBox->checkState()>0)
        tree=temptree;
    toplevelitemcount = tree->topLevelItemCount();
    qDebug()<<"index="<<index<<"\n";
    for(int k=0;k<toplevelitemcount;k++)
    {
        tree->topLevelItem(k)->setHidden(false);
    }
    switch(index)
    {
    case 0: camera="/l_image_full";break;
    case 1: camera = "/r_image_full"; break;
    case 2: camera = "/lhl_image_full"; break;
    case 3: camera = "/lhr_image_full"; break;
    case 4: camera = "/rhr_image_full";break;
    case 5: camera = "/rhl_image_full";break;
    }
    for (int i =0;i<toplevelitemcount;i++)
    {
        if(index==6)
            for(int k=0;k<toplevelitemcount;k++)
            {
                tree->topLevelItem(k)->setHidden(false);
            }
        else{

        if(tree->topLevelItem(i)->toolTip(2)!=camera)
        {
            item=tree->topLevelItem(i);
            item->setHidden(true);
           // ui->image_view->setText(treeWidget->topLevelItem(i)->text(2));

        }
            }
    }



}

void ImageVideoManagerWidget::search_child(int time,int flag, QTreeWidgetItem *item)
{
    int i,j;
    if (flag ==1)
    for( j=0;j<item->childCount();j++)

    {
        //qDebug()<<"here in loop 2"<<item->child(j)->toolTip(1);
        if(time<=item->child(j)->toolTip(1).toInt())

        {   qDebug()<<"here in loop 2"<<item->child(j)->toolTip(1);
            flag=2;
            item=item->child(j);
            break;
        }

    }
    qDebug()<<"here out loop 2"<<flag;
    if (flag ==2)
    for(int k=0;k<item->childCount();k++)
    {
        qDebug()<<"in loop 3"<<item->child(k)->toolTip(1);
        if(time<=item->child(k)->toolTip(1).toInt())
        {
        flag =3;
        qDebug()<<"here in loop 3"<<item->child(k)->toolTip(1);
        item=item->child(k);
        break;
        }
    }

    j=0;
    if (flag ==1)
    {

        if(time==item->toolTip(1).toInt())
        {
            qDebug()<<"here in loop 4"<<item->text(1);
            QTreeWidgetItem *tempitem = new QTreeWidgetItem();
            QVariant variant = item->data(0,Qt::DecorationRole);
            tempitem->setData(0,Qt::DecorationRole,variant);
            tempitem->setToolTip(0,item->child(i)->toolTip(0));
            tempitem->setToolTip(1,item->toolTip(1));
            tempitem->setToolTip(2,item->toolTip(2));
            tempitem->setText(1,item->text(1));
            tempitem->setText(2,item->text(2));
            tempitem->setText(3,item->text(3));
            tempitem->setText(4,item->text(4));
            temptree->insertTopLevelItem(0,tempitem);
        }
    }
    else
    for(int i=0;i<item->childCount();i++)
    {

       if(time==item->child(i)->toolTip(1).toInt())
       {
           //ui->image_view->setText(item->child(i)->text(1));           
        QTreeWidgetItem *tempitem = new QTreeWidgetItem();{{


            }


        }
        QVariant variant = item->child(i)->data(0,Qt::DecorationRole);
        tempitem->setData(0,Qt::DecorationRole,variant);
        tempitem->setToolTip(0,item->child(i)->toolTip(0));
        tempitem->setToolTip(1,item->child(i)->toolTip(1));
        tempitem->setToolTip(2,item->child(i)->toolTip(2));
        tempitem->setText(1,item->child(i)->text(1));
        tempitem->setText(2,item->child(i)->text(2));
        tempitem->setText(3,item->child(i)->text(3));
        tempitem->setText(4,item->child(i)->text(4));
        temptree->insertTopLevelItem(0,tempitem);
        //temptree->setHidden(false);
        qDebug()<<"here in loop 5"<<item->child(i)->toolTip(1);
       }
    }

    qDebug()<<"here out loop 5";

}


void ImageVideoManagerWidget::on_checkBox_clicked(bool checked)
{
    if(checked == true)

    {
        ui->timeslider->setEnabled(true);
        ui->stackedWidget->setCurrentWidget(temptree);
        temptree->setMaximumSize(520,550);
        temptree->setMinimumSize(520,550);
        temptree->setIndentation(10);
        temptree->setColumnCount(3);
        temptree->setColumnWidth(0,170);
        temptree->setColumnWidth(1,170);
        temptree->setColumnWidth(2,180);

        QStringList list;
        list<<"Image"<<"TimeStamp"<<"Source";
        temptree->setHeaderLabels(list);
        temptree->clear();

        //ui->slidertree->show();
       // ui->slidertree->clear();
        int time = ui->timeslider->maximum();
        qDebug()<<time;
        QTreeWidgetItem *item;

        int j=0;
        int flag=0,ok;
    {
        //settree_show();->child(i)
        //settree_hide();
           // int time = ui->timelabel->text().toInt(&ok,10);
        for( j=0;j<treeWidget->topLevelItemCount();j++)
        {
            item = treeWidget->topLevelItem(j);
            qDebug()<<"here in loop 1";
            if(time<=item->toolTip(1).toInt())
            {
                flag = 1;
                search_child(time,flag,item);
                qDebug()<<"here in loop 1"<<item->toolTip(1)<<flag;
                //break;
            }
        }
        qDebug()<<"here out loop 1";
    }
        ui->timeslider->setSliderPosition(ui->timeslider->maximum());
    }
    if (checked ==false)
    {
         ui->stackedWidget->setCurrentWidget(treeWidget);
         ui->timeslider->setEnabled (false);
    }
   // qDebug()<<temptree->topLevelItemCount();

 }
void ImageVideoManagerWidget::image_slider()
{
    ui->stackedWidget->setCurrentWidget(temptree);
    temptree->setMaximumSize(520,550);
    temptree->setMinimumSize(520,550);
    temptree->setIndentation(10);
    temptree->setColumnCount(3);
    temptree->setColumnWidth(0,170);
    temptree->setColumnWidth(1,170);
    temptree->setColumnWidth(2,180);

    QStringList list;
    list<<"Image"<<"TimeStamp"<<"Source";
    temptree->setHeaderLabels(list);
    temptree->clear();

    //ui->slidertree->show();
   // ui->slidertree->clear();
    int time = ui->timeslider->maximum();
    qDebug()<<time;
    QTreeWidgetItem *item;

    int j=0;
    int flag=0,ok;
{
    //settree_show();->child(i)
    //settree_hide();
       // int time = ui->timelabel->text().toInt(&ok,10);
    for( j=0;j<treeWidget->topLevelItemCount();j++)
    {
        item = treeWidget->topLevelItem(j);
        qDebug()<<"here in loop 1";
        if(time<=item->toolTip(1).toInt())
        {
            flag = 1;
            search_child(time,flag,item);
            qDebug()<<"here in loop 1"<<item->toolTip(1)<<flag;
            //break;
        }
    }
    qDebug()<<"here out loop 1";
}
}


