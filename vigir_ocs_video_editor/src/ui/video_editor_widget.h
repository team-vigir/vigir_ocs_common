/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#ifndef VideoEditorWidget_H
#define VideoEditorWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>
#include<QTreeWidgetItem>

#include <vigir_ocs_msgs/OCSImageList.h>
#include <vigir_ocs_msgs/OCSImageAdd.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vigir_perception_msgs/DownSampledImageRequest.h>
#include <ros/ros.h>

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class VideoEditorWidget;
}

class VideoEditorWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit VideoEditorWidget(QWidget *parent = 0);
    ~VideoEditorWidget();

    void processImageAdd(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void processImageList(const vigir_ocs_msgs::OCSImageList::ConstPtr msg);
    void processSelectedImage(const sensor_msgs::Image::ConstPtr msg);
    void processvideoimage (const vigir_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void addImageChild(QTreeWidgetItem *parent, const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info);
    QTreeWidgetItem* addvideoitem(int videocount,const sensor_msgs::Image& image);
    QTreeWidgetItem* addimageitem(int imagecount,const sensor_msgs::Image& image);
    bool check_item_time(QTreeWidgetItem *item, int time);
    void settree_show();
    QTreeWidgetItem* add_time_child(QTreeWidgetItem *pitem);
    int video_start_time, subseq_video_time;
    int video_start_time_nano, subseq_video_time_nano;

    /*void removeImage(int id);

    void initImageIdMap();*/


private:
    void addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info);
    QString timeFromMsg(const ros::Time& stamp);
    float feed_rate;
    float feed_rate_prev;
    Ui::VideoEditorWidget* ui;
    QTreeWidgetItem *item;
    QTreeWidgetItem *parentitem;
    QTreeWidgetItem *timeitem;
    int videocount;
    int imagecount;
    int interval_count;

    ros::NodeHandle nh_;

    ros::Publisher image_list_request_pub_;
    ros::Publisher image_selected_pub_;

    ros::Subscriber image_list_sub_;
    ros::Subscriber image_added_sub_;
    ros::Subscriber image_selected_sub_;

    // subscribers for video stream
    ros::Subscriber img_req_sub_full_l_ ;
    ros::Subscriber img_req_sub_full_r_;
    ros::Subscriber img_req_sub_full_lhl_;
    ros::Subscriber img_req_sub_full_lhr_ ;
    ros::Subscriber img_req_sub_full_rhl_;
    ros::Subscriber img_req_sub_full_rhr_ ;
    ros::Subscriber img_req_sub_crop_l_;
    ros::Subscriber img_req_sub_crop_r_;
    ros::Subscriber img_req_sub_crop_lhl_;
    ros::Subscriber img_req_sub_crop_lhr_ ;
    ros::Subscriber img_req_sub_crop_rhl_ ;
    ros::Subscriber img_req_sub_crop_rhr_ ;
    



protected:
    void timerEvent(QTimerEvent *event);

private Q_SLOTS:
    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);
    void on_timeslider_valueChanged(int value);
    void on_pushButton_clicked();

private:
    QBasicTimer timer;
};

#endif // VideoEditorWidget_H
