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

#include <flor_ocs_msgs/OCSImageList.h>
#include <flor_ocs_msgs/OCSImageAdd.h>

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

    void processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr msg);
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
