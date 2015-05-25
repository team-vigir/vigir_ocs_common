#ifndef ImageVideoManagerWidget_H
#define ImageVideoManagerWidget_H


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
#include <flor_perception_msgs/DownSampledImageRequest.h>
#include <ros/ros.h>

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class ImageVideoManagerWidget;
}

class ImageVideoManagerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageVideoManagerWidget(QWidget *parent = 0);
    ~ImageVideoManagerWidget();

    void processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr msg);
    void processSelectedImage(const sensor_msgs::Image::ConstPtr msg);
    void processvideoimage_l (const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void processvideoimage_r (const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void processvideoimage_lhl (const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void processvideoimage_lhr (const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void processvideoimage_rhl (const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);
    void processvideoimage_rhr(const flor_perception_msgs::DownSampledImageRequest::ConstPtr msg);

    void addImageChild(QTreeWidgetItem *parent, const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info,int flag);
    QTreeWidgetItem* addvideoitem(int videocount,const sensor_msgs::Image& image);
   // QTreeWidgetItem* addimageitem(int imagecount,const sensor_msgs::Image& image);
    bool check_item_time(QTreeWidgetItem *item, int time);
    void settree_show();
    QTreeWidgetItem* add_time_child(QTreeWidgetItem *pitem, int cnt);
    void thumbnail(const sensor_msgs::Image& image,QTreeWidgetItem *item);
    void imageaddfunction_l(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void imageaddfunction_r(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void imageaddfunction_lhl(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void imageaddfunction_lhr(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void imageaddfunction_rhl(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void imageaddfunction_rhr(const flor_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void settree_hide();
    void search_child(int time,int flag, QTreeWidgetItem *item);
    void image_slider();
    //void save_to_file(int,int);
    //void search_child_save(int timer2, int timer1, QTreeWidgetItem *item, int flag);
// for adding images to widget

    int video_start_time_l, subseq_video_time_l,childtimeitem_start_time_l;
    int video_start_time_r, subseq_video_time_r,childtimeitem_start_time_r;
    int video_start_time_lhr, subseq_video_time_lhr,childtimeitem_start_time_lhr;
    int video_start_time_lhl, subseq_video_time_lhl,childtimeitem_start_time_lhl;
    int video_start_time_rhr, subseq_video_time_rhr,childtimeitem_start_time_rhr;
    int video_start_time_rhl, subseq_video_time_rhl,childtimeitem_start_time_rhl;

    //int timer1,timer2; // to count the interval for saving files to disk




private:
    void addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info);
    QString timeFromMsg(const ros::Time& stamp);
    float feed_rate_l;
    float feed_rate_r;
    float feed_rate_lhl;
    float feed_rate_lhr;
    float feed_rate_rhl;
    float feed_rate_rhr;
    float feed_rate_prev_l;
    float feed_rate_prev_r;
    float feed_rate_prev_lhl;
    float feed_rate_prev_lhr;
    float feed_rate_prev_rhl;
    float feed_rate_prev_rhr;
    Ui::ImageVideoManagerWidget* ui;
    QTreeWidgetItem *item_l;
    QTreeWidgetItem *item_r;
    QTreeWidgetItem *parentitem_l;
    QTreeWidgetItem *timeitem_l;
    QTreeWidgetItem *childtimeitem_l;
    QTreeWidgetItem *item;
    QTreeWidgetItem *parentitem_r;
    QTreeWidgetItem *timeitem_r;
    QTreeWidgetItem *childtimeitem_r;
    QTreeWidgetItem *item_lhl;
    QTreeWidgetItem *parentitem_lhl;
    QTreeWidgetItem *timeitem_lhl;
    QTreeWidgetItem *childtimeitem_lhl;
    QTreeWidgetItem *item_lhr;
    QTreeWidgetItem *parentitem_lhr;
    QTreeWidgetItem *timeitem_lhr;
    QTreeWidgetItem *childtimeitem_lhr;
    QTreeWidgetItem *item_rhl;
    QTreeWidgetItem *parentitem_rhl;
    QTreeWidgetItem *timeitem_rhl;
    QTreeWidgetItem *childtimeitem_rhl;
    QTreeWidgetItem *item_rhr;
    QTreeWidgetItem *parentitem_rhr;
    QTreeWidgetItem *timeitem_rhr;
    QTreeWidgetItem *childtimeitem_rhr;
    QTreeWidget *temptree;
    QTreeWidget *treeWidget;
   // QStackedWidget *stack ; comment now
    int videocount_l;
    int imagecount_l;
    int interval_count_l;
    int videocount_lhl;
    int imagecount_lhl;
    int interval_count_lhl;
    int videocount_lhr;
    int imagecount_lhr;
    int interval_count_lhr;
    int videocount_r;
    int imagecount_r;
    int interval_count_r;
    int videocount_rhr;
    int imagecount_rhr;
    int interval_count_rhr;
    int videocount_rhl;
    int imagecount_rhl;
    int interval_count_rhl;
    int flag_first_image;
    //int timerflag;

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
    //void on_temptree_itemClicked(QTreeWidgetItem *item, int column);
    void on_timeslider_valueChanged(int value);
    //void on_pushButton_clicked();



    void on_cameralist_currentIndexChanged(int index);

    //void on_pushButton_clicked();

    void on_checkBox_clicked(bool checked);

private:
    QBasicTimer timer;
};

#endif // ImageVideoManagerWidget_H
