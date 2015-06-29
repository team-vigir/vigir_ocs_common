#ifndef ImageManagerWidget_H
#define ImageManagerWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <vigir_ocs_msgs/OCSImageList.h>
#include <vigir_ocs_msgs/OCSImageAdd.h>
#include <vigir_ocs_msgs/OCSKeyEvent.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

#include "hotkey_manager.h"

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class ImageManagerWidget;
}

class ImageManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ImageManagerWidget(QWidget *parent = 0);
    ~ImageManagerWidget();

    void processImageAdd(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void processImageList(const vigir_ocs_msgs::OCSImageList::ConstPtr msg);
    void processSelectedImage(const sensor_msgs::Image::ConstPtr msg);
    void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr pose);
    /*void removeImage(int id);

    void initImageIdMap();*/

public Q_SLOTS:
    void imageClicked(int,int);
    
private:
    void addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info);
    QString timeFromMsg(const ros::Time& stamp);

    Ui::ImageManagerWidget* ui;

    ros::NodeHandle nh_;

    ros::Publisher image_list_request_pub_;
    ros::Publisher image_selected_pub_;

    ros::Subscriber image_list_sub_;
    ros::Subscriber image_added_sub_;
    ros::Subscriber image_selected_sub_;

    ///Hotkey
    void addHotKeys();
    void toggleImageManagerHotkey();



protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // ImageManagerWidget_H
