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

#include <flor_ocs_msgs/OCSImageList.h>
#include <flor_ocs_msgs/OCSImageAdd.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

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

    void processImageAdd(const flor_ocs_msgs::OCSImageAdd::ConstPtr& msg);
    void processImageList(const flor_ocs_msgs::OCSImageList::ConstPtr& msg);
    /*void removeImage(int id);

    void initImageIdMap();
    void initGraspDB();

    std::vector< std::vector<QString> > readTextDBFile(QString path);

    void configureGrasps(std::string image_name, QComboBox* combo_box);*/

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


protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // ImageManagerWidget_H
