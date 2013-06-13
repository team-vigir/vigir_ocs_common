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
    //void editSlot(int,int);
    
private:

    Ui::ImageManagerWidget* ui;

    ros::NodeHandle nh_;

    ros::Publisher image_list_request_pub_;
    ros::Publisher image_selected_pub_;

    ros::Subscriber image_list_sub_;
    ros::Subscriber image_added_sub_;

    /*QString image_dir_path_;
    QString grasp_db_path_;
    QString image_id_db_path_;

    std::map<unsigned char,std::string> image_id_map_;
    typedef struct
    {
        unsigned short grasp_id;
        unsigned char image_type;
        std::string image_name;
        std::string hand;
        std::string initial_grasp_type;
        float finger_joints[12];
        geometry_msgs::Pose final_pose;
        geometry_msgs::Pose pre_grasp_pose;
    } GraspDBItem;
    std::vector<GraspDBItem> grasp_db_;

    ros::NodeHandle nh_;
    ros::Subscriber image_list_sub_;
    ros::Publisher image_remove_pub_;
    ros::Publisher image_match_request_pub_;
    ros::Publisher grasp_request_pub_;*/

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // ImageManagerWidget_H
