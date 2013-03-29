#ifndef TemplateManagerWidget_H
#define TemplateManagerWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace Ui {
class TemplateManagerWidget;
}

class TemplateManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit TemplateManagerWidget(QWidget *parent = 0);
    ~TemplateManagerWidget();

    void processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg);
    void removeTemplate(int id);

    void initTemplateIdMap();
    void initGraspDB();

    std::vector< std::vector<QString> > readTextDBFile(QString path);

public Q_SLOTS:
    void editSlot(int,int);
    
private:

    Ui::TemplateManagerWidget* ui;

    QString template_dir_path_;
    QString grasp_db_path_;
    QString template_id_db_path_;

    std::map<unsigned char,std::string> template_id_map_;
    typedef struct
    {
        unsigned char grasp_id;
        unsigned char template_type;
        std::string template_name;
        std::string hand;
        std::string initial_grasp_type;
        float finger_joints[12];
        geometry_msgs::Pose final_pose;
        geometry_msgs::Pose pre_grasp_pose;
    } GraspDBItem;
    std::vector<GraspDBItem> grasp_db_;

    ros::NodeHandle nh_;
    ros::Subscriber template_list_sub_;
    ros::Publisher template_remove_pub_;
};

#endif // TemplateManagerWidget_H
