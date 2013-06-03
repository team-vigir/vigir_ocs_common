#ifndef GRASPWIDGET_H
#define GRASPWIDGET_H

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
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>

namespace Ui {
class graspWidget;
}

class graspWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit graspWidget(QWidget *parent = 0);
    ~graspWidget();
    
public Q_SLOTS:
    void on_userSlider_sliderReleased();
    void on_templateBox_activated(const QString &arg1);
    void on_graspBox_activated(const QString &arg1);
    void on_performButton_clicked();
    void on_templateButton_clicked();
    void on_releaseButton_clicked();
    void on_manualRadio_clicked();
    void on_templateRadio_clicked();

private:
    void setProgressLevel(uint8_t level);
    void sendManualMsg(uint8_t level);
    void initTemplateMode();
    void initTemplateIdMap();
    std::vector< std::vector<QString> > readTextDBFile(QString path);
    void initGraspDB();

    Ui::graspWidget *ui;

    flor_ocs_msgs::OCSTemplateList lastList;
    void graspStateRecieved (const flor_grasp_msgs::GraspState::ConstPtr& graspState);
    void graspSelectedRecieved (const flor_grasp_msgs::GraspSelection::ConstPtr& graspMsg);
    void processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list);
    void templateMatchFeedback (const flor_grasp_msgs::TemplateSelection::ConstPtr& feedback);

    QString template_dir_path_;
    QString grasp_db_path_;
    QString template_id_db_path_;

    std::map<unsigned char,std::string> template_id_map_;
    typedef struct
    {
        unsigned short grasp_id;
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
    ros::Subscriber grasp_state_sub_;
    ros::Subscriber grasp_selected_sub_;
    ros::Subscriber template_select_sub;
    ros::Subscriber template_list_sub_;
    ros::Subscriber template_match_feedback_sub_;
    ros::Publisher grasp_selection_pub_;
    ros::Publisher template_remove_pub_;
    ros::Publisher grasp_mode_command_pub_;
    ros::Publisher template_match_request_pub_;
    ros::Publisher grasp_request_pub_;
    ros::Publisher grasp_release_pub_;

    geometry_msgs::Pose feedbackPose;
    bool templateMatchDone;
    std::string hand;
//    uint8_t currentGraspState;
    uint8_t currentGraspMode;

    // **************************
    // show robot status messages
    ros::Subscriber robot_status_sub_;
    RobotStatusCodes robot_status_codes_;

    void robotStatusCB(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
};

#endif // GRASPWIDGET_H
