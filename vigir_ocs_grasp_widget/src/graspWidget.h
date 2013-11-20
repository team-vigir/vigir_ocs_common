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

#include <ros/ros.h>

#include "tf/transform_listener.h"

#include "handOffsetWidget.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>
#include <flor_ocs_msgs/OCSLinkColor.h>

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
    void on_userSlider_2_sliderReleased();
    void on_templateBox_activated(const QString &arg1);
    void on_graspBox_activated(const QString &arg1);
    void on_performButton_clicked();
    void on_templateButton_clicked();
    void on_releaseButton_clicked();
    void on_manualRadio_clicked();
    void on_templateRadio_clicked();
    void on_show_grasp_toggled(bool checked);
    void on_stitch_template_toggled(bool checked);
    void on_pushButton_clicked();

private:
    void setProgressLevel(uint8_t level);
    void sendManualMsg(uint8_t level, uint8_t thumb);
    void initTemplateMode();
    void initTemplateIdMap();
    std::vector< std::vector<QString> > readTextDBFile(QString path);
    void initGraspDB();

    Ui::graspWidget *ui;
    handOffsetWidget *ui2;

    void graspStateReceived (const flor_grasp_msgs::GraspState::ConstPtr& graspState);
    void graspSelectedReceived (const flor_grasp_msgs::GraspSelection::ConstPtr& graspMsg);
    void processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list);
    void templateMatchFeedback (const flor_grasp_msgs::TemplateSelection::ConstPtr& feedback);

    int calcWristTarget( const geometry_msgs::Pose& wrist_pose,const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& final_pose );
    int hideHand();
    int staticTransform(geometry_msgs::Pose& palm_pose);

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

    // need to store updated template list and selected template id to calculate final position of the hand
    flor_ocs_msgs::OCSTemplateList last_template_list_;
    int selected_template_id_;
    int selected_grasp_id_;

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
    ros::Publisher template_stitch_request_pub_;
    ros::Publisher grasp_request_pub_;
    ros::Publisher grasp_release_pub_;

    geometry_msgs::Pose feedbackPose;

    bool templateMatchDone;

    std::string hand_;
    std::string hand_type_;
    uint8_t currentGraspMode;

    // **************************
    // show robot status messages
    ros::Subscriber robot_status_sub_;
    ros::Subscriber template_stitch_pose_sub_;
    ros::Subscriber hand_offset_sub_;
    RobotStatusCodes robot_status_codes_;

    void robotStatusCB(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void templateStitchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void handOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // publisher to color fingers/hand
    ros::Publisher hand_link_color_pub_;
    void publishLinkColor(std::string, unsigned char r, unsigned char g, unsigned char b);

    // publisher for ghost hand position
    ros::Publisher ghost_hand_pub_;
    void publishHandPose(unsigned int id);

    // publisher for the finger joints
    ros::Publisher ghost_hand_joint_state_pub_;
    void publishHandJointStates(unsigned int grasp_index);

    tf::TransformListener tf_;

    tf::Transform stitch_template_pose_;
    tf::Transform hand_offset_pose_;
    tf::Transform hand_T_palm_;   //describes palm in hand frame
    tf::Transform gp_T_palm_;     //describes palm in grasp pose frame

    // get joint states
    ros::Subscriber joint_states_sub_;

    void jointStatesCB(const sensor_msgs::JointState::ConstPtr& joint_states);

    bool show_grasp_;
    bool stitch_template_;

    robot_model_loader::RobotModelLoaderPtr hand_model_loader_;
    robot_model::RobotModelPtr hand_robot_model_;
    robot_state::RobotStatePtr hand_robot_state_;

    moveit_msgs::DisplayRobotState display_state_msg_;
    ros::Publisher robot_state_vis_pub_;

    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState virtual_link_joint_states_;


protected:
    void timerEvent(QTimerEvent *event);

private:
    QBasicTimer timer;
};

#endif // GRASPWIDGET_H
