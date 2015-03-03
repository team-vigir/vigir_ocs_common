#ifndef GRASPWIDGET_H
#define GRASPWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QPushButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>
#include <QPainter>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include "tf/transform_listener.h"

#include "handOffsetWidget.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>
#include <flor_ocs_msgs/OCSLinkColor.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>
#include <flor_ocs_msgs/OCSObjectSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <flor_grasp_msgs/LinkState.h>
#include <flor_planning_msgs/CircularMotionRequest.h>

#include <vigir_object_template_msgs/GetGraspInfo.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>

#define FINGER_EFFORTS 4

namespace Ui {
class graspWidget;
}

class graspWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit graspWidget(QWidget *parent = 0, std::string hand = "left", std::string hand_name = "l_hand");
    ~graspWidget();

    void processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr& msg);
    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    Ui::graspWidget * getUi();
    QLayout * getMainLayout();
public Q_SLOTS:
    void on_userSlider_sliderReleased();
    void on_templateBox_activated(const QString &arg1);
    void on_graspBox_activated(const QString &arg1);
    void on_affordanceBox_activated(const QString &arg1);
    void on_performButton_clicked();
    void on_templateButton_clicked();
    void on_releaseButton_clicked();
    void on_manualRadio_clicked();
    void on_templateRadio_clicked();
    void on_noneRadio_clicked();
    void on_show_grasp_toggled(bool checked);
    void on_stitch_template_toggled(bool checked);
    void on_verticalSlider_sliderReleased();
    void on_verticalSlider_3_sliderReleased();
    void on_verticalSlider_2_sliderReleased();
    void on_verticalSlider_4_sliderReleased();
    void sendCircularTarget();

private:
    void setProgressLevel(uint8_t level);
    void sendManualMsg(uint8_t level, int8_t thumb, int8_t left, int8_t right, int8_t spread);
    void initTemplateMode();

    QString icon_path_;
    void setUpButtons();

    Ui::graspWidget *ui;
    handOffsetWidget *ui2;

    void graspStateReceived (const flor_grasp_msgs::GraspState::ConstPtr& graspState);
    void graspSelectedReceived (const flor_grasp_msgs::GraspSelection::ConstPtr& graspMsg);
    void processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list);
    void templateMatchFeedback (const flor_grasp_msgs::TemplateSelection::ConstPtr& feedback);

    int calcWristTarget( const geometry_msgs::Pose& wrist_pose,const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& final_pose );
    void calcPlanningTarget(const geometry_msgs::Pose& palm_pose, const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& planning_hand_pose);
    int hideHand();
    int staticTransform(geometry_msgs::Pose& palm_pose);
    int poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose);
    int poseTransform(geometry_msgs::Pose& input_pose, tf::Transform transform);
    void gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans);


    // need to store updated template list and selected template id to calculate final position of the hand
    vigir_object_template_msgs::GetGraspInfo                last_grasp_srv_;
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo last_template_srv_;

    flor_ocs_msgs::OCSTemplateList         last_template_list_;
    geometry_msgs::PoseStamped             frameid_T_template_;
    vigir_object_template_msgs::Affordance current_affordance_;
    int selected_template_id_;
    int selected_grasp_id_;
    int selected_affordance_id_;

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

    ros::Publisher planning_hand_target_pub_;

    geometry_msgs::Pose feedbackPose;

    bool templateMatchDone;
    bool follow_ban_;

    std::string hand_side_;
    std::string hand_type_;
    std::string hand_name_;
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
    void publishHandJointStates(std::vector<float>&);

    tf::TransformListener tf_;

    tf::Transform stitch_template_pose_;
    tf::Transform hand_offset_pose_;
    tf::Transform hand_T_palm_;   //describes palm in hand frame
    tf::Transform gp_T_palm_;     //describes palm in grasp pose frame
    tf::Transform hand_T_marker_; //describes marker in hand frame

    QWidget*        circular_config_widget_;
    QCheckBox*      circular_use_collision_;
    QCheckBox*      circular_keep_orientation_;
    QDoubleSpinBox* circular_angle_;

    // get joint states
    ros::Subscriber link_states_sub_;

    void linkStatesCB(const flor_grasp_msgs::LinkState::ConstPtr& link_states);

    bool show_grasp_;
    bool stitch_template_;

    robot_model_loader::RobotModelLoaderPtr hand_model_loader_;
    robot_model::RobotModelPtr hand_robot_model_;
    robot_state::RobotStatePtr hand_robot_state_;

    std::vector<std::string>   hand_joint_names_;

    moveit_msgs::DisplayRobotState display_state_msg_;
    ros::Publisher robot_state_vis_pub_;

    // Used to make setting virtual joint positions (-> hand pose) easier
    sensor_msgs::JointState virtual_link_joint_states_;

    ros::Publisher     select_object_pub_;
    ros::Subscriber    select_object_sub_;

    std::vector<int>   keys_pressed_list_;

    ros::Subscriber    key_event_sub_;

    ros::ServiceClient grasp_info_client_;
    ros::ServiceClient template_info_client_;


    ros::Publisher circular_plan_request_pub_;

protected:
    void timerEvent(QTimerEvent *event);
    /**
      * Context menu action for creating a circular target point
      */
    void removeCircularContextMenu();
    /**
      * Publishes the circular target pose
      */


private:
    QBasicTimer timer;
};

#endif // GRASPWIDGET_H
