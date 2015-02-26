#ifndef GhostControlWidget_H
#define GhostControlWidget_H


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

#include <flor_ocs_msgs/OCSKeyEvent.h>
#include <flor_ocs_msgs/OCSGhostControl.h>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_grasp_msgs/InverseReachabilityForGraspRequest.h>
#include <vigir_object_template_msgs/GetGraspInfo.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "hotkey_manager.h"

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class GhostControlWidget;
}

class GhostControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit GhostControlWidget(QWidget *parent = 0);
    ~GhostControlWidget();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

    void processState( const flor_ocs_msgs::OCSGhostControl::ConstPtr& msg );
    void processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list);    
    void publishState( bool snap=false );
    int calcTargetPose(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2, geometry_msgs::Pose& pose_result);

    void saveState();
    void loadState(std::vector<unsigned char> planning_group=saved_state_planning_group_,
                   std::vector<unsigned char> pose_source=saved_state_pose_source_,
                   std::vector<unsigned char> world_lock=saved_state_world_lock_,
                   unsigned char collision_avoidance=saved_state_collision_avoidance_,
                   unsigned char lock_pelvis=saved_state_lock_pelvis_);

    bool useTorsoContextMenu();
    void snapContextMenu();

protected:
    void timerEvent(QTimerEvent *event);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

private Q_SLOTS:
    void applyClicked();
    void cancelClicked();
    void snapClicked();
    void sendTargetPoseClicked();
    void sendTargetConfigClicked();
    void resetPelvisClicked();
    void on_planning_left__clicked();
    void on_planning_torso__clicked();
    void on_position_only_ik__clicked();
    void on_planning_right__clicked();
    void on_lock_left__clicked();
    void on_lock_torso__clicked();
    void on_lock_right__clicked();
    void on_pose_left__currentIndexChanged(int index);
    void on_pose_torso__currentIndexChanged(int index);
    void on_pose_right__currentIndexChanged(int index);
    void on_lock_pelvis__clicked();
    void on_send_left_pose_button__clicked();
    void on_send_left_torso_pose_button__clicked();
    void on_send_right_pose_button__clicked();
    void on_send_right_torso_pose_button__clicked();
    void on_send_left_ghost_hand_button__clicked();
    void on_send_right_ghost_hand_button__clicked();
    //void on_left_no_lock_toggled(bool checked);
    //void on_left_marker_lock_toggled(bool checked);
    //void on_left_template_lock_toggled(bool checked);
    //void on_right_no_lock_toggled(bool checked);
    //void on_right_marker_lock_toggled(bool checked);
    //void on_right_template_lock_toggled(bool checked);
    void on_send_left_configuration_button__clicked();
    void on_send_left_torso_configuration_button__clicked();
    void on_send_right_configuration_button__clicked();
    void on_send_right_torso_configuration_button__clicked();
    void on_send_upper_body_button__clicked();
    //void on_left_moveit_marker_lock_clicked();
    //void on_right_moveit_marker_lock_clicked();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_send_ghost_to_template_button_clicked();
    void on_send_template_to_behavior_button_clicked();
    void on_templateBox_activated(const QString &arg1);
    void on_graspBox_activated(const QString &arg1);
    void on_send_left_cartesian_button__clicked();
    void on_send_right_cartesian_button__clicked();

private:
    ros::Subscriber window_control_sub;
    ros::Publisher window_control_pub;
    QRect geometry_;

    std::string getGroupNameForSettings(const std::vector<unsigned char>& settings);
    std::vector< std::vector<QString> > readTextDBFile(QString path);

    Ui::GhostControlWidget* ui;

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_; // need to make sure we subscribe to others in case someone else changes the configuration
    ros::Subscriber template_list_sub_; // subscriber for template list
    ros::Publisher state_pub_;
    ros::Publisher set_to_target_pose_pub_;
    ros::Publisher set_to_target_config_pub_;
    ros::Publisher reset_pelvis_pub_;
    ros::Publisher send_pelvis_pub_;
    ros::Publisher send_inverse_rechability_req_pub_;
    ros::Publisher send_ghost_to_template_pub_;
    ros::Publisher send_ghost_cartesian_pub_;
    ros::Publisher send_template_to_behavior_pub_;

    ros::ServiceClient grasp_info_client_;
    ros::ServiceClient template_info_client_;

    // variables that hold saved state of the widget
    static std::vector<unsigned char> saved_state_planning_group_;
    static std::vector<unsigned char> saved_state_pose_source_;
    static std::vector<unsigned char> saved_state_world_lock_;
    static unsigned char saved_state_collision_avoidance_;
    static unsigned char saved_state_lock_pelvis_;
    static unsigned char saved_state_position_only_ik_;

    flor_ocs_msgs::OCSTemplateList last_template_list_;
    int selected_template_id_;
    int selected_pose_id_;


    QBasicTimer timer;

    void addHotkeys();
};

#endif // GhostControlWidget_H
