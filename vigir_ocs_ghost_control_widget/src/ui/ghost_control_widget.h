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

#include <flor_ocs_msgs/OCSGhostControl.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

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

    void processState( const flor_ocs_msgs::OCSGhostControl::ConstPtr& msg );
    void publishState( bool snap=false );

    void saveState();
    void loadState(std::vector<unsigned char> planning_group=saved_state_planning_group_,
                   std::vector<unsigned char> pose_source=saved_state_pose_source_,
                   std::vector<unsigned char> world_lock=saved_state_world_lock_,
                   unsigned char collision_avoidance=saved_state_collision_avoidance_,
                   unsigned char lock_pelvis=saved_state_lock_pelvis_);

protected:
    void timerEvent(QTimerEvent *event);

private Q_SLOTS:
    void applyClicked();
    void cancelClicked();
    void snapClicked();
    void sendTargetPoseClicked();
    void sendTargetConfigClicked();
    void resetPelvisClicked();

private:
    Ui::GhostControlWidget* ui;

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_; // need to make sure we subscribe to others in case someone else changes the configuration
    ros::Publisher state_pub_;
    ros::Publisher set_to_target_pose_pub_;
    ros::Publisher set_to_target_config_pub_;
    ros::Publisher reset_pelvis_pub_;

    // variables that hold saved state of the widget
    static std::vector<unsigned char> saved_state_planning_group_;
    static std::vector<unsigned char> saved_state_pose_source_;
    static std::vector<unsigned char> saved_state_world_lock_;
    static unsigned char saved_state_collision_avoidance_;
    static unsigned char saved_state_lock_pelvis_;

    QBasicTimer timer;
};

#endif // GhostControlWidget_H
