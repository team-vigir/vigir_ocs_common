#ifndef glancehubsbar_H
#define glancehubsbar_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <sensor_msgs/JointState.h>
#include <QWidget>
#include <QTreeWidget>
#include <QFile>
#include <QtGui>
namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}
namespace Ui {
class glancehubsbar;
class jointListsbar;
}

class glancehubsbar : public QMainWindow
{
    Q_OBJECT

public:
    explicit glancehubsbar(QWidget *parent = 0);
    ~glancehubsbar();
    void controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg);
    void robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void updateList( const sensor_msgs::JointState::ConstPtr& joint_states );
    int getNumWarn();
    int getNumError();
protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glancehubsbar *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;
    ros::Subscriber joint_states;
    std::vector<QString> joints;
    std::vector<double> effortLimits;
    std::vector<double> upPoseLimit;
    std::vector<double> downPoseLimit;
    QTreeWidget* jointTable;
    void processRobotInfo(std::string robotInfo);
    float warnMin;
    float errorMin;
    int warn;
    int err;
    QWidget *warnwidget;
    QWidget *errorwidget;
    QTableWidget *warntable;
    QTableWidget *errortable;
    QTableWidgetItem *item;

private Q_SLOTS:
    void on_warnbutton_clicked();
    void on_errorbutton_clicked();

};

#endif // glancehubsbar_H
