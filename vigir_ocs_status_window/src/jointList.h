#ifndef JOINTLIST_H
#define JOINTLIST_H

#include <QMainWindow>
#include <QWidget>
#include <QTreeWidget>
#include <QFile>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <sensor_msgs/JointState.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

class jointList : public QWidget
{
    Q_OBJECT
    
public:
    explicit jointList(QWidget *parent = 0);
    ~jointList();
    void updateList( const sensor_msgs::JointState::ConstPtr& joint_states );
    int getNumWarn();
    int getNumError();

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private:
    ros::Subscriber joint_states;
    std::vector<QTreeWidgetItem*> joints;
    std::vector<double> effortLimits;
    std::vector<double> upPoseLimit;
    std::vector<double> downPoseLimit;
    QTreeWidget* jointTable;
    void processRobotInfo(std::string robotInfo);
    float warnMin;
    float errorMin;
    int warn;
    int err;
    bool jointsOkay;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;

Q_SIGNALS:
    void sendJointData(int,QString);
};

#endif // JOINTLIST_H
