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
#include <flor_ocs_msgs/OCSJoints.h>

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
    static void getGhostJointStates(sensor_msgs::JointState joint_info, std::vector<int>& jointStates);

    struct Limits
    {
        double effortLimit, upPoseLimit, downPoseLimit;
    };

private:    
    ros::Subscriber joint_states;
    ros::Publisher joint_pub;
    static std::vector<double> effortLimits;
    static std::vector<double> upPoseLimit;
    static std::vector<double> downPoseLimit;
    static float warnMin;
    static float errorMin;
    static std::vector<QTreeWidgetItem*> joints;
    QTreeWidget* jointTable;
    static void processRobotInfo(std::string robotInfo);

    int warn;
    int err;
    bool jointsOkay;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;

    static std::map<std::string, jointList::Limits> jointNameLimits;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;

Q_SIGNALS:
    void sendJointData(int,QString);
};

#endif // JOINTLIST_H
