#ifndef JOINTLIST_H
#define JOINTLIST_H

#include <QWidget>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
//#include < insert message file for limits FOO
#include<QTreeWidget>
#include<QFile>

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
};

#endif // JOINTLIST_H
