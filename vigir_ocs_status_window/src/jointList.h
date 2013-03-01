#ifndef JOINTLIST_H
#define JOINTLIST_H

#include <QWidget>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include<QTreeWidget>
#include<QFile>

class jointList : public QWidget
{
    Q_OBJECT
    
public:
    explicit jointList(QWidget *parent = 0);
    ~jointList();
    void updateList( const sensor_msgs::JointStateConstPtr& joint_states );

private:
    ros::Subscriber joint_states;
    QTreeWidget* jointTable;
    QTreeWidgetItem joints[];
    float positionLimits[];
    float velocityLimits[];
    float effortLimits[];
};

#endif // JOINTLIST_H
