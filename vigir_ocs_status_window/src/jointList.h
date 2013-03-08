#ifndef JOINTLIST_H
#define JOINTLIST_H

#include <QWidget>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
//#include < insert message file for limits FOO
#include<QTreeWidget>
#include<QFile>

class jointList : public QWidget
{
    Q_OBJECT
    
public:
    explicit jointList(QWidget *parent = 0);
    ~jointList();
    void updateList( const sensor_msgs::JointStateConstPtr& joint_states );
    //void getLimitsMessage ( const FOO& lmt_msg);
    int getNumWarn();
    int getNumError();

private:
    ros::Subscriber joint_states;
    //FOO limitsMessage;
    QTreeWidget* jointTable;
    QTreeWidgetItem joints[];
    float warnMin;
    float errorMin;
    int warn;
    int error;
};

#endif // JOINTLIST_H
