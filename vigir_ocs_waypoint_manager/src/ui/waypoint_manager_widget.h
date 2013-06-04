#ifndef WaypointManagerWidget_H
#define WaypointManagerWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <flor_ocs_msgs/OCSWaypointUpdate.h>
#include <flor_ocs_msgs/OCSWaypointRemove.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace Ui {
class WaypointManagerWidget;
}

class WaypointManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit WaypointManagerWidget(QWidget *parent = 0);
    ~WaypointManagerWidget();

    void processWaypointList(const nav_msgs::Path::ConstPtr& msg);
    void removeWaypoint(int id);

public Q_SLOTS:
    void editSlot(int,int);
    void confirmNavigation();

private:
    Ui::WaypointManagerWidget* ui;

    ros::NodeHandle nh_;
    ros::Subscriber waypoint_list_sub_;
    ros::Publisher waypoint_remove_pub_;
    ros::Publisher confirm_navigation_pub_;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // WaypointManagerWidget_H
