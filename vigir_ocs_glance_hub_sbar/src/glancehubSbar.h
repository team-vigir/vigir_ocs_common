#ifndef GLANCEHUBSBAR_H
#define GLANCEHUBSBAR_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <QTableWidgetItem>
#include <QTableWidget>
#include "glancehub.h"
#include <ros/ros.h>

namespace Ui {
class glancehubSbar;
}


class glancehubSbar : public QMainWindow
{
    Q_OBJECT

public:
    ~glancehubSbar();
    explicit glancehubSbar(QWidget *parent = 0);

private:
    Ui::glancehubSbar *ui;
    glancehub* ghub;
    ros::NodeHandle nh;
    ros::Publisher mode_pub;

public Q_SLOTS:
    void receiveMoveitStatus(bool);
    void receiveFootstepStatus(int);
    void receiveFlorStatus(int);
    void receiveModeChange(int);

};

#endif // glancehub_H
