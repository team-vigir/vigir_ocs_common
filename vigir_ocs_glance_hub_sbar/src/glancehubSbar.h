#ifndef GLANCEHUBSBAR_H
#define GLANCEHUBSBAR_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include "glancehub.h"
#include <ros/ros.h>
#include <QPropertyAnimation>
#include <QDialog>
#include <QBasicTimer>
#include <QElapsedTimer>

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
    void updateBoxSelection(QString mode);
    void timerEvent(QTimerEvent *event);
    QString previousSelection;
    QBasicTimer colorTimer;
    bool flashingMoveIt;
    bool flashingFootstep;
    bool coloredMoveIt;
    bool coloredFootstep;
    int flashFootstepCounter;
    int flashMoveItCounter;
    QString flashColorMoveIt;
    QString flashColorFootstep;
    QString white;
    int maxFlashes;

public Q_SLOTS:
    void receiveMoveitStatus(bool);
    void receiveFootstepStatus(int);
    void receiveFlorStatus(int);
    void receiveModeChange(int);

};

#endif // glancehub_H
