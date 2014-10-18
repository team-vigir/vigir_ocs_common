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
#include "notification_system.h"

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
    glancehub* ghub_;
    ros::NodeHandle nh_;
    ros::Publisher mode_pub_;
    void updateBoxSelection(int mode);
    void timerEvent(QTimerEvent *event);
    QString previous_selection_;
    QBasicTimer color_timer_;
    bool flashing_move_it_;
    bool flashing_footstep_;
    bool colored_moveit_;
    bool colored_footstep_;
    int flash_footstep_counter_;
    int flash_moveit_counter_;
    QString flash_color_moveit_;
    QString flash_color_footstep_;
    QString white_;
    int max_flashes_;
    std::vector<std::string> allowed_control_modes_;

public Q_SLOTS:
    void receiveMoveitStatus(bool);
    void receiveFootstepStatus(int);
    void receiveFlorStatus(int);
    void receiveModeChange(int);

};

#endif // glancehub_H
