#ifndef GLANCEHUBSBAR_H
#define GLANCEHUBSBAR_H

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <flor_control_msgs/FlorControlModeCommand.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>
#include <flor_ocs_msgs/OCSFootstepStatus.h>
#include "glancehub.h"
#include "notification_system.h"

#include <QMainWindow>
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

public Q_SLOTS:
    void receiveMoveitStatus(bool);
    void receiveFootstepStatus(int);
    void receiveFlorStatus(int);
    void modeChanged(int);

    virtual bool eventFilter( QObject * o, QEvent * e );

private:
    void timerEvent(QTimerEvent *event);

    glancehub* ghub_;

    Ui::glancehubSbar *ui;
    ros::NodeHandle nh_;
    ros::Publisher mode_pub_;

    QString previous_selection_;
    QBasicTimer color_timer_;
    bool flashing_move_it_;
    bool flashing_footstep_;
    bool colored_moveit_;
    bool colored_footstep_;
    bool ignore_events_;
    int flash_footstep_counter_;
    int flash_moveit_counter_;
    QString flash_color_moveit_;
    QString flash_color_footstep_;
    QString white_;
    int max_flashes_;
    std::vector<std::string> allowed_control_modes_;

};

#endif // glancehub_H
