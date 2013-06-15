#ifndef GLANCEHUB_H
#define GLANCEHUB_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>

namespace Ui {
class glanceHub;
}

class glanceHub : public QMainWindow
{
    Q_OBJECT

public:
    explicit glanceHub(QWidget *parent = 0);
    ~glanceHub();
    void controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg);
    void robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glanceHub *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;
};

#endif // GLANCEHUB_H
