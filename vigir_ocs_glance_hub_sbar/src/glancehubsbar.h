#ifndef glancehubsbar_H
#define glancehubsbar_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>

namespace Ui {
class glancehubsbar;
}

class glancehubsbar : public QMainWindow
{
    Q_OBJECT

public:
    explicit glancehubsbar(QWidget *parent = 0);
    ~glancehubsbar();
    void controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg);
    void robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glancehubsbar *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;
};

#endif // glancehubsbar_H
