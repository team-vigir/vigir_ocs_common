#ifndef GLANCEHUB_H
#define GLANCEHUB_H

#include <vector>
#include <algorithm>

#include <QMainWindow>
#include <QBasicTimer>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <flor_control_msgs/FlorControlModeCommand.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>

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

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glanceHub *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;
};

#endif // GLANCEHUB_H
