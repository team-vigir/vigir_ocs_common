#ifndef STATUS_WINDOW_H
#define STATUS_WINDOW_H

#include <QPainter>
#include <QPalette>
#include <QWidget>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <flor_control_msgs/FlorControlMode.h>
#include <flor_ocs_msgs/OCSRobotStability.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>

#include <jointList.h>
#include <robotStatus.h>

namespace Ui {
class status_window;
}

class status_window : public QWidget
{
    Q_OBJECT

public:
    explicit status_window(QWidget *parent = 0);
    ~status_window();

    void controlModeMsgReceived(const flor_control_msgs::FlorControlMode::ConstPtr& modeMsg);
    void stabilityMsgReceived(const flor_ocs_msgs::OCSRobotStability::ConstPtr& stabilityMsg);
    void updateButtonColor();
    QString getControllerStatus(uint8_t flag);

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private Q_SLOTS:
    void on_showJointButton_clicked();
    void on_showRobotStatus_clicked();
protected:
    void timerEvent(QTimerEvent *event);
private:
    Ui::status_window *ui;
    ros::Subscriber mode_subscriber;
    ros::Subscriber stability_subscriber;
    robotStatus* rbtStatus;
    jointList* jntList;
    QBasicTimer timer;
    QBasicTimer timerColor;
    QString oldJointStyleSheet;
    QString oldRobotStyleSheet;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;
};

#endif // STATUS_WINDOW_H
