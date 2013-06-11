#ifndef STATUS_WINDOW_H
#define STATUS_WINDOW_H

#include<QPainter>
#include <QPalette>
#include <QWidget>
#include <jointList.h>
#include <robotStatus.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <flor_ocs_msgs/OCSRobotStability.h>
#include <ros/subscriber.h>

namespace Ui {
class status_window;
}

class status_window : public QWidget
{
    Q_OBJECT

public:
    explicit status_window(QWidget *parent = 0);
    void controlModeMsgRecieved(const flor_control_msgs::FlorControlMode::ConstPtr& modeMsg);
    void stabilityMsgRecieved(const flor_ocs_msgs::OCSRobotStability::ConstPtr& stabilityMsg);
    void updateButtonColor();
    QString getControllerStatus(uint8_t flag);
    ~status_window();

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
};

#endif // STATUS_WINDOW_H
