#ifndef GLANCEHUB_H
#define GLANCEHUB_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include<QTableWidgetItem>
#include<QTableWidget>

namespace Ui {
class glancehub;
}


class glancehub : public QMainWindow
{
    Q_OBJECT

public:
    explicit glancehub(QWidget *parent = 0);
    ~glancehub();
    void controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg);
    void robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void loadFile();
    QString timeFromMsg(ros::Time stamp);

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glancehub *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;

    std::vector<std::string> errors;
    std::string messagesPath;

};

#endif // glancehub_H
