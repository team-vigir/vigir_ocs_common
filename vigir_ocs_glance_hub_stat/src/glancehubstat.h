#ifndef glancehubstat_H
#define glancehubstat_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include<QTableWidgetItem>
#include<QTableWidget>

namespace Ui {
class glancehubstat;
class completeRow;
}
class completeRow
{
public:
    QTableWidgetItem* text;
    QTableWidgetItem* time;
    QTableWidgetItem* priority;

};
class glancehubstat : public QMainWindow
{
    Q_OBJECT

public:
    explicit glancehubstat(QWidget *parent = 0);
    ~glancehubstat();
    void controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg);
    void robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    void loadFile();
    QString timeFromMsg(ros::Time stamp);

protected:
    void timerEvent(QTimerEvent *event);

private:
    Ui::glancehubstat *ui;
    ros::Subscriber controlMode_sub;
    ros::Subscriber robotStatusMoveit_sub;
    ros::Subscriber robotStatusFootstep_sub;
    QBasicTimer timer;
    QStringList labels;
    std::vector<std::string> errors;
    std::string messagesPath;

};

#endif // glancehubstat_H
