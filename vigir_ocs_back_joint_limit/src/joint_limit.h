#ifndef joint_limit_H
#define joint_limit_H

#include <QWidget>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <QBasicTimer>
#include "ui_joint_limit.h"

//namespace Ui {
//class joint_limit;
//}

class joint_limit : public QWidget
{
    Q_OBJECT

public:
    explicit joint_limit(QWidget *parent = 0);
    ~joint_limit();

private:
    Ui::joint_limit *ui;
    ros::NodeHandle nh_;
    ros::Publisher constraints_pub_;
    QBasicTimer timer;
public Q_SLOTS:
    void on_apply_clicked();
protected:
    void timerEvent(QTimerEvent *event);
};

#endif // joint_limit_H
