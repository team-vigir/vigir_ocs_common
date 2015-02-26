#ifndef BEHAVIOR_NOTIFICATION_H
#define BEHAVIOR_NOTIFICATION_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <QtCore>

namespace Ui
{
    class BehaviorNotification; //window name
}

class BehaviorNotification : public QMainWindow
{
    Q_OBJECT

private:
    Ui::BehaviorNotification *ui;


public:
    ~BehaviorNotification();
    explicit BehaviorNotification(QWidget *parent = 0);
    void setActionText(QString action_text);

public Q_SLOTS:


private Q_SLOTS:


Q_SIGNALS:

};


#endif // BEHAVIOR_NOTIFICATION_H

