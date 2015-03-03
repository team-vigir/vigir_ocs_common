#ifndef BEHAVIOR_NOTIFICATION_H
#define BEHAVIOR_NOTIFICATION_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <QtCore>
#include <QMessageBox>

namespace Ui
{
    class BehaviorNotification; //window name
}

class BehaviorNotification : public QMainWindow
{
    Q_OBJECT

private:
    Ui::BehaviorNotification *ui;
    bool confirmed_;


public:
    ~BehaviorNotification();
    explicit BehaviorNotification(QWidget *parent = 0);
    bool getConfirmed() { return confirmed_; }
    void setActionText(QString);

public Q_SLOTS:
    void confirm();


private Q_SLOTS:


Q_SIGNALS:

};


#endif // BEHAVIOR_NOTIFICATION_H

