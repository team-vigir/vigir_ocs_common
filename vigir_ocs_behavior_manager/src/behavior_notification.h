#ifndef BEHAVIOR_NOTIFICATION_H
#define BEHAVIOR_NOTIFICATION_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <QtCore>
#include <QMessageBox>
#include <QMouseEvent>

namespace Ui
{
    class BehaviorNotification; //window name
}

class BehaviorNotification : public QWidget
{
    Q_OBJECT

public:
    ~BehaviorNotification();
    explicit BehaviorNotification(QWidget *parent = 0);
    bool getConfirmed() { return confirmed_; }
    void setActionText(QString);

private:
    bool eventFilter(QObject* object,QEvent* event);

    Ui::BehaviorNotification *ui;
    bool confirmed_;


public Q_SLOTS:
    void confirm();
    void abort();


private Q_SLOTS:


Q_SIGNALS:
    void sendConfirmation(QString);
    void sendAbort(QString);
};


#endif // BEHAVIOR_NOTIFICATION_H

