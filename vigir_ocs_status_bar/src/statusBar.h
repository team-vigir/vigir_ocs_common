#ifndef STATUSBAR_H
#define STATUSBAR_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <QBasicTimer>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <QTableWidgetItem>
#include <QTableWidget>
#include "glancehubSbar.h"
#include "logSbar.h"

namespace Ui {
class StatusBar;
}


class StatusBar : public QMainWindow
{
    Q_OBJECT

public:
    ~StatusBar();
    explicit StatusBar(QWidget *parent = 0);

private:
    Ui::StatusBar *ui;
    glancehubSbar * glanceSbar;
    LogSbar * logSbar;
   
public Q_SLOTS:
    void receivePositionText(QString s);

};

#endif // STATUSBAR_H
