#ifndef MINIERROR_H
#define MINIERROR_H

#include <QMainWindow>
#include <ros/ros.h>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <QTableWidgetItem>
#include <QTableWidget>
#include <QPropertyAnimation>
#include <QTimer>
#include "robotStatus.h"

namespace Ui {
class MiniError;
}


class MiniError : public QMainWindow
{
    Q_OBJECT

public:
    ~MiniError();
    void setViewed();
    explicit MiniError(QWidget *parent = 0);


private:
    Ui::MiniError *ui;
    int newErrors;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);
    QPropertyAnimation * errorFadeIn;
    QPropertyAnimation * errorFadeOut;
    QTableWidgetItem * simTime;
    QTableWidgetItem * errorMessage;
    robotStatus * robStatus;
    QTimer* timer;
    bool visible;
    QRect * originalGeometry;

public Q_SLOTS:
    void receiveErrorData(QString,QString);
    void startActiveTimer();

private Q_SLOTS:
    void hideWindow();

};


#endif // MINIERROR
