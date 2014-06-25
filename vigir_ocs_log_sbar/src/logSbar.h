#ifndef LOGSBAR_H
#define LOGSBAR_H

#include <QMainWindow>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/package.h>
#include <QPropertyAnimation>
#include <std_msgs/Int8.h>
#include "miniError.h"
#include "miniJoint.h"
#include <flor_ocs_msgs/WindowCodes.h>

namespace Ui {
class LogSbar;
}

class LogSbar : public QMainWindow
{
    Q_OBJECT

private:
    Ui::LogSbar *ui;
    int numError;
    void setJointStatus(int);
    ros::NodeHandle n_;
    QPropertyAnimation * errorFadeIn;
    QPropertyAnimation * errorFadeOut;
    QPropertyAnimation * jointFadeIn;
    QPropertyAnimation * jointFadeOut;
    MiniError * miniError;
    MiniJoint * miniJoint;
    ros::Publisher window_control_pub_;

public:
    ~LogSbar();
    explicit LogSbar(QWidget *parent = 0);
    void resetErrorCount();
    Ui::LogSbar * getUi()
    {
        return ui;
    }
    QPropertyAnimation* getErrorFadeIn()
    {
        return errorFadeIn;
    }
    QPropertyAnimation* getErrorFadeOut()
    {
        return errorFadeOut;
    }
    QPropertyAnimation* getJointFadeIn()
    {
        return jointFadeIn;
    }
    QPropertyAnimation* getJointFadeOut()
    {
        return jointFadeOut;
    }
    MiniError * getMiniError()
    {
        return miniError;
    }
    MiniJoint * getMiniJoint()
    {
        return miniJoint;
    }
    ros::Publisher getWindowPublisher()
    {
        return window_control_pub_;
    }
    void notifyMiniError();
    void notifyMiniJoint();


public Q_SLOTS:
    void receiveErrorData(QString time, QString message);
    void receiveJointData(int,QString);

private Q_SLOTS:
    void hideErrorWindow();
    void hideJointWindow();

Q_SIGNALS:
    void makeErrorActive();
    void makeJointActive();
};

//classes for subwidgets to detect mouse enter/leave events easily within logSbar
class ErrorWidget : public QWidget
{
    Q_OBJECT
public:
    ~ErrorWidget();
    explicit ErrorWidget(QWidget *parent = 0);

private:
    LogSbar* myParent;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);
    void mousePressEvent(QMouseEvent*);


};

class JointWidget : public QWidget
{
    Q_OBJECT
public:
    ~JointWidget();
    explicit JointWidget(QWidget *parent = 0);
private:
    LogSbar* myParent;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);
    void mousePressEvent(QMouseEvent*);
};

#endif // LOGSBAR_H

