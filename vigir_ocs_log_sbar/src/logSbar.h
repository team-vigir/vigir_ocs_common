#ifndef LOGSBAR_H
#define LOGSBAR_H

#include <QMainWindow>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/package.h>
#include <QPropertyAnimation>
#include "miniError.h"
#include "miniJoint.h"

namespace Ui {
class LogSbar;
}

class LogSbar : public QMainWindow
{
    Q_OBJECT

public:
    ~LogSbar();
    explicit LogSbar(QWidget *parent = 0);
    void setErrorCount(int);
    Ui::LogSbar * getUi();    
    QPropertyAnimation * errorFadeIn;
    QPropertyAnimation * errorFadeOut;
    QPropertyAnimation * jointFadeIn;
    QPropertyAnimation * jointFadeOut;
    MiniError * miniError;
    MiniJoint * miniJoint;
    void notifyMiniError();
    void notifyMiniJoint();

private:    
    Ui::LogSbar *ui;
    int numError;    
    void setJointStatus(int);

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

