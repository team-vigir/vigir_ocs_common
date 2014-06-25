#ifndef JOYSTICKWIDGET_H
#define JOYSTICKWIDGET_H

#include <ros/ros.h>

#include <std_msgs/Int8.h>

#include <QMainWindow>
#include <QString>
#include <QWidget>
#include <QtGui>
#include <QComboBox>
#include "../Controller.h"

namespace Ui
{
    class JoystickWidget;
}

class JoystickWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit JoystickWidget(QWidget *parent = 0);
    ~JoystickWidget();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

public Q_SLOTS:
    void setDirection(QString s);
    void disableLeftLabel();
    void disableRightLabel();
    void selectTemplate();
    void populateTemplateComboBox(int tempId);

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

private:
    ros::Subscriber window_control_sub;
    QRect geometry_;

    Ui::JoystickWidget *ui;
    //handle multiple keys at once
    QSet<Qt::Key> keysPressed;
    void processKeys();
    vigir_ocs::Controller * controller;
    QSignalMapper * mapper;
    QString icon_path_;
};

#endif // JOYSTICKWIDGET_H
