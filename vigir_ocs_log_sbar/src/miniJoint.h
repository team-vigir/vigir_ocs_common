#ifndef MINIJOINT_H
#define MINIJOINT_H

#include <QMainWindow>
#include <flor_control_msgs/FlorControlModeCommand.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <QTableWidgetItem>
#include <QTableWidget>
#include <QPropertyAnimation>
#include <QTimer>
#include "jointList.h"

#define JOINT_OK 0
#define JOINT_WARN 1
#define JOINT_ERROR 2

namespace Ui {
class MiniJoint;
}


class MiniJoint : public QMainWindow
{
    Q_OBJECT

public:
    ~MiniJoint();
    explicit MiniJoint(QWidget *parent = 0);

private:
    Ui::MiniJoint *ui;
    void enterEvent(QEvent*);
    void leaveEvent(QEvent*);
    QPropertyAnimation * jointFadeIn;
    QPropertyAnimation * jointFadeOut;
    QTableWidgetItem * joint;
    QTableWidgetItem * jointStatus;
    jointList * jList;    
    QTimer* timer;
    bool visible;
    void itemClicked(QTableWidgetItem *);

public Q_SLOTS:
    void receiveJointData(int,QString);
    void startActiveTimer();

private Q_SLOTS:
    void hideWindow();
    void toggleJointListWindow();

};

#endif // MINIERROR
