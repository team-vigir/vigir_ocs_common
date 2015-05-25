#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <QMainWindow>
#include <QWidget>
#include <QPoint>
#include <QPainter>
#include <QTableWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QFile>
#include <QComboBox>
#include <QtGui>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>

class completeRow
{
public:
    QTableWidgetItem* text;
    QTableWidgetItem* time;
    QTableWidgetItem* priority;

};



class robotStatus : public QWidget
{
    Q_OBJECT

public:
    explicit robotStatus(QWidget *parent = 0);
    ~robotStatus();
    //void msgRecieved( const )
    int getNumUnread();
    int getNumError();
    int getNumWarn();
    QString timeFromMsg(const ros::Time msg);
    void receivedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr msg);
    void clearCalledMsg(const std_msgs::Bool::ConstPtr msg);
    void clearTable();
    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr pose);
private Q_SLOTS:
    void on_clearButton_clicked();
    void on_msgTable_cellClicked(int row, int column);
    void on_radioButtons_updated();

private:
    int unreadMsgs;
    int numError;
    int numWarn;
    int maxRows;

    QTableWidget* msgTable;
    QPushButton* clearButton;
    QCheckBox* showOk;
    QCheckBox* showDebug;
    QCheckBox* showWarn;
    QCheckBox* showError;

    QStringList labels;
    std::string messagesPath;
    std::vector<completeRow*> messages;
    std::vector<std::string> errors;
    QFont bold;
    QFont normal;
    ros::Subscriber rosSubscriber;
    ros::Subscriber clearCalled;
    ros::Publisher callClear_pub;
    void loadFile();
    void updateTable();
    //QTreeWidget* jointTable;
    //QTreeWidgetItem joints[];
    //float positionLimits[];
    //float velocityLimits[];
    //float effortLimits[];

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;

Q_SIGNALS:
    void sendErrorData(QString,QString);
};

#endif // ROBOTSTATUS_H
