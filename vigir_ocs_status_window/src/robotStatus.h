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

#include <ros/subscriber.h>
#include <ros/time.h>

#include <flor_ocs_msgs/OCSRobotStatus.h>

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
    void recievedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);

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

    std::vector<completeRow*> messages;
    std::vector<std::string> errors;
    QFont bold;
    QFont normal;
    ros::Subscriber rosSubscriber;
    QFile messagesFile;
    void loadFile();
    void updateTable();
    //QTreeWidget* jointTable;
    //QTreeWidgetItem joints[];
    //float positionLimits[];
    //float velocityLimits[];
    //float effortLimits[];

protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // ROBOTSTATUS_H
