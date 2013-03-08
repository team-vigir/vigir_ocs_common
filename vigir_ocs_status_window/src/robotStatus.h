#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <QWidget>
//#include <ros/subscriber.h>
#include <QPoint>
#include <QPainter>
#include <QTableWidget>
#include <QPushButton>

class robotStatus : public QWidget
{
    Q_OBJECT

public:
    explicit robotStatus(QWidget *parent = 0);
    ~robotStatus();
    //void msgRecieved( const )
    int getNumUnread();

private Q_SLOTS:
    void on_clearButton_clicked();
    void on_msgTable_cellClicked(int row, int column);
private:
    QTableWidget* msgTable;
    int unreadMsgs;
    QPushButton* clearButton;
    QStringList labels;
    std::vector<QTableWidgetItem> messages;
    QFont bold;
    QFont normal;
    //ros::Subscriber joint_states;
    //QTreeWidget* jointTable;
    //QTreeWidgetItem joints[];
    //float positionLimits[];
    //float velocityLimits[];
    //float effortLimits[];
};

#endif // ROBOTSTATUS_H
