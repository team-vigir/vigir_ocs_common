#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <QWidget>
#include <ros/subscriber.h>
#include <QPoint>
#include <QPainter>
#include <QTableWidget>
#include <QPushButton>
#include <QCheckBox>
#include <flor_ocs_msgs/OCSRobotStatus.h>
#include <QFile>
#include <ros/time.h>
#include <QComboBox>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

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
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* robot_model_;

    int unreadMsgs;
    int numError;
    int numWarn;

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
    //QTreeWidget* jointTable;
    //QTreeWidgetItem joints[];
    //float positionLimits[];
    //float velocityLimits[];
    //float effortLimits[];
};

#endif // ROBOTSTATUS_H
