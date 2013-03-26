#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <QWidget>
#include <ros/subscriber.h>
#include <QPoint>
#include <QPainter>
#include <QTableWidget>
#include <QPushButton>
#include <flor_ocs_msgs/OCSRobotError.h>
#include <QFile>
#include <ros/time.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

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
    void recievedMessage(const flor_ocs_msgs::OCSRobotError::ConstPtr& msg);

private Q_SLOTS:
    void on_clearButton_clicked();
    void on_msgTable_cellClicked(int row, int column);
private:
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* robot_model_;
    int unreadMsgs;
    int numError;
    int numWarn;

    QTableWidget* msgTable;
    QPushButton* clearButton;
    QStringList labels;
    QStringList errors;

    std::vector<QTableWidgetItem*> messages;
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
