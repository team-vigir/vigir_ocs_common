#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QBasicTimer>
#include <ros/ros.h>
#include <vigir_ocs_msgs/OCSLogging.h>
#include <std_msgs/String.h>
#include "boost/filesystem.hpp"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT
    
public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    
protected:
    void timerEvent(QTimerEvent *event);
public Q_SLOTS:
    void on_startButton_clicked();

    void on_stopButton_clicked();

    void on_experimentName_textChanged(const QString &arg1);


private:
    void sendMsg(bool run);
    Ui::Widget *ui;
    ros::Publisher ocs_logging_pub_;
    ros::Subscriber ocs_responce_sub_;
    ros::Publisher ocs_responce_pub_;
    QBasicTimer timer;
    bool first;
    std::string  experiment_directory_;
	void on_responce_recieved(const std_msgs::String::ConstPtr& msg);

};

#endif // WIDGET_H
