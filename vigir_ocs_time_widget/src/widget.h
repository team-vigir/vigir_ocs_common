#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <QBasicTimer>

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
    void updateDate();
    void updateTime();
    void timerCallback(const ros::TimerEvent& e);
    void rosTimeRecieved();
	QString timeFromMsg(const ros::Time stamp);
    void rosTimeMsgRecieved(const rosgraph_msgs::Clock::ConstPtr& time);
    ~Widget();
    
protected:
    void timerEvent(QTimerEvent *event);
private:
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* robot_model_;
    ros::Subscriber joint_states;
    Ui::Widget *ui;
    ros::Subscriber rosTime;
    QBasicTimer timer;
};

#endif // WIDGET_H
