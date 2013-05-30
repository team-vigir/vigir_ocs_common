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
    QString timeFromMsg(const ros::Time& stamp);
    ~Widget();
    
protected:
    void timerEvent(QTimerEvent *event);
private:

    Ui::Widget *ui;
    QBasicTimer timer;
};

#endif // WIDGET_H
