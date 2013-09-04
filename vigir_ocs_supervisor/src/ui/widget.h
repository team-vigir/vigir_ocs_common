#ifndef WIDGET_H
#define WIDGET_H
#include <ros/ros.h>
#include <flor_control_msgs/FlorRobotStatus.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <flor_control_msgs/FlorRobotStateCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>

#include <QWidget>
#include<QAbstractButton>
#include <QBasicTimer>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT
    
public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    void robotstate( const flor_control_msgs::FlorRobotStatus::ConstPtr& msg );
    void behavstate( const atlas_msgs::AtlasSimInterfaceState::ConstPtr& msg );
    void controlstate(const flor_control_msgs::FlorRobotStateCommand::ConstPtr& msg);


protected:
    void timerEvent(QTimerEvent *event);

private Q_SLOTS:

    void on_connect_clicked();

    void on_start_clicked();

    void on_off_clicked();

    void on_low_clicked();

    void on_high_clicked();

    void on_send_mode_clicked();

private:

    Ui::Widget *ui;
    ros::NodeHandle nh;
    ros::Subscriber sub_state;
    ros::Subscriber sub_behav;
    ros::Subscriber sub_control;
    ros::Publisher pub;

    QBasicTimer timer;

};

#endif // WIDGET_H
