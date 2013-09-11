#ifndef WIDGET_H
#define WIDGET_H
#include <ros/ros.h>
#include <flor_control_msgs/FlorRobotStatus.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <flor_control_msgs/FlorRobotStateCommand.h>
#include <flor_control_msgs/FlorRobotFault.h>
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
    void robotfault(const flor_control_msgs::FlorRobotFault::ConstPtr& msg);
   // void recievedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);

protected:
    void timerEvent(QTimerEvent *event);
private Q_SLOTS:

    void on_connect_clicked();

    void on_start_clicked();

    void on_off_clicked();

    void on_low_clicked();

    void on_high_clicked();

    void on_send_mode_clicked();

    void enableStart();

private:

    Ui::Widget *ui;
    ros::NodeHandle nh;
    ros::Subscriber sub_state;
    ros::Subscriber sub_behav;
    ros::Subscriber sub_control;
    ros::Publisher pub;
    ros::Subscriber sub_fault;
    //ros::Subscriber status_msg_sub;
    QBasicTimer timer;
    int last_inlet_pr;
    int last_run_state;
    int last_air_sump_pressure;
    int last_pump_rpm;
   int last_pump_return_pressure;
   int last_pump_supply_pressure;
   int last_pump_time_meter;


};

#endif // WIDGET_H
