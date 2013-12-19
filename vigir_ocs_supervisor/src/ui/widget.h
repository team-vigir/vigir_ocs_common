#ifndef WIDGET_H
#define WIDGET_H
#include <ros/ros.h>
#include <flor_ocs_msgs/RobotStatusCodes.h>
#include <flor_control_msgs/FlorRobotStatus.h>
#include<flor_ocs_msgs/OCSRobotStatus.h>
#include <flor_control_msgs/FlorControlMode.h>
#include <flor_control_msgs/FlorRobotStateCommand.h>
#include <flor_control_msgs/FlorRobotFault.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <flor_atlas_msgs/AtlasHandPower.h>
#include<QTableWidget>
#include<QTableWidgetItem>
#include <QWidget>
#include<QAbstractButton>
#include <QBasicTimer>

namespace Ui {
class Widget;
class completeRow;
}
class completeRow
{
public:
    QTableWidgetItem* text;
    QTableWidgetItem* time;
    QTableWidgetItem* priority;

};
class Widget : public QWidget
{
    Q_OBJECT

public:
    int count_row;
    std::vector<completeRow*> messages;
    std::vector<std::string> errors;
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    void robotstate( const flor_control_msgs::FlorRobotStatus::ConstPtr& msg );
    void behavstate( const flor_control_msgs::FlorControlMode::ConstPtr& msg );
    void controlstate(const flor_control_msgs::FlorRobotStateCommand::ConstPtr& msg);
    void robotfault(const flor_control_msgs::FlorRobotFault::ConstPtr& msg);
    void receivedMessage(const flor_ocs_msgs::OCSRobotStatus::ConstPtr& msg);
    QString timeFromMsg(const ros::Time msg);
    void loadFile();

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

    void on_enableapplyhand_stateChanged(int arg1);

    void on_applyhand_clicked();

private:

    Ui::Widget *ui;
    ros::NodeHandle nh;
    ros::Subscriber sub_state;
    ros::Subscriber sub_behav;
    ros::Subscriber sub_control;
    ros::Publisher pub;
    ros::Publisher pub_hand_power ;
    ros::Subscriber sub_fault;
    ros::Subscriber status_msg_sub;
    QBasicTimer timer;
    int   last_run_state;
    float avg_inlet_pr;
    float avg_air_sump_pressure;
    float avg_pump_rpm;
    float avg_pump_return_pressure;
    float avg_pump_supply_pressure;
    float avg_pump_supply_temperature;
    float avg_motor_temperature      ;
    float avg_motor_driver_temp      ;
    int unreadMsgs;
    int numError;
    int numWarn;
    int maxRows;
    QFont bold;
    QFont normal;
    QStringList labels;
    std::string messagesPath;
    double      filter_rate_;


};

#endif // WIDGET_H
