#ifndef joint_limit_H
#define joint_limit_H

#include <QWidget>
#include <QBasicTimer>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/publisher.h>

#include <std_msgs/Int8.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>

namespace Ui {
class joint_limit;
}

class joint_limit : public QWidget
{
    Q_OBJECT

public:
    explicit joint_limit(QWidget *parent = 0);
    ~joint_limit();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private:
    ros::Subscriber window_control_sub;
    QRect geometry_;

    Ui::joint_limit *ui;
    ros::NodeHandle nh_;
    ros::Publisher constraints_pub_;

    float lbzMinVal;
    float lbzMaxVal;
    float mbyMinVal;
    float mbyMaxVal;
    float ubxMinVal;
    float ubxMaxVal;

    QBasicTimer timer;

    std::vector<int> keys_pressed_list_;

    ros::Subscriber key_event_sub_;
public Q_SLOTS:
    void on_apply_clicked();
    void on_lbzMin_sliderReleased();
    void on_lbzMax_sliderReleased();
    void on_mbyMin_sliderReleased();
    void on_mbyMax_sliderReleased();
    void on_ubxMin_sliderReleased();
    void on_ubxMax_sliderReleased();
    void on_Presets_comboBox_currentIndexChanged(int index);
    void on_lock_yaw__toggled(bool checked);
    void on_lock_pitch__toggled(bool checked);
    void on_lock_roll__toggled(bool checked);
protected:
    void timerEvent(QTimerEvent *event);
};

#endif // joint_limit_H
