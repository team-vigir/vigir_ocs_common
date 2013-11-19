#ifndef HANDOFFSETWIDGET_H
#define HANDOFFSETWIDGET_H

#include <QWidget>
#include <QSettings>
#include <QDir>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

//#include <graspWidget.h>

namespace Ui {
class handOffsetWidget;
}

class handOffsetWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit handOffsetWidget(QWidget *parent = 0);
    ~handOffsetWidget();
    
public Q_SLOTS:
    void on_roll_inc_clicked();
    void on_roll_dec_clicked();
    void on_pitch_inc_clicked();
    void on_pitch_dec_clicked();
    void on_yaw_inc_clicked();
    void on_yaw_dec_clicked();
    void on_load_offset_clicked();
    void on_save_offset_clicked();

private:
    Ui::handOffsetWidget *ui;

    std::string hand_;

    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;

    ros::NodeHandle nh_;
    ros::Publisher l_hand_template_offset_pub_;
    ros::Publisher r_hand_template_offset_pub_;
    geometry_msgs::PoseStamped hand_offset;

    QSettings saved_offsets;

    void calc_offset();
};

#endif // HANDOFFSETWIDGET_H
