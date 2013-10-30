#ifndef joint_limit_H
#define joint_limit_H

#include <QWidget>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <QBasicTimer>

namespace Ui {
class joint_limit;
}

class joint_limit : public QWidget
{
    Q_OBJECT

public:
    explicit joint_limit(QWidget *parent = 0);
    ~joint_limit();

private:
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
public Q_SLOTS:
    void on_apply_clicked();
    void on_lbzMin_sliderReleased();
    void on_lbzMax_sliderReleased();
    void on_mbyMin_sliderReleased();
    void on_mbyMax_sliderReleased();
    void on_ubxMin_sliderReleased();
    void on_ubxMax_sliderReleased();
    void on_Presets_comboBox_currentIndexChanged(int index);
protected:
    void timerEvent(QTimerEvent *event);
};

#endif // joint_limit_H
