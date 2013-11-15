#ifndef CameraViewWidget_H
#define CameraViewWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>

#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>

#include "camera_view.h"

namespace Ui {
class CameraViewWidget;
}

class CameraViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit CameraViewWidget(QWidget *parent = 0, rviz::VisualizationManager* context = 0);
    ~CameraViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    vigir_ocs::CameraView* getCameraView() { return camera_view_; }

private:
    Ui::CameraViewWidget*ui;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;

    vigir_ocs::CameraView* camera_view_;

Q_SIGNALS:
    void pitchChanged(int);

public Q_SLOTS:
    void updatePitch(int);
    void updateFeedFPS(int);
    void updateSelectedFPS(int);
    void scan();
    void isLocked();
    void alterChoices(int);

    void setFeedToSingleImage();
    void setAreaToSingleImage();

    void enableGroup(bool);

    void sliderValues(int lockedValue);

};

#endif // CameraViewWidget_H
