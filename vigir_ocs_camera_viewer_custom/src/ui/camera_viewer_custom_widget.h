#ifndef CameraViewerCustomWidget_H
#define CameraViewerCustomWidget_H


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


namespace Ui {
class CameraViewerCustomWidget;
}

class CameraViewerCustomWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit CameraViewerCustomWidget(QWidget *parent = 0);
    ~CameraViewerCustomWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);
private:
    Ui::CameraViewerCustomWidget*ui;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;

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

#endif // CameraViewerCustomWidget_H
