#ifndef CameraViewWidget_H
#define CameraViewWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QMenu>
#include <QPainter>
#include <QSlider>

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
    explicit CameraViewWidget(QWidget *parent = 0, vigir_ocs::Base3DView* copy_from = 0);
    ~CameraViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    vigir_ocs::CameraView* getCameraView() { return camera_view_; }

Q_SIGNALS:
    void pitchChanged(int);

public Q_SLOTS:
    void updatePitch(int);
    void updateCurrentPitch(int);
    void updateFeedFPS(int);
    void updateSelectedFPS(int);
    void scan();
    void alterChoices(int);

    void setFeedToSingleImage();
    void setAreaToSingleImage();

    void sliderValues(int lockedValue);

    void imageFeedSliderChanged(int);
    void imageFeedSliderReleased();
    void imageFeedButtonClicked();

    void areaFeedSliderChanged(int);
    void areaFeedSliderReleased();
    void areaFeedButtonClicked();

    void transparencySliderChanged(int);
    void transparencySliderReleased();
    void transparencyButtonClicked();

private:
    Ui::CameraViewWidget*ui;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;

    vigir_ocs::CameraView* camera_view_;

    // "context" menus for our buttons
    QMenu image_feed_menu_;
    QMenu area_feed_menu_;
    QMenu image_transparency_menu_;

    QSlider *feed_slider;
    QSlider *area_slider;
    QSlider *transparency_slider;

    QString icon_path_;

};

#endif // CameraViewWidget_H
