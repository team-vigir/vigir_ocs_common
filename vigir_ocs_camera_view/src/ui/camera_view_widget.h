/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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

#include <vigir_ocs_msgs/OCSKeyEvent.h>

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

    //void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

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
    void loadButtonIconAndStyle(QPushButton* btn, QString image_name);
    Ui::CameraViewWidget*ui;   

    ros::NodeHandle n_;   

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
