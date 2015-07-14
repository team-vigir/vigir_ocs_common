/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
@TODO_ADD_AUTHOR_INFO
#ifndef MAIN_CAMERA_VIEW_WIDGET_H
#define MAIN_CAMERA_VIEW_WIDGET_H

#include <QWidget>
#include <QPushButton>

#include <map>
#include <vector>
#include <algorithm>
#include <statusBar.h>
#include <ros/ros.h>
#include "base_3d_view.h"
#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include <std_msgs/Float32.h>
#include "notification_system.h"
#include "hotkey_manager.h"

namespace Ui
{
    class MainCameraViewWidget;
}

//forward declare to have both reference each other
class MainCameraContextMenu;

class MainCameraViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainCameraViewWidget(QWidget *parent = 0);
    ~MainCameraViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    virtual void updatePitch( const std_msgs::Float32::ConstPtr pitch);
    std::map<std::string,QWidget*> getViewsList(){return views_list_;}

public Q_SLOTS:
    void oneViewToggle();
    void fourViewToggle();
    void lockPitchUpdates();
    void sendPitch();
    void cameraInitialized();

private Q_SLOTS:
    void toggleSidebarVisibility();

private:
    Ui::MainCameraViewWidget *ui;

    MainCameraContextMenu * main_camera_context_menu_;


    std::map<std::string,QWidget*> views_list_;

    int views_initialized_;

    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

    QBasicTimer timer;
    virtual void timerEvent(QTimerEvent *event);

    QWidget* position_widget_;
    QPushButton* four_view_button_;
    QPushButton* one_view_button_;

    QString icon_path_;   

    ros::NodeHandle nh_;

    ros::Subscriber neck_pos_sub_;


    ros::Subscriber ocs_sync_sub_;
    void changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state);
    void synchronizeToggleButtons(const vigir_ocs_msgs::OCSSynchronize::ConstPtr msg);


    bool lock_pitch_slider_;

    StatusBar * statusBar;
    QSignalMapper* stop_mapper_;

    QPushButton* sidebar_toggle_;


    //Hotkey
    void addHotkeys();
    void getSingleImageMainViewHotkey();
    void setMainView5FPSHotkey();
    void closeSelectedHotkey();
    void increaseAlphaHotkey();
    void decreaseAlphaHotkey();

};

#endif // MAIN_CAMERA_VIEW_WIDGET_H
