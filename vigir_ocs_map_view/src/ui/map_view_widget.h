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
#ifndef MAP_VIEW_WIDGET_H
#define MAP_VIEW_WIDGET_H

#include <QWidget>

#include <vector>
#include <algorithm>
#include "statusBar.h"
#include <ros/ros.h>
#include "map_region_configure_widget.h"
#include "region_3d_configure_widget.h"
#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include <QPropertyAnimation>
#include "vigir_ocs_msgs/OCSSynchronize.h"
#include "notification_system.h"
#include "hotkey_manager.h"
#include <ui/footstep_config.h>

namespace Ui
{
    class MapViewWidget;
}

class MapViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit MapViewWidget(QWidget *parent = 0);
    ~MapViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );   

    // process window control messages to update toggle buttons
    void processWindowControl(const std_msgs::Int8::ConstPtr& visible);

public Q_SLOTS:
    void hideWaypointButton();
    void hideJoystick();
    void requestMap();
    void requestOctomap();
    void requestPointCloud();
    void toggleMapConfig();
    void toggleRegionConfig();
    void toggleWindow(int window);
    void toggleFootstepConfig();

    void populateFootstepParameterSetBox(std::vector<std::string> parameter_sets);
    void setFootstepParameterSetBox(std::string parameter_set);
    void update3dPlanning(bool);

private Q_SLOTS:
    void toggleSidebarVisibility();
    
private:
    Ui::MapViewWidget *ui;
    StatusBar * statusBar;    

    QBasicTimer timer;
    ros::NodeHandle n_;   

    void synchronizeToggleButtons(const vigir_ocs_msgs::OCSSynchronize::ConstPtr &msg);
    void changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state);
    ros::Subscriber ocs_sync_sub_;
    ros::Publisher  ocs_sync_pub_;

    QSignalMapper* stop_mapper_;
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);    

    virtual void timerEvent(QTimerEvent *event);

    MapRegionConfigure * mapRegionConfig;
    Region3DConfigure * region3dConfig;

    void loadButtonIconAndStyle(QPushButton* btn, QString image_name);
    void setupToolbar();
    QString icon_path_;

    QMenu regionMenu;
    QMenu mapMenu;
    QMenu footstep_menu_;

    QPushButton* sidebar_toggle_;

    //Hotkey stuff
    void addHotkeys();
    void unfilteredHotkey();
    void stereoHotkey();

    QSignalMapper* toggle_mapper_;
    FootstepConfigure* footstep_configure_widget_;

    ros::Subscriber window_control_sub_;
    ros::Publisher window_control_pub_;

};

#endif // map_view_WIDGET_H
