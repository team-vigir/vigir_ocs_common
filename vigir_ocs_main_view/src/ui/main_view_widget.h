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
#ifndef MAIN_VIEW_WIDGET_H
#define MAIN_VIEW_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QSignalMapper>
#include <map>
#include <vector>
#include <algorithm>

//#include <ros/ros.h>

#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include <vigir_ocs_msgs/OCSControlMode.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

#include "statusBar.h"
#include "graspWidget.h"
#include <QSpacerItem>
#include <QBasicTimer>
#include "graspWidget.h"
#include <QPropertyAnimation>
#include <QFrame>
#include <boost/bind.hpp>

#include "ui/template_loader_widget.h"
#include "perspective_view.h"
#include "ortho_view.h"
#include <ros/package.h>
#include <rviz/visualization_manager.h>
#include <rviz/displays_panel.h>
#include <rviz/views_panel.h>
#include <QPropertyAnimation>
#include <vigir_ocs_msgs/WindowCodes.h>
#include <ui/footstep_config.h>
#include "notification_system.h"
#include "main_context_menu.h"
#include "behavior_relay.h"

#include "ui/ghost_control_widget.h"

namespace Ui
{
    class MainViewWidget;
}

class MainViewWidget : public QWidget
{   
    Q_OBJECT

public:
    explicit MainViewWidget(QWidget *parent = 0);
    virtual ~MainViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

   // void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    // process window control messages to update toggle buttons
    void processWindowControl(const std_msgs::Int8::ConstPtr visible);

    virtual void timerEvent(QTimerEvent *event);

    vigir_ocs::Base3DView* getPrimaryView() {return primary_view_;}
    Ui::MainViewWidget* getUi(){return ui;}    
    void useTorsoContextMenu();
    void snapGhostContextMenu();

public Q_SLOTS:
    void oneViewToggle();
    void fourViewToggle();
    void ft_sensorToggled(bool);
    void zero_leftPressed();
    void zero_rightPressed();
    void toggleWindow(int);    
    void setManipulationMode(int);        

    //consider protecting
    void updateContextMenu();
    void setObjectManipulationMode();
    void setObjectMode(int mode);
    void contextToggleWindow(int window);
    void graspWidgetToggle();
    void setCameraMode();
    void setWorldMode();
    void updateBehaviorNotifications();
    void update3dPlanning(bool);

protected Q_SLOTS:
    void toggleSidebarVisibility();    
    void hideGraspWidgets();
    void toggleFootstepConfig();
    void setLidarSpinRate(double spin_rate);

    void populateFootstepParameterSetBox(std::vector<std::string> parameter_sets);
    void setFootstepParameterSetBox(std::string parameter_set);

protected:
    void setupToolbar();
    void systemCommandContext(std::string command);
    void loadButtonIconAndStyle(QPushButton* btn, QString image_name);
    void modeCB(const vigir_ocs_msgs::OCSControlMode::ConstPtr msg);
    void changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state);
    void synchronizeToggleButtons(const vigir_ocs_msgs::OCSSynchronize::ConstPtr msg);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

    vigir_ocs_msgs::OCSControlMode controlModes;

    vigir_ocs::Base3DView* primary_view_;

    QWidget * notification_container_;
    QVBoxLayout* notification_layout_;
    BehaviorRelay* behavior_relay_;


    Ui::MainViewWidget *ui;
    MainViewContextMenu* main_view_context_menu_;

    std::map<std::string,QWidget*> views_list_;
    std::map<std::string,QWidget*> getViewsList(){return views_list_;}

    QWidget* position_widget_;
    QPushButton* four_view_button_;
    QPushButton* one_view_button_;
    QSignalMapper* toggle_mapper_;

    QString icon_path_;

    //std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;
    ros::Subscriber window_control_sub_;
    ros::Publisher window_control_pub_;
    //ros::Subscriber key_event_sub_;
    ros::Publisher ft_zero_pub_;


    ros::Publisher snap_ghost_pub_;
    ros::Publisher use_torso_pub_;

    ros::Publisher mode_pub_;
    ros::Subscriber mode_sub_;
    ros::Publisher interactive_marker_mode_pub_;

    ros::Subscriber ocs_sync_sub_;

    ros::Publisher lidar_spin_rate_pub_;


    StatusBar * statusBar;

    QBasicTimer timer;

    graspWidget * leftGraspWidget;
    graspWidget * rightGraspWidget;
    QPushButton * grasp_toggle_button_;
    QPropertyAnimation * graspFadeIn;
    QPropertyAnimation * graspFadeOut;    

    QWidget *graspContainer;

    QSignalMapper* stop_mapper_;

    QPushButton* sidebar_toggle_;

    FootstepConfigure* footstep_configure_widget_;
    QMenu footstep_menu_;


    GhostControlWidget * ghost_control_widget_;
    bool use_torso_checked_;

    std::vector<BehaviorNotification*> behavior_notifications_;
};

#endif // MAIN_VIEW_WIDGET_H
