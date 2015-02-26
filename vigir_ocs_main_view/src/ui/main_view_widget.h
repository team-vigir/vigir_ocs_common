#ifndef MAIN_VIEW_WIDGET_H
#define MAIN_VIEW_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QSignalMapper>
#include <map>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <flor_ocs_msgs/OCSKeyEvent.h>
#include <flor_ocs_msgs/OCSControlMode.h>
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
#include <flor_ocs_msgs/WindowCodes.h>
#include "footstep_config.h"
#include "notification_system.h"
#include "main_context_menu.h"

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

   // void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    // process window control messages to update toggle buttons
    void processWindowControl(const std_msgs::Int8::ConstPtr& visible);

    virtual void timerEvent(QTimerEvent *event);

    vigir_ocs::Base3DView* getPrimaryView() {return primary_view_;}
    Ui::MainViewWidget* getUi(){return ui;}
    GhostControlWidget* getGhostControlWidget() {return ghost_control_widget_;}
    void useTorsoContextMenu();

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

protected Q_SLOTS:
    void toggleSidebarVisibility();    
    void hideGraspWidgets();
    void populateFootstepParameterSetBox(std::vector<std::string> parameter_sets);
    void toggleFootstepConfig();
    void setLidarSpinRate(double spin_rate);



protected:
    void setupToolbar();
    void systemCommandContext(std::string command);
    void loadButtonIcon(QPushButton* btn, QString image_name);
    void modeCB(const flor_ocs_msgs::OCSControlMode::ConstPtr& msg);
    void changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state);
    void synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);

    flor_ocs_msgs::OCSControlMode controlModes;

    vigir_ocs::Base3DView* primary_view_;

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
};

#endif // MAIN_VIEW_WIDGET_H
