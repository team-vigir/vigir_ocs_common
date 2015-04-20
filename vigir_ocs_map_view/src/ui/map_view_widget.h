#ifndef MAP_VIEW_WIDGET_H
#define MAP_VIEW_WIDGET_H

#include <QWidget>

#include <vector>
#include <algorithm>
#include "statusBar.h"
#include <ros/ros.h>
#include "map_region_configure_widget.h"
#include "region_3d_configure_widget.h"
#include <flor_ocs_msgs/OCSKeyEvent.h>
#include <QPropertyAnimation>
#include "flor_ocs_msgs/OCSSynchronize.h"
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

    void synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg);
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
