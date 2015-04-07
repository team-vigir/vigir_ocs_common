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
#include <flor_ocs_msgs/OCSKeyEvent.h>
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

    virtual void updatePitch( const std_msgs::Float32::ConstPtr &pitch);
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
    void synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg);


    bool lock_pitch_slider_;

    StatusBar * statusBar;
    QSignalMapper* stop_mapper_;

    QPushButton* sidebar_toggle_;


    //Hotkey
    void addHotkeys();
    void getSingleImageMainViewHotkey();
    void setMainView5FPSHotkey();
    void closeSelectedHotkey();


};

#endif // MAIN_CAMERA_VIEW_WIDGET_H
