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

namespace Ui
{
    class MainCameraViewWidget;
}

class MainCameraViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainCameraViewWidget(QWidget *parent = 0);
    ~MainCameraViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    virtual void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);
    virtual void updatePitch( const std_msgs::Float32::ConstPtr &pitch);

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

    std::vector<contextMenuItem *> contextMenuElements;

    void addContextMenu();
    void systemCommandContext(std::string command);
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

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;
    ros::Subscriber neck_pos_sub_;
    ros::Publisher sys_command_pub_;

    ros::Subscriber ocs_sync_sub_;
    void changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state);
    void synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg);

    std_msgs::String sysCmdMsg;

    bool lock_pitch_slider_;

    StatusBar * statusBar;
    QSignalMapper* stop_mapper_;

    QPushButton* sidebar_toggle_;
};

#endif // MAIN_CAMERA_VIEW_WIDGET_H
