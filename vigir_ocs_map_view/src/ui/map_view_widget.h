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

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

public Q_SLOTS:
    void hideWaypointButton();
    void hideJoystick();
    void requestMap();
    void requestOctomap();
    void requestPointCloud();
    void toggleMapConfig();
    void toggleRegionConfig();
    void hideRegionConfig();
    void hideMapConfig();
    
private:
    Ui::MapViewWidget *ui;
    StatusBar * statusBar;
    std::vector<int> keys_pressed_list_;

    QBasicTimer timer;
    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;

    QSignalMapper* stop_mapper_;
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);
    void timerEvent(QTimerEvent *event);

    MapRegionConfigure * mapRegionConfig;
    Region3DConfigure * region3dConfig;

    QPropertyAnimation* mapConfigFadeIn;
    QPropertyAnimation* mapConfigFadeOut;

    QPropertyAnimation* region3dConfigFadeIn;
    QPropertyAnimation* region3dConfigFadeOut;

    void loadButtonIcon(QPushButton* btn, QString image_name);
    void setupToolbar();
    QString icon_path_;


};

#endif // map_view_WIDGET_H
