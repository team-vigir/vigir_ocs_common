#ifndef MAP_VIEW_WIDGET_H
#define MAP_VIEW_WIDGET_H

#include <QWidget>

#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>

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
    
private:
    Ui::MapViewWidget *ui;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;

    ros::Subscriber key_event_sub_;

};

#endif // map_view_WIDGET_H
