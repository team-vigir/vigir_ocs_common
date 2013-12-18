#ifndef MAIN_CAMERA_VIEW_WIDGET_H
#define MAIN_CAMERA_VIEW_WIDGET_H

#include <QWidget>
#include <QPushButton>

#include <map>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

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
    void sendPitch();
    void cameraInitialized();
    
private:
    Ui::MainCameraViewWidget *ui;

    std::map<std::string,QWidget*> views_list_;
    int views_initialized_;

    QWidget* position_widget_;
    QPushButton* four_view_button_;
    QPushButton* one_view_button_;

    QString icon_path_;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle nh_;

    ros::Subscriber key_event_sub_;
	ros::Subscriber neck_pos_sub_;

};

#endif // MAIN_CAMERA_VIEW_WIDGET_H
